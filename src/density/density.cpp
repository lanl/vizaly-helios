/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Author: Hoby Rakotoarivelo
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "density/density.h"
/* -------------------------------------------------------------------------- */

Density::Density(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm)
  : json_path(in_path),
    my_rank(in_rank),
    nb_ranks(in_nb_ranks),
    comm(in_comm) {
  assert(nb_ranks > 0);
  nlohmann::json json;
  std::string buffer;

  std::ifstream file(json_path);
  assert(file.is_open());
  assert(file.good());

  // parse params and do basic checks
  file >> json;

  assert(json.count("hacc"));
  assert(json["hacc"].count("input"));
  assert(json["hacc"].count("output"));

  assert(json.count("density"));
  assert(json["density"].count("inputs"));
  assert(json["density"].count("extents"));
  assert(json["density"]["extents"].count("min"));
  assert(json["density"]["extents"].count("max"));
  assert(json["density"].count("nb_bins"));

  assert(json.count("plots"));
  assert(json["plots"].count("density"));
  assert(json["plots"].count("buckets"));

  // retrieve number of cells per axis
  int const c_min = json["density"]["extents"]["min"];
  int const c_max = json["density"]["extents"]["max"];
  cells_per_axis = 1 + c_max - c_min;
  assert(cells_per_axis > 0);

  // dispatch files to MPI ranks
  int partition_size = json["density"]["inputs"].size();
  bool rank_mismatch = (partition_size < nb_ranks) or (partition_size % nb_ranks != 0);

  if (nb_ranks == 1 or not rank_mismatch) {
    int offset = static_cast<int>(partition_size / nb_ranks);
    assert(offset);

    local_rho_count = 0;
    for (int i = 0; i < offset; ++i) {
      int index = i + my_rank * offset;
      auto&& current = json["density"]["inputs"][index];
      inputs.emplace_back(current["data"], current["count"]);
      std::cout << "rank["<< my_rank <<"]: \""<< inputs.back().first << "\""<< std::endl;
      local_rho_count += inputs.back().second;
    }

    // resize buffer
    density_field.resize(local_rho_count);
    // retrieve the total number of elems
    MPI_Allreduce(&local_rho_count, &total_rho_count, 1, MPI_LONG, MPI_SUM, comm);

  } else
    throw std::runtime_error("mismatch on number of ranks and data partition");

  nb_bins = json["density"]["bins"];
  assert(nb_bins > 0);

  histogram.resize(nb_bins);
  buckets.resize(nb_bins);
  bits.resize(nb_bins);
  setNumberBits();

  // plots
  output_plot = json["plots"]["density"];
  output_bucket = json["plots"]["buckets"];

  // set the HACC IO manager
  ioMgr = std::make_unique<HACCDataLoader>();
  input_hacc  = json["hacc"]["input"];
  output_hacc = json["hacc"]["output"];

}

/* -------------------------------------------------------------------------- */
void Density::cacheData() {

  bool const master_rank = (my_rank == 0);

  assert(not input_hacc.empty());
  assert(not inputs.empty());

  // step 1: load particle file

  ioMgr->init(input_hacc, comm);
  ioMgr->saveParams();
  ioMgr->setSave(true);

  std::string const columns[] = {"x", "y", "z", "vx", "vy", "vz", "id" };

  if (master_rank)
    std::cout << "Caching particle data ... " << std::flush;

  for (int i = 0; i < 7; ++i) {
    if (ioMgr->load(columns[i])) {

      // retrieve debug infos
      if (master_rank) {
        std::cout << ioMgr->getDataInfo();
        if (i == 0)
          std::cout << ioMgr->getLog();
      }

      // store actual data
      auto const n = ioMgr->getNumElements();
      if (columns[i] != "id") {
        auto const data = static_cast<float*>(ioMgr->data);
        if (i < 3) {
          coords[i].resize(n);
          std::copy(data, data + n, coords[i].data());
        } else {
          velocs[i].resize(n);
          std::copy(data, data + n, velocs[i].data());
        }
      } else {
        auto const data = static_cast<long*>(ioMgr->data);
        index.resize(n);
        std::copy(data, data + n, index.data());
      }

      // finalize
      ioMgr->close();
    }
    MPI_Barrier(comm);
  }

  // update particle count
  local_particles = ioMgr->getNumElements();
  MPI_Allreduce(&local_particles, &total_particles, 1, MPI_LONG, MPI_SUM, comm);

  // cache data extents
  for (int i = 0; i < 3; ++i) {
    coords_min[i] = static_cast<float>(ioMgr->data_extents[i].first);
    coords_max[i] = static_cast<float>(ioMgr->data_extents[i].second);
  }

  if (master_rank) {
    std::cout << "done." << std::endl;
    std::cout << "Caching density data ... " << std::flush;
  }

  // step 2: load density file
  long offset = 0;
  long count = 0;
  std::string path;

  for (auto&& current : inputs) {
    std::tie(path, count) = current;

    std::ifstream file(path, std::ios::binary);
    assert(file.good());

    auto buffer = reinterpret_cast<char *>(density_field.data() + offset);
    auto size = count * sizeof(float);

    file.seekg(0, std::ios::beg);
    file.read(buffer, size);
    file.close();

    // update offset
    offset += count;
  }

  if (master_rank)
    std::cout << "done." << std::endl;

}


/* -------------------------------------------------------------------------- */
void Density::computeFrequencies() {

  if (my_rank == 0)
    std::cout << "Computing frequencies ... " << std::flush;

  assert(local_rho_count);
  assert(total_rho_count);

  // determine data values extents
  total_rho_max = 0.0;
  total_rho_min = 0.0;
  local_rho_min = *std::min_element(density_field.data(), density_field.data() + local_rho_count);
  local_rho_max = *std::max_element(density_field.data(), density_field.data() + local_rho_count);
  MPI_Allreduce(&local_rho_max, &total_rho_max, 1, MPI_DOUBLE, MPI_MAX, comm);
  MPI_Allreduce(&local_rho_min, &total_rho_min, 1, MPI_DOUBLE, MPI_MIN, comm);

  // compute histogram of values
  long local_histo[nb_bins];
  auto total_histo = histogram.data();

  std::fill(local_histo, local_histo + nb_bins, 0);
  std::fill(total_histo, total_histo + nb_bins, 0);

  double const range = total_rho_max - total_rho_min;
  double const capacity = range / nb_bins;

  for (auto k = 0; k < local_rho_count; ++k) {
    double relative_value = (density_field[k] - total_rho_min) / range;
    int bin_index = static_cast<int>((range * relative_value) / capacity);

    if (bin_index >= nb_bins)
      bin_index--;

    local_histo[bin_index]++;
  }

  MPI_Allreduce(local_histo, total_histo, nb_bins, MPI_LONG, MPI_SUM, comm);

  if (my_rank == 0) {
    dumpHistogram();
    std::cout << "done." << std::endl;
    std::cout << "= number of particles: " << total_rho_count << std::endl;
    std::cout << "= number of bins: " << nb_bins << std::endl;
    std::cout << "= density range: [" << total_rho_min << ", " << total_rho_max << "]" << std::endl;
    std::cout << "= histogram file: '" << output_plot << ".dat'" << std::endl;
  }

  MPI_Barrier(comm);
}

/* -------------------------------------------------------------------------- */
void Density::dumpHistogram() {

  double const width = (total_rho_max - total_rho_min) / static_cast<double>(nb_bins);

  std::ofstream file(output_plot + ".dat", std::ios::trunc);
  assert(file.is_open());
  assert(file.good());

  file << "# bins: " << std::to_string(nb_bins) << std::endl;
  file << "# col 1: density range" << std::endl;
  file << "# col 2: particle count" << std::endl;

  int k = 1;
  for (auto&& value : histogram) {
    file << total_rho_min + (k * width) << "\t" << value << std::endl;
    k++;
  }

  file.close();
}


/* -------------------------------------------------------------------------- */
void Density::setNumberBits() {

  // temporary
  bits[0] = 22;
  for (int i =   1; i <  20; ++i) bits[i] = 23;
  for (int i =  20; i <  50; ++i) bits[i] = 24;
  for (int i =  50; i < 100; ++i) bits[i] = 25;
  for (int i = 100; i < nb_bins; ++i) bits[i] = 26;
}

/* -------------------------------------------------------------------------- */
long Density::deduceDensityIndex(const float* particle) const {

  float range[3];
  float shifted[3];
  auto const n_cells_axis = static_cast<float>(cells_per_axis);

  // step 1: shift particle coordinates and compute related range
  for (int i = 0; i < 3; ++i) {
    shifted[i] = particle[i] - coords_min[i];
    range[i] = coords_max[i] - coords_min[i];
  }

  // step 2: physical coordinates to logical coordinates
  auto i = static_cast<int>(std::floor(shifted[0] * n_cells_axis / range[0]));
  auto j = static_cast<int>(std::floor(shifted[1] * n_cells_axis / range[1]));
  auto k = static_cast<int>(std::floor(shifted[2] * n_cells_axis / range[2]));

  // step 3: logical coordinates to flat array index
  return i + j * cells_per_axis + k * cells_per_axis * cells_per_axis;
}

/* -------------------------------------------------------------------------- */
int Density::deduceBucketIndex(float const& rho) const {

  assert(rho < total_rho_max);
  auto const coef = rho / (local_rho_max - local_rho_min);
  auto const bucket_index = std::max(static_cast<int>(std::floor(coef * float(nb_bins))) - 1, 0);
  assert(bucket_index < nb_bins);
  return bucket_index;
}

/* -------------------------------------------------------------------------- */
void Density::bucketParticles() {

  if (my_rank == 0)
    std::cout << "Bucketing particles ... " << std::flush;

  for (int i = 0; i < local_particles; ++i) {
    float particle[] = { coords[0][i], coords[1][i], coords[2][i] };
    auto const density_index = deduceDensityIndex(particle);
    assert(density_index < local_rho_count);
    auto const bucket_index  = deduceBucketIndex(density_field[density_index]);
   // if (my_rank == 0)
   //   std::cout << "density_index: " << density_index << ", bucket_index: " << bucket_index << std::endl;
    assert(bucket_index < nb_bins);
    // copy data in correct bucket
    //buckets[bucket_index].emplace_back(i);
  }

  MPI_Barrier(comm);

  //dumpBucketDistrib();

  if (my_rank == 0) {
    std::cout << "done" << std::endl;
    // for debug purposes
//    auto const width = (total_rho_max - total_rho_min) / static_cast<double>(nb_bins);
//    for (int i = 0; i < nb_bins; ++i) {
//      auto const rho_max = total_rho_min + (i * width);
//      std::printf("bucket[%d]: rho_max=%.f, nb_particles=%lu\n", i, rho_max, buckets[i].size());
//    }
  }
}

/* -------------------------------------------------------------------------- */
void Density::dumpBucketDistrib() {

  long local_count[nb_bins];
  long total_count[nb_bins];
  for (int i = 0; i < nb_bins; ++i)
    local_count[i] = buckets[i].size();

  MPI_Reduce(local_count, total_count, nb_bins, MPI_LONG, MPI_SUM, 0, comm);

  if (my_rank == 0) {
    std::ofstream file(output_bucket + ".dat", std::ios::trunc);
    assert(file.is_open());
    assert(file.good());

    file << "# bins: " << std::to_string(nb_bins) << std::endl;
    file << "# col 1: bin " << std::endl;
    file << "# col 2: particle count" << std::endl;
    for (int i = 0; i < nb_bins; ++i)
      file << i << "\t" << total_count[i] << std::endl;
    file.close();
  }

  MPI_Barrier(comm);
}

/* -------------------------------------------------------------------------- */
void Density::process(int step) {

  assert(step < 6);
  assert(not buckets.empty());

  float* data = nullptr;
  switch (step) {
    case 0: data = coords[0].data(); break;
    case 1: data = coords[1].data(); break;
    case 2: data = coords[2].data(); break;
    case 3: data = velocs[0].data(); break;
    case 4: data = velocs[1].data(); break;
    case 5: data = velocs[2].data(); break;
    default: break;
  }

  if (my_rank == 0)
    std::cout << "Inflate and deflate data ... " << std::flush;

  auto kernel = CompressorFactory::create("fpzip");
  size_t local_bytes[] = {0, 0};
  size_t total_bytes[] = {0, 0};

  std::vector<float> dataset;
  decompressed[step].reserve(local_particles);

  for (int j = 0; j < nb_bins; ++j) {
    // retrieve number of particles for this bucket
    auto nb_elems = buckets[j].size();

    // step 1: create dataset according to computed bin.
    dataset.reserve(nb_elems);
    for (auto&& particle_index : buckets[j])
      dataset.emplace_back(data[particle_index]);

    // step 2: inflate agregated dataset and release memory
    void* raw_inflate = nullptr;
    void* raw_deflate = nullptr;

    kernel->parameters["bits"] = std::to_string(bits[j]);
    kernel->compress(dataset.data(), raw_inflate, "float", sizeof(float), &nb_elems);
    dataset.clear();

    // update compression metrics
    local_bytes[0] += kernel->getBytes();
    local_bytes[1] += nb_elems * sizeof(float);

    // step 3: deflate data and store it
    kernel->decompress(raw_inflate, raw_deflate, "float", sizeof(float), &nb_elems);
    for (int k = 0; k < nb_elems; ++k)
      decompressed[step].emplace_back(static_cast<float*>(raw_deflate)[k]);
  }

  MPI_Barrier(comm);
  MPI_Allreduce(local_bytes+0, total_bytes+0, 1, MPI_UNSIGNED_LONG, MPI_SUM, comm);
  MPI_Allreduce(local_bytes+1, total_bytes+1, 1, MPI_UNSIGNED_LONG, MPI_SUM, comm);

  if (my_rank == 0) {
    std::cout << "done" << std::endl;
    auto const compression_ratio = static_cast<float>(total_bytes[1]) / static_cast<float>(total_bytes[0]);
    std::cout << "= total inflate size: " << total_bytes[0] << std::endl;
    std::cout << "= total deflate size: " << total_bytes[1] << std::endl;
    std::cout << "= compression ratio: " << compression_ratio << std::endl;
  }

  switch (step) {
    case 0: coords[0].clear(); coords[0].shrink_to_fit(); break;
    case 1: coords[1].clear(); coords[1].shrink_to_fit(); break;
    case 2: coords[2].clear(); coords[2].shrink_to_fit(); break;
    case 3: velocs[0].clear(); velocs[0].shrink_to_fit(); break;
    case 4: velocs[1].clear(); velocs[1].shrink_to_fit(); break;
    case 5: velocs[2].clear(); velocs[2].shrink_to_fit(); break;
    default: break;
  }

  MPI_Barrier(comm);
}

/* -------------------------------------------------------------------------- */
void Density::dump() {

  if (my_rank == 0)
    std::cout << "Dumping dataset ... " << std::flush;

  // step 0: ease memory pressure by releasing unused data
  density_field.clear();
  density_field.shrink_to_fit();
  histogram.clear();
  histogram.shrink_to_fit();
  bits.clear();
  bits.shrink_to_fit();

  // step 1: sort particles indices
  std::vector<long> sorted_indices;
  sorted_indices.reserve(local_particles);

  for (auto&& bucket : buckets) {
    for (auto&& i : bucket) {
      sorted_indices.emplace_back(i);
    }
  }
  // release memory
  index.clear();
  index.shrink_to_fit();
  buckets.clear();
  buckets.shrink_to_fit();
  MPI_Barrier(comm);

  // step 2: prepare dataset partition and header
  int periods[3] = {0, 0, 0};
  auto dim_size = ioMgr->mpi_partition;
  MPI_Cart_create(comm, 3, dim_size, periods, 0, &comm);

  // init writer and open file
  gio::GenericIO gioWriter(comm, output_hacc);
  gioWriter.setNumElems(local_particles);

  // init physical params
  for (int d = 0; d < 3; ++d) {
    gioWriter.setPhysOrigin(ioMgr->phys_orig[d], d);
    gioWriter.setPhysScale(ioMgr->phys_scale[d], d);
  }

  MPI_Barrier(comm);

  unsigned const flags[] = {
    gio::GenericIO::VarHasExtraSpace,
    gio::GenericIO::VarHasExtraSpace|gio::GenericIO::VarIsPhysCoordX,
    gio::GenericIO::VarHasExtraSpace|gio::GenericIO::VarIsPhysCoordY,
    gio::GenericIO::VarHasExtraSpace|gio::GenericIO::VarIsPhysCoordZ
  };

  gioWriter.addVariable( "x", decompressed[0].data(), flags[1]);
  gioWriter.addVariable( "y", decompressed[1].data(), flags[2]);
  gioWriter.addVariable( "z", decompressed[2].data(), flags[3]);
  gioWriter.addVariable("vx", decompressed[3].data(), flags[0]);
  gioWriter.addVariable("vy", decompressed[4].data(), flags[0]);
  gioWriter.addVariable("vz", decompressed[5].data(), flags[0]);
  gioWriter.addVariable("id", sorted_indices.data(), flags[0]);
  gioWriter.write();

  // release memory
  for (auto& dataset : decompressed) {
    dataset.clear();
    dataset.shrink_to_fit();
  }

  MPI_Barrier(comm);

  if (my_rank == 0)
    std::cout << " done." << std::endl;
}

/* -------------------------------------------------------------------------- */
void Density::run() {

  // step 1: load current rank dataset in memory
  cacheData();

  // step 2: compute frequencies and histogram
  computeFrequencies();

  // step 3: bucket particles
  bucketParticles();

//  // inflate and deflate bucketed data
//  for (int step = 0; step < 6; ++step)
//    process(step);
//
//  // dump them
//  dump();
}

/* -------------------------------------------------------------------------- */