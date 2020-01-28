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

  assert(json.count("particles"));
  assert(json.count("density"));
  assert(json["density"].count("inputs"));
  assert(json["density"].count("extents"));
  assert(json["density"]["extents"].count("min"));
  assert(json["density"]["extents"].count("max"));
  assert(json["density"].count("nb_bins"));
  assert(json["density"].count("plots"));

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

  output_plot = json["density"]["plots"];
  nb_bins = json["density"]["bins"];
  assert(nb_bins > 0);

  histogram.resize(nb_bins);
  buckets.resize(nb_bins);

  // set the HACC IO manager
  ioMgr = std::make_unique<HACCDataLoader>();
  input_hacc = json["particles"];
}

/* -------------------------------------------------------------------------- */
bool Density::cacheData() {

  bool const master_rank = (my_rank == 0);

  // step 1: load particle file
  if (not input_hacc.empty()) {

    ioMgr->init(input_hacc, comm);
    ioMgr->saveParams();
    ioMgr->setSave(true);

    std::string const scalars[] = { "x", "y", "z" };

    if (master_rank)
      std::cout << "Caching particle data ... " << std::flush;

    for (int i = 0; i < 3; ++i) {
      if (ioMgr->load(scalars[i])) {
        // retrieve debug infos
        if (master_rank) {
          std::cout << ioMgr->getDataInfo();
          std::cout << ioMgr->getLog();
        }
        // store actual data
        auto const n = ioMgr->getNumElements();
        auto const data = static_cast<float*>(ioMgr->data);
        coords[i].resize(n);
        std::copy(data, data + n, coords[i].data());
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

    if (master_rank)
      std::cout << "done." << std::endl;
  }

  // step 2: load density file
  if (not inputs.empty()) {

    long offset = 0;
    long count = 0;
    std::string path;

    if (master_rank)
      std::cout << "Caching density data ... " << std::flush;

    for (auto&& current : inputs) {
      std::tie(path, count) = current;

      std::ifstream file(path, std::ios::binary);
      if (not file.good())
        return false;

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

  return true;
}


/* -------------------------------------------------------------------------- */
void Density::computeFrequencies() {

  if (my_rank == 0)
    std::cout << "Computing frequencies ... " << std::flush;

  assert(local_rho_count);
  assert(total_rho_count);

  // determine data values extents
  total_rho_max = 0;
  total_rho_min = 0;
  double local_rho_min = *std::min_element(density_field.data(), density_field.data() + local_rho_count);
  double local_rho_max = *std::max_element(density_field.data(), density_field.data() + local_rho_count);
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
    int index = static_cast<int>((range * relative_value) / capacity);

    if (index >= nb_bins)
      index--;

    local_histo[index]++;
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
  auto const coef = rho / (total_rho_max - total_rho_min);
  auto const index = static_cast<int>(std::floor(coef * float(nb_bins))) - 1;
  assert(index < nb_bins);
  return index;
}

/* -------------------------------------------------------------------------- */
void Density::bucketParticles() {

  for (int i = 0; i < local_particles; ++i) {
    float particle[] = { coords[0][i], coords[1][i], coords[2][i] };
    auto const density_index = deduceDensityIndex(particle);
    auto const bucket_index  = deduceBucketIndex(density_field[density_index]);
    // copy data in correct bucket
    buckets[bucket_index].push_back(i);
  }

  for (auto&& current : buckets)
    current.shrink_to_fit();

  // for debug purposes
  if (my_rank == 0) {
    auto const width = (total_rho_max - total_rho_min) / static_cast<double>(nb_bins);
    for (int i = 0; i < nb_bins; ++i) {
      auto const rho_max = total_rho_min + (i * width);
      std::printf("bucket[%d]: rho_max=%.f, nb_particles=%lu\n", i, rho_max, buckets[i].size());
    }
  }
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
void Density::inflate() {/* TODO */}

/* -------------------------------------------------------------------------- */
void Density::dump() {/* TODO */}

/* -------------------------------------------------------------------------- */
void Density::run() {

  // step 1: load current rank dataset in memory
  cacheData();

  // step 2: compute frequencies and histogram
  computeFrequencies();

  // step 3: bucket particles
  bucketParticles();
}

/* -------------------------------------------------------------------------- */