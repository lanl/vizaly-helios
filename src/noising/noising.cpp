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

#include "noising/noising.h"

/* -------------------------------------------------------------------------- */
Noising::Noising(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm)
  : json_path(in_path),
    my_rank(in_rank),
    nb_ranks(in_nb_ranks),
    comm(in_comm)
{
  assert(nb_ranks > 0);
  nlohmann::json json;
  std::string buffer;

  std::ifstream file(json_path);
  assert(file.is_open());
  assert(file.good());

  // parse params and do basic checks
  file >> json;

  assert(json["noising"].count("type"));
  assert(json["noising"].count("input"));
  assert(json["noising"].count("output"));
  assert(json["noising"].count("d_min"));
  assert(json["noising"].count("d_max"));
  assert(json["noising"].count("logs"));
  assert(json["noising"].count("plots"));

  std::string type = json["noising"]["type"];
  assert(type == "gaussian");

  input = json["noising"]["input"];
  output = json["noising"]["output"];
  output_log = json["noising"]["logs"];
  output_plot = json["noising"]["plots"];
  dist_min = json["noising"]["d_min"];
  dist_max = json["noising"]["d_max"];
  assert(dist_min < dist_max);

  for (auto&& name : json["input"]["scalars"])
    scalars.push_back(name);

  num_scalars = scalars.size();
  dataset.resize(num_scalars);
  histo.resize(num_scalars);

  // set the IO manager
  ioMgr = std::make_unique<HACCDataLoader>();
}

/* -------------------------------------------------------------------------- */
size_t Noising::cache(long offset) {

  debug_log << "Caching dataset ... ";

  // set 'physOrigin' and 'physScale'
  // and update MPI cart partition while loading file.
  ioMgr->saveParams();
  ioMgr->setSave(true);

  for (int i=0; i < num_scalars; ++i) {
    if (ioMgr->load(scalars[i])) {
      auto const n = ioMgr->getNumElements();
      auto const data = static_cast<float*>(ioMgr->data);
      dataset[i].resize(n + offset);
      std::copy(data, data + n, dataset[i].data() + offset);
    }
    MPI_Barrier(comm);
  }

  if (ioMgr->load("id")) {
    auto const n = ioMgr->getNumElements();
    auto const data = static_cast<long*>(ioMgr->data);
    particles_index.resize(n + offset);
    std::copy(data, data + n, particles_index.data() + offset);
  }

  if (my_rank == 0) {
    std::cout << "mpiCartPartitions: " << ioMgr->mpi_partition[0] << ", "
              << ioMgr->mpi_partition[1] << ", "
              << ioMgr->mpi_partition[2] << std::endl;
    std::cout << "physOrig: " << ioMgr->phys_orig[0] << ", "
              << ioMgr->phys_orig[1] << ", "
              << ioMgr->phys_orig[2] << std::endl;
    std::cout << "physScale: " << ioMgr->phys_scale[0] << ", "
              << ioMgr->phys_scale[1] << ", "
              << ioMgr->phys_scale[2] << std::endl;
  }

  debug_log << " done." << std::endl;
  MPI_Barrier(comm);

  size_t const copied_count = particles_index.size() - offset;
  return copied_count;
};


/* -------------------------------------------------------------------------- */
void Noising::dump() {

  debug_log << "Dumping dataset ... ";
  if (my_rank == 0)
    std::cout << debug_log.str();

  assert(local_count > 0);

  int periods[3] = {0,0,0};
  auto dim_size = ioMgr->mpi_partition;
  MPI_Cart_create(comm, 3, dim_size, periods, 0, &comm);

  // init writer and open file
  gio::GenericIO gioWriter(comm, output);
  gioWriter.setNumElems(local_count);

  // init physical params
  for (int d=0; d < 3; ++d) {
    gioWriter.setPhysOrigin(ioMgr->phys_orig[d], d);
    gioWriter.setPhysScale(ioMgr->phys_scale[d], d);
  }

  MPI_Barrier(comm);

  unsigned default_flag = gio::GenericIO::VarHasExtraSpace;

  // populate params now
  for (int i=0; i < num_scalars; ++i) {
    unsigned flag = default_flag;
    switch (i) {
      case 0: flag |= gio::GenericIO::VarIsPhysCoordX; break;
      case 1: flag |= gio::GenericIO::VarIsPhysCoordY; break;
      case 2: flag |= gio::GenericIO::VarIsPhysCoordZ; break;
      default: break;
    }
    gioWriter.addVariable(scalars[i].data(), dataset[i].data(), flag);
  }

  gioWriter.addVariable("id", particles_index.data(), default_flag);
  gioWriter.write();

  debug_log << " done." << std::endl;
  if (my_rank == 0)
    std::cout << debug_log.str();

  MPI_Barrier(comm);
}

/* -------------------------------------------------------------------------- */
std::vector<float> Noising::computeGaussianNoise(int field) {

  assert(field < num_scalars);

  // first gather the dataset size per rank.
  int nb_local = dataset[field].size();
  int nb_per_rank[nb_ranks];
  MPI_Gather(&nb_local, 1, MPI_INT, nb_per_rank, 1, MPI_INT, 0, comm);

  std::vector<float> noise(nb_local);

  // generate noise only on master rank
  if (my_rank == 0) {

    // generate a seed for random engine
    std::random_device device;

    // use Mersenne twister engine
    std::mt19937 engine { device() };

    // define a normal distribution generator
    auto const mean = static_cast<float>(0.5 * (dist_min + dist_max));
    auto const stddev = static_cast<float>((dist_max - dist_min) * dev_fact);

    std::normal_distribution<float> distrib(mean, stddev);

    for (auto& val : noise)
      val = distrib(engine);

    // generate number for current rank and send it
    for (int i = 1; i < nb_ranks; ++i) {
      // generate data
      std::vector<float> dist_noise(nb_per_rank[i]);
      for (int j = 0; j < nb_per_rank[i]; ++j) {
        dist_noise[j] = distrib(engine);
      }
      // send to correct rank
      MPI_Send(dist_noise.data(), nb_per_rank[i], MPI_FLOAT, i, 0, comm);
    }
  } else {
    MPI_Recv(noise.data(), nb_local, MPI_FLOAT, 0, 0, comm, MPI_STATUS_IGNORE);
  }

  return noise;
}

/* -------------------------------------------------------------------------- */
bool Noising::computeHistogram(int i, std::vector<float> const& noise) {

  int const nb_particles = noise.size();
  assert(nb_particles > 0);

  debug_log << "computing noise histogram for '"<< scalars[i] <<"'"<< std::endl;

  // step 1. determine lower and upper bounds on data
  float local_max = std::numeric_limits<float>::min();
  float local_min = std::numeric_limits<float>::max();
  float total_max = 0;
  float total_min = 0;

  for (auto const& value : noise) {
    if (value > local_max) { local_max = value; }
    if (value < local_min) { local_min = value; }
  }

  MPI_Allreduce(&local_max, &total_max, 1, MPI_FLOAT, MPI_MAX, comm);
  MPI_Allreduce(&local_min, &total_min, 1, MPI_FLOAT, MPI_MIN, comm);

  debug_log << "= local_extents: [" << local_min << ", " << local_max << "]"<< std::endl;
  debug_log << "= total_extents: [" << total_min << ", " << total_max << "]"<< std::endl;
  MPI_Barrier(comm);

  // Compute histogram of values
  if (total_max > 0) {

    debug_log << "num_bins: " << num_bins << std::endl;

    long local_histogram[num_bins];
    long total_histogram[num_bins];

    std::fill(local_histogram, local_histogram + num_bins, 0);
    std::fill(total_histogram, total_histogram + num_bins, 0);

    double range = total_max - total_min;
    double capacity = range / num_bins;

    for (auto k=0; k < nb_particles; ++k) {
      double relative_value = (noise[k] - total_min) / range;
      int index = static_cast<int>((range * relative_value) / capacity);

      if (index >= num_bins)
        index--;

      local_histogram[index]++;
    }

    MPI_Allreduce(local_histogram, total_histogram, num_bins, MPI_LONG, MPI_SUM, comm);

    // fill result array eventually
    histo[i].clear();
    histo[i].resize(num_bins);

    for (int j=0; j < num_bins; ++j)
      histo[i][j] = double(total_histogram[j]) / nb_particles;

    if (my_rank == 0)
      dumpHistogram(i);

    MPI_Barrier(comm);
    return true;
  }

  return false;
}

/* -------------------------------------------------------------------------- */
void Noising::computeSignalSpectrum(int field) {
  // TODO
}

/* -------------------------------------------------------------------------- */
void Noising::run() {

  ioMgr->init(input, comm);
  local_count = cache();
  total_count = 0;
  MPI_Allreduce(&local_count, &total_count, 1, MPI_LONG, MPI_SUM, comm);

  for (int i = 0; i < num_scalars; ++i) {

    debug_log << "Process field "<< scalars[i] << "."<< std::endl;

    // a) compute and apply noise on current dataset
    debug_log << "\t- apply gaussian noise ... ";
    int const nb_particles = dataset[i].size();
    auto const noise = computeGaussianNoise(i);
    for (int j = 0; j < nb_particles; ++j) {
      dataset[i][j] += noise[j];
    }

    debug_log << " done." << std::endl;
    MPI_Barrier(comm);

    // b) compute histogram
    debug_log << "\t- compute noise histogram ... ";
    computeHistogram(i, noise);
    debug_log << " done." << std::endl;
    MPI_Barrier(comm);

    // c) compute signal spectrum
    debug_log << "\t- compute signal spectrum ... ";
    computeSignalSpectrum(i);
    debug_log << " done." << std::endl;
    MPI_Barrier(comm);
  }

  debug_log << "\tdistribution: ["<< dist_min << ", "<< dist_max << "]."<< std::endl;
  debug_log << "\tdeviation: "<< (dist_max - dist_min) * dev_fact << "."<< std::endl;
  debug_log << "\ttotal: "<< total_count << " particles."<< std::endl;
  MPI_Barrier(comm);

  // now dump everything
  dump();

  if (my_rank == 0)
    dumpLogs();
}

/* -------------------------------------------------------------------------- */
void Noising::dumpHistogram(int field) {

  assert(field < num_scalars);

  auto const& scalar = scalars[field];
  std::string path = output_plot + "_" + scalar + ".dat";

  std::ofstream file(path, std::ios::out|std::ios::trunc);
  assert(file.is_open());
  assert(file.good());

  file << "# scalar: " << scalar << std::endl;
  file << "# num_bins: " << std::to_string(num_bins) << std::endl;
  for (auto const& value : histo[field]) {
    file << value << std::endl;
  }

  file.close();
}

/* -------------------------------------------------------------------------- */
void Noising::dumpLogs() {

  std::ofstream logfile(output_log, std::ios::out);
  logfile << debug_log.str();
  logfile.close();
  std::cout << "Logs generated in "<< output_log << std::endl;

  debug_log.clear();
  debug_log.str("");
}
/* -------------------------------------------------------------------------- */