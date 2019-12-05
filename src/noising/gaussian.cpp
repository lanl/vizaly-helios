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

#include "noising/gaussian.h"

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

  std::string type = json["noising"]["type"];
  assert(type == "gaussian");

  input = json["noising"]["input"];
  output = json["noising"]["output"];
  output_log = json["noising"]["logs"];
  dist_min = json["noising"]["d_min"];
  dist_max = json["noising"]["d_max"];
  assert(dist_min < dist_max);

  for (auto&& name : json["input"]["scalars"])
    scalars.push_back(name);

  num_scalars = scalars.size();
  dataset.resize(num_scalars);

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
    index.resize(n + offset);
    std::copy(data, data + n, index.data() + offset);
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

  size_t const copied_count = index.size() - offset;
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

  gioWriter.addVariable("id", index.data(), default_flag);
  gioWriter.write();

  debug_log << " done." << std::endl;
  if (my_rank == 0)
    std::cout << debug_log.str();

  MPI_Barrier(comm);
}

/* -------------------------------------------------------------------------- */
void Noising::processField(int i) {

  assert(i < num_scalars);

  // generate a seed for random engine
  // we have to re-generate it for each scalar
  std::random_device device;

  // use Mersenne twister engine to generate pseudo-random numbers.
  std::mt19937 engine { device() };

  // define a normal distribution generator
  double const mean = 0.5 * (dist_min + dist_max);
  double const stddev = (dist_max - dist_min) * dev_fact;

  std::normal_distribution<double> distrib(mean, stddev);

  for (float& val : dataset[i]) {
    val += distrib(engine);
  }
}

/* -------------------------------------------------------------------------- */
void Noising::run() {

  debug_log << "Adding gaussian noise to dataset:" << std::endl;

  ioMgr->init(input, comm);
  local_count = cache();
  total_count = 0;
  MPI_Allreduce(&local_count, &total_count, 1, MPI_LONG, MPI_SUM, comm);

  for (int i = 0; i < num_scalars; ++i) { processField(i); }

  debug_log << "\tdist_range: ["<< dist_min << ", "<< dist_max << "]."<< std::endl;
  debug_log << "\tdeviation: "<< (dist_max - dist_min) * dev_fact << "."<< std::endl;
  debug_log << "\tlocal updated: "<< local_count << " particles,"<< std::endl;
  debug_log << "\ttotal updated: "<< total_count << " particles."<< std::endl;
  MPI_Barrier(comm);

  // now dump everything
  dump();

  if (my_rank == 0)
    dumpLogs();
}

/* -------------------------------------------------------------------------- */
void Noising::dumpLogs() {

  std::ofstream logfile(output_log, std::ios::out);
  logfile << debug_log.str();
  logfile.close();
  std::cout << "Logs generated in "<< output_log << std::endl;

  debug_log.clear();
  debug_log.str("");
  MPI_Barrier(comm);
}
/* -------------------------------------------------------------------------- */