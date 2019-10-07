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
 * 3. Neither the name of mosquitto nor the names of its
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

#include <fstream>
#include "merge/merge.h"
/* -------------------------------------------------------------------------- */
Merger::Merger(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm)
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

  assert(json["input"].find("full") != json["input"].end());
  assert(json["input"].find("scalars") != json["input"].end());
  assert(json["merge"].find("halos") != json["merge"].end());
  assert(json["merge"].find("non-halos") != json["merge"].end());
  assert(json["merge"].find("output") != json["merge"].end());

  input_full = json["input"]["full"];
  halo_file = json["merge"]["halos"];
  non_halo_file = json["merge"]["non-halos"];
  output_combined = json["merge"]["output"];
  output_log = json["merge"]["logs"];

  for (auto&& name : json["input"]["scalars"])
    scalars.push_back(name);

  num_scalars = scalars.size();
  dataset.resize(num_scalars);

  // set the IO manager
  ioMgr = std::make_unique<HACCDataLoader>();
}

/* -------------------------------------------------------------------------- */
// generic method for caching a given dataset
template <bool save>
size_t Merger::cache(long offset) {

  debug_log << "Caching dataset ... ";

  if (save) {
    // set 'physOrigin' and 'physScale'
    // and update MPI cart partition while loading file.
    ioMgr->saveParams();
    ioMgr->setSave(true);
  }

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

  if (my_rank == 0 and save) {
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
void Merger::dump() {

  debug_log << "Dumping dataset ... ";
  if (my_rank == 0)
    std::cout << debug_log.str();

  assert(local_parts > 0);

  int periods[3] = {0,0,0};
  auto dim_size = ioMgr->mpi_partition;
  MPI_Cart_create(comm, 3, dim_size, periods, 0, &comm);

  // init writer and open file
  gio::GenericIO gioWriter(comm, output_combined);
  gioWriter.setNumElems(local_parts);

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
void Merger::run() {

  debug_log << "Reconstruct decompressed dataset:" << std::endl;

  ioMgr->init(halo_file, comm);
  local_halos = cache<false>(0);

  ioMgr->init(non_halo_file, comm);
  local_non_halos = cache<true>(local_halos);

  // get total number of particles
  total_halos = 0;
  total_non_halos = 0;
  MPI_Allreduce(&local_halos, &total_halos, 1, MPI_LONG, MPI_SUM, comm);
  MPI_Allreduce(&local_non_halos, &total_non_halos, 1, MPI_LONG, MPI_SUM, comm);

  local_parts = local_halos + local_non_halos;
  total_parts = total_halos + total_non_halos;

  int const format = static_cast<int>(std::ceil(std::log10(total_parts)));

  debug_log << "\tlocal parts: "<< local_parts << " particles"<< std::endl;
  debug_log << "\tlocal halos: "<< local_halos << " particles"<< std::endl;
  debug_log << "\tlocal non-halos: "<< local_non_halos << " particles"<< std::endl;

  debug_log << "\ttotal parts: "<< total_parts << " particles"<< std::endl;
  debug_log << "\ttotal halos: "<< total_halos << " particles"<< std::endl;
  debug_log << "\ttotal non-halos: "<< total_non_halos << " particles"<< std::endl;

  MPI_Barrier(comm);

  // now dump everything
  dump();

  if (my_rank == 0)
    dumpLogs();
}

/* -------------------------------------------------------------------------- */
void Merger::dumpLogs() {

  std::ofstream logfile(output_log, std::ios::out);
  logfile << debug_log.str();
  logfile.close();
  std::cout << "Logs generated in "<< output_log << std::endl;

  debug_log.clear();
  debug_log.str("");
  MPI_Barrier(comm);
}
/* -------------------------------------------------------------------------- */
