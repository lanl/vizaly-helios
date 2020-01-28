/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Authors: Pascal Grosset, Jesus Pulido and Hoby Rakotoarivelo.
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

#include <algorithm>
#include <sstream>
#include <climits>
#include "io/hacc.h"
#include "utils/memory.h"
/* -------------------------------------------------------------------------- */
void HACCDataLoader::init(std::string in_file, MPI_Comm _comm) {
  filename = in_file;
  comm = _comm;
  do_dump = false;

  MPI_Comm_size(comm, &nb_ranks);
  MPI_Comm_rank(comm, &rank);
}

/* -------------------------------------------------------------------------- */
bool HACCDataLoader::close() {
  return Memory::release(data, data_type);
}

/* -------------------------------------------------------------------------- */
bool HACCDataLoader::saveParams(){

  gio::GenericIO gioReader(comm, filename);
  std::vector<gio::GenericIO::VariableInfo> VI;

  // open file
  gioReader.openAndReadHeader(gio::GenericIO::MismatchRedistribute);
  int numDataRanks = gioReader.readNRanks();

  if (nb_ranks > numDataRanks) {
    std::cout << "Num data ranks: " << numDataRanks;
    std::cout << "Use <= MPI ranks than data ranks" << std::endl;
    return false;
  }

  // get dimensions of the input file
  // and read in the scalars information
  gioReader.readPhysOrigin(phys_orig);
  gioReader.readPhysScale(phys_scale);
  gioReader.getVariableInfo(VI);

  int numVars = static_cast<int>(VI.size());

  for (int i = 0; i < numVars; i++) {
    gio::Data readInData;
    readInData.init(
      i, VI[i].Name, static_cast<int>(VI[i].Size),
      VI[i].IsFloat, VI[i].IsSigned,
      VI[i].IsPhysCoordX, VI[i].IsPhysCoordY, VI[i].IsPhysCoordZ
    );

    readInData.determineDataType();
    scalar_data.push_back(readInData);
  }

  return true;
}

/* -------------------------------------------------------------------------- */
bool HACCDataLoader::load(std::string paramName) {

  Timer clock;
  clock.start();

  log.str("");
  param = paramName;

  // Init GenericIO reader + open file
  gio::GenericIO gioReader(comm, filename);
  std::vector<gio::GenericIO::VariableInfo> VI;

  // Open file
  gioReader.openAndReadHeader(gio::GenericIO::MismatchRedistribute);
  int numDataRanks = gioReader.readNRanks();

  if (nb_ranks > numDataRanks) {
    std::cout << "Num data ranks: " << numDataRanks;
    std::cout << "Use <= MPI ranks than data ranks" << std::endl;
    return false;
  }

  // Count number of elements
  total_nb_elems = 0;
  for (int i = 0; i < numDataRanks; ++i)
    total_nb_elems += gioReader.readNumElems(i);

  // Read in the scalars information
  gioReader.getVariableInfo(VI);
  int numVars = static_cast<int>(VI.size());
  bool paramToLoad = false;

  gio::Data readInData;
  for (int i = 0; i < numVars; i++) {
    if (VI[i].Name == paramName) {
      readInData.init(
        i, VI[i].Name, static_cast<int>(VI[i].Size),
        VI[i].IsFloat, VI[i].IsSigned,
        VI[i].IsPhysCoordX, VI[i].IsPhysCoordY, VI[i].IsPhysCoordZ
      );

      readInData.determineDataType();
      data_type = readInData.data_type;
      elem_size = readInData.size;
      paramToLoad = true;
      break;
    }
  }

  if (not paramToLoad) {
    std::cout << "Cannot find that parameter, exiting now!";
    return false;
  }

  //
  // Split ranks among data
  int numDataRanksPerMPIRank = numDataRanks / nb_ranks;
  int loadRange[2];
  loadRange[0] = rank * numDataRanksPerMPIRank;
  loadRange[1] = (rank + 1) * numDataRanksPerMPIRank;
  if (rank == nb_ranks - 1)
    loadRange[1] = numDataRanks;

  int splitDims[3];
  gioReader.readDims(splitDims);
  log << "splitDims: "
      << splitDims[0] << ","
      << splitDims[1] << ","
      << splitDims[2] << std::endl;

  //
  // Determine memory size and allocate memory
  size_t maxNumElementsPerRank = 0;
  local_nb_elems = 0;
  for (int i = loadRange[0]; i < loadRange[1]; i++) {
    local_nb_elems += gioReader.readNumElems(i);
    maxNumElementsPerRank = std::max(maxNumElementsPerRank, local_nb_elems);
  }

  Memory::allocate(data, data_type, local_nb_elems, 0);

  readInData.setNumElements(maxNumElementsPerRank);
  readInData.allocate(1);
  size_per_dim[0] = local_nb_elems;	// For compression

  log << "totalNumberOfElements: " << total_nb_elems << std::endl;
  log << "numElements: " << local_nb_elems << std::endl;

  //
  // Actually load the data
  int min[] = {INT_MAX, INT_MAX, INT_MAX};
  int max[] = {INT_MIN, INT_MIN, INT_MIN};

  size_t offset = 0;
  // for each rank
  for (int i = loadRange[0]; i < loadRange[1]; i++) {

    size_t Np = gioReader.readNumElems(i);

    int coords[3];
    gioReader.readCoords(coords, i);
    log << "Coord indices: " << coords[0] << ", " << coords[1] << ", " << coords[2] << " | ";

    double cur[3], nxt[3];
    for (int j = 0; j < 3; ++j) {
      cur[j] = float(coords[j]) / splitDims[j] * phys_scale[j] + phys_orig[j];
      nxt[j] = float(coords[j] + 1) / splitDims[j] * phys_scale[j] + phys_orig[j];
    }

    log << "coordinates: (";
    log << cur[0] <<", "<< cur[1] <<", "<< cur[2] <<") -> ";
    log << nxt[0] <<", "<< nxt[1] <<", "<< nxt[2] <<")" << std::endl;

    if (do_dump) {
      for (int j = 0; j < 3; ++j) {
        min[j] = std::min(static_cast<int>(cur[j]), min[j]);
        max[j] = std::max(static_cast<int>(nxt[j]), max[j]);
      }
    }

    auto name = readInData.name.c_str();
    void* raw = readInData.data;

    switch (readInData.data_type) {
      case gio::Type::Float:  gioReader.addVariable(name, (float*)    raw, true); break;
      case gio::Type::Double: gioReader.addVariable(name, (double*)   raw, true); break;
      case gio::Type::Int:    gioReader.addVariable(name, (int*)      raw, true); break;
      case gio::Type::Int8:   gioReader.addVariable(name, (int8_t*)   raw, true); break;
      case gio::Type::Int16:  gioReader.addVariable(name, (int16_t*)  raw, true); break;
      case gio::Type::Int32:  gioReader.addVariable(name, (int32_t*)  raw, true); break;
      case gio::Type::Int64:  gioReader.addVariable(name, (int64_t*)  raw, true); break;
      case gio::Type::Uint8:  gioReader.addVariable(name, (uint8_t*)  raw, true); break;
      case gio::Type::Uint16: gioReader.addVariable(name, (uint16_t*) raw, true); break;
      case gio::Type::Uint32: gioReader.addVariable(name, (uint32_t*) raw, true); break;
      case gio::Type::Uint64: gioReader.addVariable(name, (uint64_t*) raw, true); break;
      default: break;
    }

    gioReader.readDataSection(0, Np, i, false); // reading the whole file

    auto const bytes = Np * readInData.size;

    switch (readInData.data_type) {
      case gio::Type::Float:  std::memcpy(&((float*)   data)[offset], raw, bytes); break;
      case gio::Type::Double: std::memcpy(&((double*)  data)[offset], raw, bytes); break;
      case gio::Type::Int:    std::memcpy(&((int*)     data)[offset], raw, bytes); break;
      case gio::Type::Int8:   std::memcpy(&((int8_t*)  data)[offset], raw, bytes); break;
      case gio::Type::Int16:  std::memcpy(&((int16_t*) data)[offset], raw, bytes); break;
      case gio::Type::Int32:  std::memcpy(&((int32_t*) data)[offset], raw, bytes); break;
      case gio::Type::Int64:  std::memcpy(&((int64_t*) data)[offset], raw, bytes); break;
      case gio::Type::Uint8:  std::memcpy(&((uint8_t*) data)[offset], raw, bytes); break;
      case gio::Type::Uint16: std::memcpy(&((uint16_t*)data)[offset], raw, bytes); break;
      case gio::Type::Uint32: std::memcpy(&((uint32_t*)data)[offset], raw, bytes); break;
      case gio::Type::Uint64: std::memcpy(&((uint64_t*)data)[offset], raw, bytes); break;
      default: break;
    }

    offset = offset + Np;
  }

  clock.stop();

  if (do_dump) {
    int x_range = max[0] - min[0];
    int y_range = max[1] - min[1];
    int z_range = max[2] - min[2];

    data_extents[0] = std::make_pair(min[0], max[0]);
    data_extents[1] = std::make_pair(min[1], max[1]);
    data_extents[2] = std::make_pair(min[2], max[2]);

    mpi_partition[0] = static_cast<int>(phys_scale[0] / x_range);
    mpi_partition[1] = static_cast<int>(phys_scale[1] / y_range);
    mpi_partition[2] = static_cast<int>(phys_scale[2] / z_range);

    log << "\t[x_min, x_max]: [" << min[0] << ", " << max[0] << "]" << std::endl;
    log << "\t[y_min, y_max]: [" << min[1] << ", " << max[1] << "]" << std::endl;
    log << "\t[z_min, z_max]: [" << min[2] << ", " << max[2] << "]" << std::endl;

    log << "mpiCartPartitions: ";
    log << mpi_partition[0] << ", "
        << mpi_partition[1] << ", "
        << mpi_partition[2] << std::endl;
  }

  readInData.release();
  return true;
}

/* -------------------------------------------------------------------------- */
void HACCDataLoader::save(std::string in_param, void* raw) {

  for (auto&& scalar : scalar_data) {
    if (scalar.name == in_param) {
      scalar.do_write = true;
      scalar.setNumElements(local_nb_elems);
      scalar.allocate();
      std::memcpy(scalar.data, raw, scalar.size * local_nb_elems);

      log.str("");
      log << std::endl;
      log << "HACCDataLoader::save" << std::endl;
      log << in_param << " found. It has " << scalar.nb_elems << " elements";
      log << " of size " << scalar.size << std::endl;
    }
  }
}

/* -------------------------------------------------------------------------- */
void HACCDataLoader::dump(std::string in_file) {

  Timer clock;
  log.str("");

  // Create setup
  int periods[3] = { 0, 0, 0 };
  MPI_Cart_create(comm, 3, mpi_partition, periods, 0, &comm);

  // Init GenericIO writer + open file
  gio::GenericIO gioWriter(comm, in_file);
  gioWriter.setNumElems(local_nb_elems);

  // Init physical parameters
  for (int d = 0; d < 3; ++d) {
    gioWriter.setPhysOrigin(phys_orig[d], d);
    gioWriter.setPhysScale(phys_scale[d], d);
  }

  MPI_Barrier(comm);

  // Populate parameters
  for (auto&& scalar : scalar_data) {
    unsigned flag = gio::GenericIO::VarHasExtraSpace;

    if (scalar.x_var) flag |= gio::GenericIO::VarIsPhysCoordX;
    else if (scalar.y_var) flag |= gio::GenericIO::VarIsPhysCoordY;
    else if (scalar.z_var) flag |= gio::GenericIO::VarIsPhysCoordZ;

    auto name = scalar.name.c_str();
    void* raw = scalar.data;

    switch (scalar.data_type) {
      case gio::Type::Float:  gioWriter.addVariable(name, (float*)    raw, flag); break;
      case gio::Type::Double: gioWriter.addVariable(name, (double*)   raw, flag); break;
      case gio::Type::Int:    gioWriter.addVariable(name, (int*)      raw, flag); break;
      case gio::Type::Int8:   gioWriter.addVariable(name, (int8_t*)   raw, flag); break;
      case gio::Type::Int16:  gioWriter.addVariable(name, (int16_t*)  raw, flag); break;
      case gio::Type::Int32:  gioWriter.addVariable(name, (int32_t*)  raw, flag); break;
      case gio::Type::Int64:  gioWriter.addVariable(name, (int64_t*)  raw, flag); break;
      case gio::Type::Uint8:  gioWriter.addVariable(name, (uint8_t*)  raw, flag); break;
      case gio::Type::Uint16: gioWriter.addVariable(name, (uint16_t*) raw, flag); break;
      case gio::Type::Uint32: gioWriter.addVariable(name, (uint32_t*) raw, flag); break;
      case gio::Type::Uint64: gioWriter.addVariable(name, (uint64_t*) raw, flag); break;
      default: std::cout << " = undefined data type!" << std::endl; break;
    }
  }

  gioWriter.write();

  log << "HACCDataLoader::dump " << in_file << std::endl;
  MPI_Barrier(comm);
  clock.stop();

  log << "Writing data took " << clock.getDuration() << " s" << std::endl;
}
/* -------------------------------------------------------------------------- */
