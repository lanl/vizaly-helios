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

#include <algorithm>
#include <sstream>
#include <climits>
#include "io/hacc.h"
/* -------------------------------------------------------------------------- */
void HACCDataLoader::init(std::string _filename, MPI_Comm _comm) {
  filename = _filename;
  comm = _comm;
  saveData = false;

  MPI_Comm_size(comm, &numRanks);
  MPI_Comm_rank(comm, &myRank);
}

/* -------------------------------------------------------------------------- */
bool HACCDataLoader::saveInputFileParameters(){

  gio::GenericIO gioReader(comm, filename);
  std::vector<gio::GenericIO::VariableInfo> VI;

  // open file
  gioReader.openAndReadHeader(gio::GenericIO::MismatchRedistribute);
  int numDataRanks = gioReader.readNRanks();

  if (numRanks > numDataRanks) {
    std::cout << "Num data ranks: " << numDataRanks;
    std::cout << "Use <= MPI ranks than data ranks" << std::endl;
    return false;
  }

  // get dimensions of the input file
  // and read in the scalars information
  gioReader.readPhysOrigin(physOrigin);
  gioReader.readPhysScale(physScale);
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
    inOutData.push_back(readInData);
  }

  return true;
}

/* -------------------------------------------------------------------------- */
bool HACCDataLoader::loadData(std::string paramName) {

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

  if (numRanks > numDataRanks) {
    std::cout << "Num data ranks: " << numDataRanks;
    std::cout << "Use <= MPI ranks than data ranks" << std::endl;
    return false;
  }

  // Count number of elements
  totalNumberOfElements = 0;
  for (int i = 0; i < numDataRanks; ++i)
    totalNumberOfElements += gioReader.readNumElems(i);

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
      dataType = readInData.dataType;
      elemSize = readInData.size;
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
  int numDataRanksPerMPIRank = numDataRanks / numRanks;
  int loadRange[2];
  loadRange[0] = myRank * numDataRanksPerMPIRank;
  loadRange[1] = (myRank + 1) * numDataRanksPerMPIRank;
  if (myRank == numRanks - 1)
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
  numElements = 0;
  for (int i = loadRange[0]; i < loadRange[1]; i++) {
    numElements += gioReader.readNumElems(i);
    maxNumElementsPerRank = std::max(maxNumElementsPerRank,numElements);
  }

  allocateMem(gio::to_string(dataType), numElements, 0, data);

  readInData.setNumElements(maxNumElementsPerRank);
  readInData.allocateMem(1);
  sizePerDim[0] = numElements;	// For compression

  log << "totalNumberOfElements: " << totalNumberOfElements << std::endl;
  log << "numElements: " << numElements << std::endl;

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
      cur[j] = float(coords[j]) / splitDims[j] * physScale[j] + physOrigin[j];
      nxt[j] = float(coords[j] + 1) / splitDims[j] * physScale[j] + physOrigin[j];
    }

    log << "coordinates: (";
    log << cur[0] <<", "<< cur[1] <<", "<< cur[2] <<") -> ";
    log << nxt[0] <<", "<< nxt[1] <<", "<< nxt[2] <<")" << std::endl;

    if (saveData) {
      for (int j = 0; j < 3; ++j) {
        min[j] = std::min(static_cast<int>(cur[j]), min[j]);
        max[j] = std::max(static_cast<int>(nxt[j]), max[j]);
      }
    }

    auto name = readInData.name.c_str();
    void* raw = readInData.data;

    switch (readInData.dataType) {
      case gio::TYPE::FLOAT:  gioReader.addVariable(name, (float*)    raw, true); break;
      case gio::TYPE::DOUBLE: gioReader.addVariable(name, (double*)   raw, true); break;
      case gio::TYPE::INT08:  gioReader.addVariable(name, (int8_t*)   raw, true); break;
      case gio::TYPE::INT16:  gioReader.addVariable(name, (int16_t*)  raw, true); break;
      case gio::TYPE::INT32:  gioReader.addVariable(name, (int32_t*)  raw, true); break;
      case gio::TYPE::INT64:  gioReader.addVariable(name, (int64_t*)  raw, true); break;
      case gio::TYPE::UINT08: gioReader.addVariable(name, (uint8_t*)  raw, true); break;
      case gio::TYPE::UINT16: gioReader.addVariable(name, (uint16_t*) raw, true); break;
      case gio::TYPE::UINT32: gioReader.addVariable(name, (uint32_t*) raw, true); break;
      case gio::TYPE::UINT64: gioReader.addVariable(name, (uint64_t*) raw, true); break;
      default: break;
    }

    gioReader.readDataSection(0, Np, i, false); // reading the whole file

    auto const bytes = Np * readInData.size;

    switch (readInData.dataType) {
      case gio::TYPE::FLOAT:  std::memcpy(&((float*)   data)[offset], raw, bytes); break;
      case gio::TYPE::DOUBLE: std::memcpy(&((double*)  data)[offset], raw, bytes); break;
      case gio::TYPE::INT08:  std::memcpy(&((int8_t*)  data)[offset], raw, bytes); break;
      case gio::TYPE::INT16:  std::memcpy(&((int16_t*) data)[offset], raw, bytes); break;
      case gio::TYPE::INT32:  std::memcpy(&((int32_t*) data)[offset], raw, bytes); break;
      case gio::TYPE::INT64:  std::memcpy(&((int64_t*) data)[offset], raw, bytes); break;
      case gio::TYPE::UINT08: std::memcpy(&((uint8_t*) data)[offset], raw, bytes); break;
      case gio::TYPE::UINT16: std::memcpy(&((uint16_t*)data)[offset], raw, bytes); break;
      case gio::TYPE::UINT32: std::memcpy(&((uint32_t*)data)[offset], raw, bytes); break;
      case gio::TYPE::UINT64: std::memcpy(&((uint64_t*)data)[offset], raw, bytes); break;
      default: break;
    }

    offset = offset + Np;
  }

  clock.stop();

  if (saveData) {
    int x_range = max[0] - min[0];
    int y_range = max[1] - min[1];
    int z_range = max[2] - min[2];

    mpiCartPartitions[0] = static_cast<int>(physScale[0] / x_range);
    mpiCartPartitions[1] = static_cast<int>(physScale[1] / y_range);
    mpiCartPartitions[2] = static_cast<int>(physScale[2] / z_range);

    log << "mpiCartPartitions: ";
    log << mpiCartPartitions[0] << ", "
        << mpiCartPartitions[1] << ", "
        << mpiCartPartitions[2] << std::endl;
  }

  readInData.deAllocateMem();
  return true;
}

/* -------------------------------------------------------------------------- */
void HACCDataLoader::saveCompData(std::string paramName, void* raw) {

  for (auto&& scalar : inOutData) {
    if (scalar.name == paramName) {
      scalar.doWrite = true;
      scalar.setNumElements(numElements);
      scalar.allocateMem();
      std::memcpy(scalar.data, raw, scalar.size * numElements);

      log.str("");
      log << "\nHACCDataLoader::saveCompData" << std::endl;
      log << paramName << " found. It has " << scalar.numElements << " elements";
      log << " of size " << scalar.size << std::endl;
    }
  }
}

/* -------------------------------------------------------------------------- */
void HACCDataLoader::writeData(std::string _filename) {

  Timer clock;
  log.str("");

  // Create setup
  int periods[3] = { 0, 0, 0 };
  MPI_Cart_create(comm, 3, mpiCartPartitions, periods, 0, &comm);

  // Init GenericIO writer + open file
  gio::GenericIO gioWriter(comm, _filename);
  gioWriter.setNumElems(numElements);

  // Init physical parameters
  for (int d = 0; d < 3; ++d) {
    gioWriter.setPhysOrigin(physOrigin[d], d);
    gioWriter.setPhysScale(physScale[d], d);
  }

  MPI_Barrier(comm);

  // Populate parameters
  for (auto&& scalar : inOutData) {
    unsigned flag = gio::GenericIO::VarHasExtraSpace;

    if (scalar.xVar) flag |= gio::GenericIO::VarIsPhysCoordX;
    else if (scalar.yVar) flag |= gio::GenericIO::VarIsPhysCoordY;
    else if (scalar.zVar) flag |= gio::GenericIO::VarIsPhysCoordZ;

    auto name = scalar.name.c_str();
    void* raw = scalar.data;

    switch (scalar.dataType) {
      case gio::TYPE::FLOAT:  gioWriter.addVariable(name, (float*)    raw, flag); break;
      case gio::TYPE::DOUBLE: gioWriter.addVariable(name, (double*)   raw, flag); break;
      case gio::TYPE::INT08:  gioWriter.addVariable(name, (int8_t*)   raw, flag); break;
      case gio::TYPE::INT16:  gioWriter.addVariable(name, (int16_t*)  raw, flag); break;
      case gio::TYPE::INT32:  gioWriter.addVariable(name, (int32_t*)  raw, flag); break;
      case gio::TYPE::INT64:  gioWriter.addVariable(name, (int64_t*)  raw, flag); break;
      case gio::TYPE::UINT08: gioWriter.addVariable(name, (uint8_t*)  raw, flag); break;
      case gio::TYPE::UINT16: gioWriter.addVariable(name, (uint16_t*) raw, flag); break;
      case gio::TYPE::UINT32: gioWriter.addVariable(name, (uint32_t*) raw, flag); break;
      case gio::TYPE::UINT64: gioWriter.addVariable(name, (uint64_t*) raw, flag); break;
      default: std::cout << " = undefined data type!" << std::endl; break;
    }
  }

  gioWriter.write();

  log << "HACCDataLoader::writeData " << _filename << std::endl;
  MPI_Barrier(comm);
  clock.stop();

  log << "Writing data took " << clock.getDuration() << " s" << std::endl;
}