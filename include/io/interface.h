/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Authors: Pascal Grosset and Hoby Rakotoarivelo.
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

#pragma once
/* -------------------------------------------------------------------------- */
#include <string>
#include <mpi.h>
#include <sstream>
#include <unordered_map>
/* -------------------------------------------------------------------------- */
#include "data.h"
#include "utils/json.h"
/* -------------------------------------------------------------------------- */
class DataLoaderInterface {

public:
	virtual void init(std::string _filename, MPI_Comm _comm) = 0;
	virtual bool loadData(std::string paramName) = 0;
	virtual void saveCompData(std::string paramName, void* raw) = 0;
	virtual void writeData(std::string _filename) = 0;
	virtual bool saveInputFileParameters() = 0;
	virtual bool close() = 0;
	virtual void setParam(std::string paramName, std::string type, std::string value) = 0;
  virtual bool loadUncompressedFields(nlohmann::json const& jsonInput) = 0;

  void setSave(bool state) { saveData = state; }
	size_t getNumElements() { return numElements; }
	size_t * getSizePerDim() { return sizePerDim; }
	size_t getTypeSize() { return elemSize; }
	std::string getType() { return gio::to_string(dataType); }
	std::string getParam() { return param; }
	std::string getLog() { return log.str(); }

  std::string getDataInfo() {
    std::stringstream info;
    info << std::endl;
    info << "Loader type: " << loader << std::endl;
    info << "Filename: " << filename << std::endl;
    info << "Total number of elements: " << totalNumberOfElements << std::endl;
    info << "Param: " << param << std::endl;
    info << "dataType: " << gio::to_string(dataType) << std::endl;
    info << "numElements: " << numElements << std::endl;
    info << "sizePerDim: "
             << sizePerDim[0] << " " << sizePerDim[1] << " "
             << sizePerDim[2] << " " << sizePerDim[3] << " "
             << sizePerDim[4] << std::endl;

    return info.str();
  }

public:

  bool saveData = false;
  int origNumDims = 0;
  MPI_Comm comm = MPI_COMM_NULL;

  std::string loader = "";
  std::string filename = "";
  std::string param = "";
  std::stringstream log {};
  gio::TYPE dataType = gio::TYPE::INT32;

  size_t origDims[5] {0, 0, 0, 0, 0};
  size_t sizePerDim[5] {0, 0, 0, 0, 0};	  // For compression
  size_t rankOffset[3] {0, 0, 0};
  size_t elemSize = 0;				            // size in bytes of that parameter
  size_t totalNumberOfElements = 0;	      // total number of particles for input file
  size_t numElements = 0;				          // number of particles for that mpi rank

  void* data = nullptr;
  std::unordered_map<std::string, std::string> loaderParams;
  std::vector<gio::Data> inOutData;
};
/* -------------------------------------------------------------------------- */