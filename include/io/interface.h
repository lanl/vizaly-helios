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

#pragma once
/* -------------------------------------------------------------------------- */
#include <string>
#include <sstream>
#include <unordered_map>
#include <mpi.h>
#include "data.h"
#include "utils/json.h"
/* -------------------------------------------------------------------------- */
class DataLoaderInterface {

public:
	virtual void init(std::string in_file, MPI_Comm _comm) = 0;
	virtual bool load(std::string in_param) = 0;
	virtual void save(std::string in_param, void* raw) = 0;
	virtual void dump(std::string in_file) = 0;
	virtual bool saveParams() = 0;
	virtual bool close() = 0;

  void setSave(bool state) { do_dump = state; }
	size_t getNumElements() const { return local_nb_elems; }
	size_t* getSizePerDim() { return size_per_dim; }
	size_t getTypeSize() const { return elem_size; }
	std::string getType() const { return gio::to_string(data_type); }
	std::string getParam() const { return param; }
	std::string getLog() const { return log.str(); }

  std::string getDataInfo() {
    std::stringstream info;
    info << std::endl;
    info << "Loader type: " << loader << std::endl;
    info << "Filename: " << filename << std::endl;
    info << "Total number of elements: " << total_nb_elems << std::endl;
    info << "Param: " << param << std::endl;
    info << "dataType: " << gio::to_string(data_type) << std::endl;
    info << "numElements: " << local_nb_elems << std::endl;
    info << "sizePerDim: "
         << size_per_dim[0] << " " << size_per_dim[1] << " "
         << size_per_dim[2] << " " << size_per_dim[3] << " "
         << size_per_dim[4] << std::endl;

    return info.str();
  }

public:

  bool do_dump = false;
  void* data = nullptr;
  MPI_Comm comm = MPI_COMM_NULL;

  std::string loader = "";
  std::string filename = "";
  std::string param = "";
  std::stringstream log {};
  gio::Type data_type = gio::Type::Int32;

  size_t size_per_dim[5] {0, 0, 0, 0, 0};
  size_t elem_size = 0;				                // size in bytes of that parameter
  size_t total_nb_elems = 0;	                // total number of particles
  size_t local_nb_elems = 0;			            // number of particles for that rank

  std::unordered_map<std::string, std::string> loader_params;
  std::vector<gio::Data> scalar_data;
};
/* -------------------------------------------------------------------------- */
