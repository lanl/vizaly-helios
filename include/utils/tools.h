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

/* -------------------------------------------------------------------------- */
#pragma once
/* -------------------------------------------------------------------------- */
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <mpi.h>

#include "utils/json.h"
#include "utils/timer.h"
#include "utils/log.h"
#include "utils/utils.h"
/* -------------------------------------------------------------------------- */
namespace tools {
/* -------------------------------------------------------------------------- */
inline bool valid(int argc, char* argv[], int my_rank, int nb_ranks) {

  if (argc < 2) {
    if (my_rank == 0)
      std::cerr << "Usage: mpirun -n <int> ./analyzer [input-json]" << std::endl;
    return false;
  }

  // check input json file
  std::string path(argv[1]);
  std::ifstream file(path);
  nlohmann::json json;

  if (not file.good()) {
    if (my_rank == 0)
      std::cerr << "Error while opening parameter file: "<< argv[1] << std::endl;
    file.close();
    return false;
  }

  try {
    // pass file to json parser
    file >> json;

    // retrieve input file
    if (json["input"]["filetype"] != "HACC") {
      if (my_rank == 0)
        std::cerr << "Only HACC data is supported" << std::endl;
      file.close();
      return false;
    }

    file.close();
    return true;
  } catch(nlohmann::json::parse_error& e) {
    if (my_rank == 0)
      std::cerr << "Invalid JSON file " << path << "\n" << e.what() << std::endl;
    file.close();
    return false;
  }
}
/* -------------------------------------------------------------------------- */
inline std::string base(std::string path) {
  char sep = '/';
#ifdef _WIN32
  sep = '\\';
#endif

  size_t i = path.rfind(sep, path.length());
  if (i != std::string::npos) {
    return std::string(path.substr(i+1, path.length() - i));
  }
  return std::string("");
}
/* -------------------------------------------------------------------------- */
} // namespace tools
