/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Author: Pascal Grosset and Hoby Rakotoarivelo
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

#include <cstdio>
#include <cstdbool>
#include <string>
#include <fstream>
#include <iostream>

#include "utils/tools.h"
#include "utils/json.h"
/* -------------------------------------------------------------------------- */
namespace tools {
/* -------------------------------------------------------------------------- */
std::string base(std::string const &path) {
#ifdef _WIN32
  char sep = '\\';
#else
  char sep = '/';
#endif

  size_t i = path.rfind(sep, path.length());
  if (i != std::string::npos) {
    return std::string(path.substr(i + 1, path.length() - i));
  }
  return std::string("");
}

/* -------------------------------------------------------------------------- */
bool createFolder(std::string const &folder) {
#ifdef _WIN32
  folder = ".\\" + folderName;
#endif

  int const res = stat(folder.c_str(), &file_info);

  // Directory already exists
  if (file_info.st_mode & S_IFDIR) {
    return true;
  } else if (res != 0) {
    const int dir_err = mkdir(folder.c_str(), S_IRWXU|S_IRWXG|S_IROTH|S_IXOTH);
    if (-1 == dir_err) {
      std::cerr << "Error on creating directory " << folder << std::endl;
      return false;
    }
  } else {
    std::cerr << folder << " is not a directory!" << std::endl;
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
bool isPowerOfTwo(int n) {

  if (n != 0) {
    while (n != 1) {
      if (n % 2 != 0)
        return false;
      n = n / 2;
    }
    return true;
  }
  return false;
}

/* -------------------------------------------------------------------------- */
std::string extractFileName(std::string const &input) {
  std::size_t pos = input.find_last_of("/\\");
  return input.substr(pos + 1);
}

/* -------------------------------------------------------------------------- */
bool valid(int argc, char **argv, int rank, int nb_ranks) {

  if (argc < 2) {
    if (rank == 0) {
      std::cerr << "Usage: mpirun -n <int> ./program params.json";
      std::cerr << std::endl;
    }
    return false;
  }

  // check input JSON file
  std::string path(argv[1]);
  nlohmann::json json;

  try {
    std::ifstream file(path);
    if (file.good()) {
      file >> json;
      file.close();
    } else {
      if (rank == 0)
        std::cerr << "Error while opening parameter file: " << path << std::endl;
      file.close();
      return false;
    }
  }
  catch (nlohmann::json::parse_error &e) {
    if (rank == 0) {
      std::cerr << "Error: invalid input file " << path << std::endl;
      std::cerr << e.what() << std::endl;
      std::cerr << "Please verify your JSON file using e.g. ";
      std::cerr << "https://jsonformatter.curiousconcept.com" << std::endl;
    }
    return false;
  }

  /*
  // check if powers of 2 number of ranks
  if (json["compress"]["output"].count("dump")) {
    if (not isPowerOfTwo(nb_ranks)) {
      if (rank == 0) {
        std::cerr << "Please run with powers of two ranks when dumping HACC files.";
        std::cerr << std::endl;
      }
      return false;
    }
  }*/

  return true;
}

/* -------------------------------------------------------------------------- */
void dump(std::string const& path, std::string const& content, std::string const& ext) {
  std::ofstream file(path + ext, std::ios::out);
  file << content;
  file.close();
}

/* -------------------------------------------------------------------------- */
void append(std::string const& path, std::string const& content, std::string const& ext) {
#ifndef NDEBUG
  std::ofstream file(path + ext, std::ios::out|std::ios::app);
  file << content;
  file.close();
#endif
}

/* -------------------------------------------------------------------------- */
void append(std::string const& path, std::stringstream& content, std::string const& ext) {
#ifndef NDEBUG
  std::ofstream file(path + ext, std::ios::out|std::ios::app);
  file << content.str();
  file.close();
  content.str("");  // clears the log
#endif
}

/* -------------------------------------------------------------------------- */
}
