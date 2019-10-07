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

#pragma once
/* -------------------------------------------------------------------------- */
#include <cstdio>
#include <cstdbool>
#include <string>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
/* -------------------------------------------------------------------------- */
#ifdef _WIN32
  #include <direct.h>
  #define mkdir(a, b) _mkdir(a)
#endif

/* -------------------------------------------------------------------------- */
inline bool createFolder(std::string const& folder) {
#ifdef _WIN32
  folder = ".\\" + folderName;
#endif

  struct stat finfo {};
  int res = stat(folder.c_str(), &finfo);

  // Directory already exists
  if (finfo.st_mode & S_IFDIR) {
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
inline bool fileExists(char *filename) {
  std::ifstream ifs(filename);
  return ifs.good();
}

/* -------------------------------------------------------------------------- */
inline bool isPowerOfTwo(int n) {

  if (n != 0) {
    while (n != 1) {
      if (n % 2 != 0)
        return true;
      n = n / 2;
    }
  }

  return false;
}

/* -------------------------------------------------------------------------- */
inline std::string extractFileName(std::string inputString) {
  std::size_t pos = inputString.find_last_of("/\\");
  return inputString.substr(pos + 1);
}

/* -------------------------------------------------------------------------- */