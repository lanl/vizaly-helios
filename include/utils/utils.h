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

#include <stdio.h>
#include <stdbool.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

#ifdef _WIN32
  #include <direct.h>
  #define mkdir(a, b) _mkdir(a)
#endif

/* -------------------------------------------------------------------------- */
inline int createFolder(std::string folderName) {
#ifdef _WIN32
  folderName = ".\\" + folderName;
#endif

  struct stat finfo;
  int res = stat(folderName.c_str(), &finfo);

  if (finfo.st_mode & S_IFDIR)
    return 1; // Directory already exists
  else if (res != 0) {
    const int dir_err = mkdir(folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err) {
      std::cout << "Could not create directory " << folderName << std::endl;
      return 0;
    }
  } else {
    std::cout << folderName << " is not a directory!" << std::endl;
    return 0;
  }
  return 1;
}

/* -------------------------------------------------------------------------- */
inline bool fileExists(char *filename) {
  std::ifstream ifs(filename);
  return ifs.good();
}

/* -------------------------------------------------------------------------- */
inline bool isPowerOfTwo(int n) {
  if (n == 0)
    return 0;

  while (n != 1) {
    if (n % 2 != 0)
      return 0;
    n = n / 2;
  }
  return 1;
}

/* -------------------------------------------------------------------------- */
inline std::string extractFileName(std::string inputString) {
  std::size_t pos = inputString.find_last_of("/\\");
  return inputString.substr(pos + 1);
}

/* -------------------------------------------------------------------------- */
inline int allocateMem(std::string dataType, size_t numElements, int offset, void *&data) {
  if (dataType == "float")
    data = new float[numElements + offset];
  else if (dataType == "double")
    data = new double[numElements + offset];
  else if (dataType == "int8_t")
    data = new int8_t[numElements + offset];
  else if (dataType == "int16_t")
    data = new int16_t[numElements + offset];
  else if (dataType == "int32_t")
    data = new int32_t[numElements + offset];
  else if (dataType == "int64_t")
    data = new int64_t[numElements + offset];
  else if (dataType == "uint8_t")
    data = new uint8_t[numElements + offset];
  else if (dataType == "uint16_t")
    data = new uint16_t[numElements + offset];
  else if (dataType == "uint32_t")
    data = new uint32_t[numElements + offset];
  else if (dataType == "uint64_t")
    data = new uint64_t[numElements + offset];
  else
    return 0;

  return 1;
}

/* -------------------------------------------------------------------------- */
inline int deAllocateMem(std::string dataType, void *&data) {
  if (data == NULL) // already deallocated!
    return 1;

  if (dataType == "float")
    delete[](float *) data;
  else if (dataType == "double")
    delete[](double *) data;
  else if (dataType == "int8_t")
    delete[](int8_t *) data;
  else if (dataType == "int16_t")
    delete[](int16_t *) data;
  else if (dataType == "int32_t")
    delete[](int32_t *) data;
  else if (dataType == "int64_t")
    delete[](int64_t *) data;
  else if (dataType == "uint8_t")
    delete[](uint8_t *) data;
  else if (dataType == "uint16_t")
    delete[](uint16_t *) data;
  else if (dataType == "uint32_t")
    delete[](uint32_t *) data;
  else if (dataType == "uint64_t")
    delete[](uint64_t *) data;
  else
    return 0;

  data = NULL;

  return 1;
}
/* -------------------------------------------------------------------------- */