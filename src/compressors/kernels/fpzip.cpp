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

#if ENABLE_FPZIP
/* -------------------------------------------------------------------------- */
#include <sstream>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include "compressors/kernels/fpzip.hpp"
#include "utils/timer.h"
/* -------------------------------------------------------------------------- */
int FPZIPCompressor::compress
  (void *input, void *&output, std::string type, size_t type_size, size_t *n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();

  output = malloc(1024 + type_size * numel);

  FPZ *fpz = fpzip_write_to_buffer(output, 1024 + type_size * numel);

  if (type == "float")
    fpz->type = FPZIP_TYPE_FLOAT;
  else
    fpz->type = FPZIP_TYPE_DOUBLE;

  fpz->prec = 27; // Number of bits of precision (input param) (of 32 for float)

  if (parameters.count("bits")) {
    std::string value = parameters["bits"];
    if (not value.empty())
      fpz->prec = std::stoi(value);
  }

  fpz->nx = n[0];
  fpz->ny = (n[1] != 0 ? n[1] : 1);
  fpz->nz = (n[2] != 0 ? n[2] : 1);
  fpz->nf = (n[3] != 0 ? n[3] : 1);


  // perform actual compression
  bytes = fpzip_write(fpz, input);
  if (not bytes) {
    std::cerr << "Compression failed: "<<  fpzip_errstr[fpzip_errno] << std::endl;
    return EXIT_FAILURE;
  }

  fpzip_write_close(fpz);
  timer.stop();

  log << std::endl << name;
  log << " ~ InputBytes: " << type_size * numel;
  log << ", OutputBytes: " << bytes;
  log << ", cRatio: " << (type_size * numel / (float) bytes);
  log << ", #elements: " << numel << std::endl;
  log << name << " ~ CompressTime: " << timer.getDuration() << " s" << std::endl;

  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
int FPZIPCompressor::decompress
  (void*& input, void*& output, std::string type, size_t type_size, size_t *n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();

  output = malloc(type_size * numel);

  FPZ *fpz = fpzip_read_from_buffer(input);
  if (type == "float")
    fpz->type = FPZIP_TYPE_FLOAT;
  else
    fpz->type = FPZIP_TYPE_DOUBLE;

  fpz->prec = 27;

  if (parameters.count("bits")) {
    std::string value = parameters["bits"];
    if (not value.empty())
      fpz->prec = std::stoi(value);
  }

  fpz->nx = static_cast<int>(n[0]);
  fpz->ny = static_cast<int>(n[1] != 0 ? n[1] : 1);
  fpz->nz = static_cast<int>(n[2] != 0 ? n[2] : 1);
  fpz->nf = static_cast<int>(n[3] != 0 ? n[3] : 1);

  if (not fpzip_read(fpz, output)) {
    std::cerr << "Decompression failed: "<< fpzip_errstr[fpzip_errno] << std::endl;
    return EXIT_FAILURE;
  }

  fpzip_read_close(fpz);
  timer.stop();

  std::free(input);
  input = nullptr;

  log << name << " ~ DecompressTime: " << timer.getDuration() << " s"<< std::endl;

  return EXIT_SUCCESS;
}
/* -------------------------------------------------------------------------- */
#endif