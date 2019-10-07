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

#include "compressors/kernels/blosc.hpp"
#include "utils/timer.h"

/* -------------------------------------------------------------------------- */
int BLOSCCompressor::compress
  (void* input, void*& output, std::string type, size_t type_size, size_t* n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  // compress
  Timer timer;
  timer.start();

  size_t isize = type_size * numel;
  size_t osize = isize + BLOSC_MAX_OVERHEAD;

  output = std::malloc(isize); //byte array;
  // default: {clevel=9, shuffle=1, sizeof(data), isize, input, output, osize)
  osize = blosc_compress(9, 1, type_size, isize, input, output, osize);

  if (osize < 0) {
    std::cerr << "Compression failed: " << osize << std::endl;
    return EXIT_FAILURE;
  }
  else if (osize > 0) {
    output = std::realloc(output, osize);
  }

  timer.stop();
  bytes = osize;

  log << std::endl << name;
  log << " ~ InputBytes: " << isize;
  log << ", OutputBytes: " << osize;
  log << ", cRatio: " << isize / static_cast<float>(osize);
  log << ", #elements: " << numel << std::endl;
  log << name << " ~ CompressTime: " << timer.getDuration() << " s"<< std::endl;

  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
int BLOSCCompressor::decompress
  (void*& input, void *&output, std::string type, size_t type_size, size_t *n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();

  size_t osize = type_size * numel;
  output = std::malloc(osize);
  auto size = blosc_decompress(input, output, osize);
  if (size < 0) {
    std::cerr << "Decompression failed: " << size << std::endl;
    return EXIT_FAILURE;
  }

  std::free(input);
  input = nullptr;
  timer.stop();

  log << name << " ~ DecompressTime: " << timer.getDuration() << " s" << std::endl;

  return EXIT_SUCCESS;
}
/* -------------------------------------------------------------------------- */