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

#if ENABLE_ISABELA
/* -------------------------------------------------------------------------- */
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <sstream>
#include "compressors/kernels/isabela.hpp"
#include "utils/timer.h"
/* -------------------------------------------------------------------------- */
int IsabelaCompressor::compress
  (void* input, void*& output, std::string type, size_t type_size, size_t * n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();

  output = malloc(type_size * numel);

  enum ISABELA_status status;
  struct isabela_stream i_strm {};
  struct isabela_config config {};

  double tol = 1E-3;
  if (parameters.count("tolerance")) {
    std::string value = parameters["tolerance"];
    if (not value.empty()) {
      tol = std::stod(value);
    }
  }

  config.ncoefficients = 30;
  if (parameters.count("nCoeff")) {
    std::string value = parameters["nCoeff"];
    if (not value.empty()) {
      config.ncoefficients = std::stoul(value);
    }
  }

  // Compress window_size numebr of elements at a time
  config.window_size = 1024;
  if (parameters.count("window-size")) {
    std::string value = parameters["window-size"];
    if (not value.empty()) {
      config.window_size = std::stoul(value);
    }
  }

  // Relative error between approximate and original values should be
  // no more than 5%, default = 1;
  config.error_rate = tol;

  // Size of each element
  config.element_byte_size = type_size;

  // Use either BSplines or Wavelets.
  config.transform = ISABELA_BSPLINES;
  // config.transform = ISABELA_WAVELETS;

  // Setup compression (deflate) with isabela_config
  status = isabelaDeflateInit (&i_strm, type_size, &config);
  assert (status == ISABELA_SUCCESS);

  i_strm.next_in = input;
  i_strm.avail_in = type_size * numel;
  i_strm.next_out = output;

  // Perform compression
  status = isabelaDeflate (&i_strm, ISABELA_FINISH);
  assert (status == ISABELA_SUCCESS);

  size_t csize = i_strm.avail_out;

  // Cleanup
  status = isabelaDeflateEnd (&i_strm);
  assert (status == ISABELA_SUCCESS);

  timer.stop();
  bytes = csize;
  auto const input_bytes = static_cast<float>(type_size * numel);

  log << std::endl << name;
  log << " ~ InputBytes: " << input_bytes;
  log << ", OutputBytes: " << csize;
  log << ", cRatio: " << input_bytes / static_cast<float>(csize);
  log << ", #elements: " << numel << std::endl;
  log << name << " ~ CompressTime: " << timer.getDuration() << " s" << std::endl;

  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
int IsabelaCompressor::decompress
  (void*& input, void*& output, std::string type, size_t type_size, size_t* n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();

  output = malloc(type_size * numel);

  enum ISABELA_status status;
  struct isabela_stream i_strm {};
  struct isabela_config config {};

  double tol = 1E-3;
  if (parameters.count("tolerance")) {
    std::string value = parameters["tolerance"];
    if (not value.empty()) {
      tol = std::stod(value);
    }
  }

  int pcnt = 30;
  if (parameters.count("pcnt")) {
    std::string value = parameters["pcnt"];
    if (not value.empty()) {
      pcnt = std::stoi(value);
    }
  }

  config.window_size = 1024;
  if (parameters.count("window-size")) {
    std::string value = parameters["window-size"];
    if (not value.empty()) {
      config.window_size = std::stoul(value);
    }
  }

  config.ncoefficients = pcnt;
  config.error_rate = tol;
  config.element_byte_size = type_size;
  config.transform = ISABELA_BSPLINES;

  // Setup compression (deflate) with isabela_config
  status = isabelaInflateInit (&i_strm, type_size, &config);

  i_strm.next_in = input;
  i_strm.avail_in = type_size * numel;
  i_strm.next_out = output;

  // Perform Decompression
  status = isabelaInflate(&i_strm, ISABELA_FINISH);
  if (status != ISABELA_SUCCESS) {
    std::cerr << "Decompression failed: "<< status << std::endl;
    return EXIT_FAILURE;
  }

  // Cleanup
  status = isabelaInflateEnd(&i_strm);
  if (status != ISABELA_SUCCESS) {
    std::cerr << "Cleanup failed: "<< status << std::endl;
    return EXIT_FAILURE;
  }

  std::free(input);
  input= nullptr;

  log << name << " ~ DecompressTime: " << timer.getDuration() << " s"<< std::endl;

  return EXIT_SUCCESS;
}
/* -------------------------------------------------------------------------- */
#endif