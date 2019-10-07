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

#if ENABLE_SZ
/* -------------------------------------------------------------------------- */
#include <sstream>
#include "compressors/kernels/sz.hpp"
#include "utils/timer.h"
/* -------------------------------------------------------------------------- */
int SZCompressor::compress
  (void* input, void*& output, std::string type, size_t type_size, size_t* n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();
  SZ_Init(nullptr);

  int mode = PW_REL; // Default by Sheng, PW_REL = 10
  std::string _mode = "PW_REL";

  double relTol = 0.;
  if (parameters.count("rel")) {
    std::string value = parameters["rel"];
    if (not value.empty()) {
      relTol = std::stod(value);
      mode = REL;
      _mode = "REL";
    }
  }

  double absTol = 0.;
  if (parameters.count("abs")) {
    std::string value = parameters["abs"];
    if (not value.empty()) {
      relTol = std::stod(value);
      mode = ABS;
      _mode = "ABS";
    }
  }

  double powerTol = 0.;
  if (parameters.count("pw_rel")) {
    std::string value = parameters["pw_rel"];
    if (not value.empty()) {
      relTol = std::stod(value);
      mode = PW_REL;
      _mode = "PW_REL";
      // Unknown mode, just fill in input to SZ
    }
  }

  size_t size = 0;
  uint8_t* data = SZ_compress_args(
    SZ_FLOAT, static_cast<float*>(input), &size, mode,
    absTol, relTol, powerTol, n[4], n[3], n[2], n[1], n[0]
  );

  output = data;
  bytes = size;
  timer.stop();
  auto const input_bytes = static_cast<float>(type_size * numel);

  log << std::endl << name;
  log << " ~ InputBytes: " << input_bytes;
  log << ", OutputBytes: " << size;
  log << ", cRatio: " << input_bytes / static_cast<float>(size);
  log << ", #elements: " << numel << std::endl;

  log << " ~ Mode used: " << _mode;
  log << ", abs: " << absTol;
  log << ", rel: " << relTol;
  log << ", pw_tol: " << powerTol;
  log << " val: " << n[4] <<", "<< n[3] <<", "<< n[2] <<", "<<n[1] <<", "<< n[0];
  log << std::endl;

  log << name << " ~ CompressTime: " << timer.getDuration() <<" s"<< std::endl;
  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
int SZCompressor::decompress
  (void*& input, void*& output, std::string type, size_t type_size, size_t* n) {

  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0)
      numel *= n[i];

  Timer timer;
  timer.start();

  output = SZ_decompress(
    SZ_FLOAT, static_cast<std::uint8_t*>(input),
    bytes, n[4], n[3], n[2], n[1], n[0]
  );

  timer.stop();

  std::free(input);
  input = nullptr;

  log << name << " ~ DecompressTime: " << timer.getDuration() << " s"<< std::endl;
  return EXIT_SUCCESS;
}
/* -------------------------------------------------------------------------- */
#endif