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

#include <sstream>
#include <cstring>
/* -------------------------------------------------------------------------- */
#include "compressors/kernels/zfp.hpp"
#include "utils/timer.h"
/* -------------------------------------------------------------------------- */

zfp_type ZFPCompressor::getZfpType(std::string const& type) const {
  if (type == "float")
    return zfp_type_float;
  else if (type == "double")
    return zfp_type_double;
  else if (type == "int32_t" or type == "int16_t")
    return zfp_type_int32;
  else if (type == "int64_t")
    return zfp_type_int64;
  else
    throw std::runtime_error("invalid type");
}

/* -------------------------------------------------------------------------- */
int ZFPCompressor::compress
  (void *input, void *&output, std::string data_type, size_t type_size, size_t* n) {

  size_t numel = n[0];
  numDims = 1;
  for (int i = 1; i < 5; i++) {
    if (n[i] != 0) {
      numel *= n[i];
      numDims++;
    }
  }

  // Read in json compression parameters
  double abs = 1E-3;
  int rel = 32;
  bool compressionTypeAbs = true;

  if (parameters.count("abs")) {
    std::string value = parameters["abs"];
    abs = std::stod(value);
    compressionTypeAbs = true;
  }
  else if (parameters.count("rel")) {
    std::string value = parameters["rel"];
    rel = std::stoi(value);
    compressionTypeAbs = false;
  }

  Timer timer;
  timer.start();

  zfp_type type = getZfpType(data_type);

  // allocate meta data for the 1D input array
  zfp_field* field = nullptr;

  if (numDims == 1)
    field = zfp_field_1d(input, type, numel);
  else if (numDims == 2)
    field = zfp_field_2d(input, type, n[1], n[0]);
  else if (numDims == 3)
    field = zfp_field_3d(input, type, n[2], n[1], n[0]);
  else if (numDims == 4)
    field = zfp_field_4d(input, type, n[3], n[2], n[1], n[0]);

  // allocate meta data for a compressed stream
  zfp_stream *zfp = zfp_stream_open(nullptr);

  // set absolute/relative error tolerance
  if (compressionTypeAbs)
    zfp_stream_set_accuracy(zfp, abs);
  else
    zfp_stream_set_precision(zfp, rel);

  //allocate buffer for compressed data
  size_t bufsize = zfp_stream_maximum_size(zfp, field);
  output = malloc(bufsize);

  // associate bit stream with allocated buffer
  bitstream *stream = stream_open(output, bufsize);
  zfp_stream_set_bit_stream(zfp, stream);
  zfp_stream_rewind(zfp);

  // Compress
  size_t zfpsize = zfp_compress(zfp, field);
  if (not zfpsize) {
    std::cerr << "Compression failed: " << zfpsize << std::endl;
    return EXIT_FAILURE;
  }

  zfp_field_free(field);
  zfp_stream_close(zfp);
  stream_close(stream);

  zfpCompressedSize = zfpsize;
  bytes = zfpsize;
  timer.stop();

  log << std::endl << name;
  log << " ~ InputBytes: " << type_size * numel;
  log << ", OutputBytes: " << bytes;
  log << ", cRatio: " << type_size * numel / static_cast<float>(bytes);
  log << ", #elements: " << numel << std::endl;
  log << name << " ~ CompressTime: " << timer.getDuration() << " s"<< std::endl;

  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
int ZFPCompressor::decompress
  (void *&input, void *&output, std::string data_type, size_t type_size, size_t* n) {

  numDims = 1;
  size_t numel = n[0];
  for (int i = 1; i < 5; i++)
    if (n[i] != 0) {
      numel *= n[i];
      numDims++;
    }

  // Read in json compression parameters
  double abs = 1E-3;
  int rel = 32;
  bool compressionTypeAbs = true;

  if (parameters.count("abs")) {
    std::string value = parameters["abs"];
    abs = std::stod(value);
    compressionTypeAbs = true;
  }
  else if (parameters.count("rel")) {
    std::string value = parameters["rel"];
    rel = std::stoi(value);
    compressionTypeAbs = false;
  }

  Timer timer;
  timer.start();

  zfp_type type = getZfpType(data_type);

  // allocate meta data  array of decompressed data
  output = std::malloc(numel * type_size);

  zfp_field* field = nullptr;

  if (numDims == 1)
    field = zfp_field_1d(output, type, numel);
  else if (numDims == 2)
    field = zfp_field_2d(output, type, n[1], n[0]);
  else if (numDims == 3)
    field = zfp_field_3d(output, type, n[2], n[1], n[0]);
  else if (numDims == 4)
    field = zfp_field_4d(output, type, n[3], n[2], n[1], n[0]);

  // allocate meta data for a compressed stream
  zfp_stream* zfp = zfp_stream_open(nullptr);

  // set absolute/relative error tolerance
  if (compressionTypeAbs)
    zfp_stream_set_accuracy(zfp, abs);
  else
    zfp_stream_set_precision(zfp, rel);

  // allocate buffer for decompressed data and transfer data
  size_t bufsize = zfp_stream_maximum_size(zfp, field);
  void *buffer = std::malloc(bufsize);
  std::memcpy(buffer, input, zfpCompressedSize);

  // associate bit stream with allocated buffer
  bitstream *stream = stream_open(buffer, zfpCompressedSize);
  zfp_stream_set_bit_stream(zfp, stream);
  zfp_stream_rewind(zfp);

  // DeCompress
  size_t zfpsize = zfp_decompress(zfp, field);
  if (not zfpsize) {
    std::cerr << "Decompression failed: " << zfpsize << std::endl;
    return EXIT_FAILURE;
  }

  std::free(buffer);
  zfp_field_free(field);
  zfp_stream_close(zfp);
  stream_close(stream);
  timer.stop();

  log << name << " ~ DecompressTime: " << timer.getDuration() << " s"<< std::endl;
  return EXIT_SUCCESS;
}
/* -------------------------------------------------------------------------- */