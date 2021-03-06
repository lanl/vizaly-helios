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
#if ENABLE_ZFP
/* -------------------------------------------------------------------------- */
#include "interface.h"
#include "zfp.h"
/* -------------------------------------------------------------------------- */
class ZFPCompressor: public CompressorInterface {

  enum Mode { zfp_ABS, zfp_REL, zfp_BIT };

public:
   ZFPCompressor() { name = "zfp"; }
  ~ZFPCompressor() = default;

  void init() override {}
  int compress(void* in, void*& out, std::string type, size_t type_size, size_t* n) override;
  int decompress(void*& in, void*& out, std::string data_type, size_t type_size, size_t* n) override;
  void close() override {}

  zfp_type getZfpType(std::string const& type) const;

private:
  size_t zfpCompressedSize = 0;
  int dims = 0;
  Mode zfp_mode_ = zfp_ABS;
};
/* -------------------------------------------------------------------------- */
#endif
