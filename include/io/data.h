/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Authors: Pascal Grosset and Hoby Rakotoarivelo.
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
#include <string>
/* -------------------------------------------------------------------------- */
namespace gio {
  enum class Type {
    Float, Double, Int, Int8, Int16, Int32, Int64, Uint8, Uint16, Uint32, Uint64
  };

  std::string to_string(Type data_type);
/* -------------------------------------------------------------------------- */
class Data {

public:
   Data() = default;
  ~Data() = default;

  void init(
    int in_id, std::string const& in_name,
    int in_size, bool in_is_float, bool in_is_signed,
    bool in_x_var, bool in_y_var, bool in_z_var,
    void* in_data = nullptr
  );

  void setNumElements(size_t in_nb_elems) { nb_elems = in_nb_elems; }
  bool determineDataType();
  bool allocate(int offset = 1);
  bool release();

public:
  int id = 0;
  int size = 0;           // in bytes
  size_t nb_elems = 0;
  std::string name = "";
  Type data_type = Type::Int32;

  bool is_float  = false;
  bool is_signed = false;
  bool is_ghost  = false;
  bool x_var     = false;
  bool y_var     = false;
  bool z_var     = false;
  bool do_load   = false;
  bool do_write  = false;
  void* data = nullptr;
};
/* -------------------------------------------------------------------------- */
} // namespace gio