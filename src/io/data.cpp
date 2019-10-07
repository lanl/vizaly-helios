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

#include "io/data.h"
#include <cassert>
#include "utils/memory.h"
/* -------------------------------------------------------------------------- */
namespace gio {

/* -------------------------------------------------------------------------- */
std::string to_string(Type data_type) {
  switch (data_type) {
    case Type::Float:  return "float";
    case Type::Double: return "double";
    case Type::Int8:   return "int8_t";
    case Type::Int16:  return "int16_t";
    case Type::Int32:  return "int32_t";
    case Type::Int64:  return "int64_t";
    case Type::Uint8:  return "uint8_t";
    case Type::Uint16: return "uint16_t";
    case Type::Uint32: return "uint32_t";
    case Type::Uint64: return "uint64_t";
    default: return "";
  }
}
/* -------------------------------------------------------------------------- */
void Data::init(
  int in_id, std::string const& in_name,
  int in_size, bool in_is_float, bool in_is_signed,
  bool in_x_var, bool in_y_var, bool in_z_var, void *in_data)
{
  id       = in_id;
  name     = in_name;
  size     = in_size;
  is_float  = in_is_float;
  is_signed = in_is_signed;
  x_var     = in_x_var;
  y_var     = in_y_var;
  z_var     = in_z_var;
  data     = in_data;
  do_write  = false;
}

/* -------------------------------------------------------------------------- */
bool Data::determineDataType() {
  if (is_float) {
    switch (size) {
      case 4: data_type = Type::Float; break;
      case 8: data_type = Type::Double; break;
      default: return false;
    }
  } else {
    if (is_signed) {
      switch (size) {
        case 1: data_type = Type::Int8; break;
        case 2: data_type = Type::Int16; break;
        case 4: data_type = Type::Int32; break;
        case 8: data_type = Type::Int64; break;
        default: return false;
      }
    } else {
      switch (size) {
        case 1: data_type = Type::Uint8; break;
        case 2: data_type = Type::Uint16; break;
        case 4: data_type = Type::Uint32; break;
        case 8: data_type = Type::Uint64; break;
        default: return false;
      }
    }
  }
  return true;
}

/* -------------------------------------------------------------------------- */
bool Data::allocate(int offset) {
  assert(offset > 0);
  determineDataType();
  return Memory::allocate(data, data_type, nb_elems, offset);
}

/* -------------------------------------------------------------------------- */
bool Data::release() {
  return Memory::release(data, data_type);
}
/* -------------------------------------------------------------------------- */
} // namespace gio