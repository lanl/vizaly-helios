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
/* -------------------------------------------------------------------------- */
namespace gio {

/* -------------------------------------------------------------------------- */
std::string to_string(TYPE datatype) {
  switch (datatype) {
    case TYPE::FLOAT:  return "float";
    case TYPE::DOUBLE: return "double";
    case TYPE::INT08:  return "int8_t";
    case TYPE::INT16:  return "int16_t";
    case TYPE::INT32:  return "int32_t";
    case TYPE::INT64:  return "int64_t";
    case TYPE::UINT08: return "uint8_t";
    case TYPE::UINT16: return "uint16_t";
    case TYPE::UINT32: return "uint32_t";
    case TYPE::UINT64: return "uint64_t";
    default: return "";
  }
}
/* -------------------------------------------------------------------------- */
void Data::init(
  int _id, std::string _name,
  int _size, bool _isFloat, bool _isSigned,
  bool _xVar, bool _yVar, bool _zVar, void *_data)
{
  id       = _id;
  name     = _name;
  size     = _size;
  isFloat  = _isFloat;
  isSigned = _isSigned;
  xVar     = _xVar;
  yVar     = _yVar;
  zVar     = _zVar;
  data     = _data;
  doWrite  = false;
}

/* -------------------------------------------------------------------------- */
bool Data::determineDataType() {
  if (isFloat) {
    switch (size) {
      case 4: dataType = TYPE::FLOAT; break;
      case 8: dataType = TYPE::DOUBLE; break;
      default: return false;
    }
  } else {
    if (isSigned) {
      switch (size) {
        case 1: dataType = TYPE::INT08; break;
        case 2: dataType = TYPE::INT16; break;
        case 4: dataType = TYPE::INT32; break;
        case 8: dataType = TYPE::INT64; break;
        default: return false;
      }
    } else {
      switch (size) {
        case 1: dataType = TYPE::UINT08; break;
        case 2: dataType = TYPE::UINT16; break;
        case 4: dataType = TYPE::UINT32; break;
        case 8: dataType = TYPE::UINT64; break;
        default: return false;
      }
    }
  }
  return true;
}

/* -------------------------------------------------------------------------- */
bool Data::allocateMem(int offset) {
  assert(offset >= 0);
  determineDataType();

  switch (dataType) {
    case TYPE::FLOAT:  data = new float   [numElements + offset]; break;
    case TYPE::DOUBLE: data = new double  [numElements + offset]; break;
    case TYPE::INT08:  data = new int8_t  [numElements + offset]; break;
    case TYPE::INT16:  data = new int16_t [numElements + offset]; break;
    case TYPE::INT32:  data = new int32_t [numElements + offset]; break;
    case TYPE::INT64:  data = new int64_t [numElements + offset]; break;
    case TYPE::UINT08: data = new uint8_t [numElements + offset]; break;
    case TYPE::UINT16: data = new uint16_t[numElements + offset]; break;
    case TYPE::UINT32: data = new uint32_t[numElements + offset]; break;
    case TYPE::UINT64: data = new uint64_t[numElements + offset]; break;
    default: return false;
  }

  return true;
}

/* -------------------------------------------------------------------------- */
bool Data::deAllocateMem() {
  if (data != nullptr) {
    switch (dataType) {
      case TYPE::FLOAT:  delete[] (float*)    data; break;
      case TYPE::DOUBLE: delete[] (double*)   data; break;
      case TYPE::INT08:  delete[] (int8_t*)   data; break;
      case TYPE::INT16:  delete[] (int16_t*)  data; break;
      case TYPE::INT32:  delete[] (int32_t*)  data; break;
      case TYPE::INT64:  delete[] (int64_t*)  data; break;
      case TYPE::UINT08: delete[] (uint8_t*)  data; break;
      case TYPE::UINT16: delete[] (uint16_t*) data; break;
      case TYPE::UINT32: delete[] (uint32_t*) data; break;
      case TYPE::UINT64: delete[] (uint64_t*) data; break;
      default: return false;
    }
  }

  data = nullptr;
  return true;
}
/* -------------------------------------------------------------------------- */
} // namespace gio