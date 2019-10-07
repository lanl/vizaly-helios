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
  enum class TYPE {
    FLOAT, DOUBLE,
    INT08, INT16, INT32, INT64,
    UINT08, UINT16, UINT32, UINT64
  };

  std::string to_string(TYPE datatype);

// Stores the genericIO data being read in
class Data {

public:
   Data() = default;
  ~Data() = default;

  void init(
    int _id, std::string _name,
    int _size, bool _isFloat, bool _isSigned,
    bool _xVar, bool _yVar, bool _zVar,
    void *_data = nullptr
  );

  void setNumElements(size_t _numElements) { numElements = _numElements; }
  bool determineDataType();
  bool allocateMem(int offset = 1);
  bool deAllocateMem();

public:
  int id = 0;
  int size = 0;           // in bytes
  size_t numElements = 0;
  std::string name = "";
  TYPE dataType = TYPE::INT32;

  bool isFloat  = false;
  bool isSigned = false;
  bool ghost    = false;
  bool xVar     = false;
  bool yVar     = false;
  bool zVar     = false;
  bool loadData = false;
  bool doWrite  = false;

  void* data = nullptr;
};
} // namespace gio