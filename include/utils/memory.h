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

#pragma once

/* -------------------------------------------------------------------------- */
#if defined(__APPLE__) && defined(__MACH__)
  #include <mach/mach.h>
#elif defined(__unix__) || defined(__unix) || defined(unix)
  #include <sys/sysinfo.h>
  #include <unistd.h>
#elif defined(WIN32)
  #include <windows.h>
  #include "psapi.h" //MSVC Performance query
#endif
/* -------------------------------------------------------------------------- */
#include <stdio.h>
#include <iostream>
#include <map>
/* -------------------------------------------------------------------------- */
class Memory {

public:
   Memory() = default;
  ~Memory() = default;

  void start();
  void stop();

  unsigned long getMemorySizeInB() { return usage_size; }
  double getMemorySizeInKB() { return usage_size / 1024.0; }
  double getMemorySizeInMB() { return usage_size / (1024.0 * 1024.0); }
  double getMemoryInUseInB();
  double getMemoryInUseInKB();
  double getMemoryInUseInMB();


  unsigned long getMemoryRSSInB() { return usage_rss; }
  double getMemoryRSSInKB() { return usage_rss / 1024.0; }
  double getMemoryRSSInMB() { return usage_rss / (1024.0 * 1024.0); }

  // static data and methods
  static std::map<std::string, size_t> sizeOf;
  static bool allocate(void *&data, std::string type, size_t nb_elems=1, int offset=0);
  static bool release(void *&data, std::string type);

private:
  void getMemorySize(unsigned long &size, unsigned long &rss);

  unsigned long before_size = 0;
  unsigned long usage_size = 0;  // program size
  unsigned long before_rss = 0;
  unsigned long usage_rss = 0;  // resident set size
};
/* -------------------------------------------------------------------------- */