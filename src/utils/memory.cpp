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

#include "utils/memory.h"
/* -------------------------------------------------------------------------- */
/* static*/ std::map<std::string, size_t> Memory::sizeOf = {
  {"int",      sizeof(int)},
  {"float",    sizeof(float)},
  {"double",   sizeof(double)},
  {"int8_t",   sizeof(int8_t)},
  {"int16_t",  sizeof(int16_t)},
  {"int32_t",  sizeof(int32_t)},
  {"int64_t",  sizeof(int64_t)},
  {"uint8_t",  sizeof(uint8_t)},
  {"uint16_t", sizeof(uint16_t)},
  {"uint32_t", sizeof(uint32_t)},
  {"uint64_t", sizeof(uint64_t)}
};

/* -------------------------------------------------------------------------- */
void Memory::start() {
  getMemorySize(before_size, before_rss);
}

/* -------------------------------------------------------------------------- */
void Memory::stop() {
  size_t after_size, after_rss;
  getMemorySize(after_size, after_rss);

  usage_size = after_size - before_size;
  usage_rss = after_rss - before_rss;
}

/* -------------------------------------------------------------------------- */
bool Memory::allocate(void *&data, gio::Type data_type, size_t nb_elems, int offset) {

  switch (data_type) {
    case gio::Type::Float:  data = new float   [nb_elems + offset]; break;
    case gio::Type::Double: data = new double  [nb_elems + offset]; break;
    case gio::Type::Int:    data = new int     [nb_elems + offset]; break;
    case gio::Type::Int8:   data = new int8_t  [nb_elems + offset]; break;
    case gio::Type::Int16:  data = new int16_t [nb_elems + offset]; break;
    case gio::Type::Int32:  data = new int32_t [nb_elems + offset]; break;
    case gio::Type::Int64:  data = new int64_t [nb_elems + offset]; break;
    case gio::Type::Uint8:  data = new uint8_t [nb_elems + offset]; break;
    case gio::Type::Uint16: data = new uint16_t[nb_elems + offset]; break;
    case gio::Type::Uint32: data = new uint32_t[nb_elems + offset]; break;
    case gio::Type::Uint64: data = new uint64_t[nb_elems + offset]; break;
    default: return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
bool Memory::release(void *&data, gio::Type data_type) {
  if (data != nullptr) {
    switch (data_type) {
      case gio::Type::Float:  delete[] (float*)    data; break;
      case gio::Type::Double: delete[] (double*)   data; break;
      case gio::Type::Int:    delete[] (int*)      data; break;
      case gio::Type::Int8:   delete[] (int8_t*)   data; break;
      case gio::Type::Int16:  delete[] (int16_t*)  data; break;
      case gio::Type::Int32:  delete[] (int32_t*)  data; break;
      case gio::Type::Int64:  delete[] (int64_t*)  data; break;
      case gio::Type::Uint8:  delete[] (uint8_t*)  data; break;
      case gio::Type::Uint16: delete[] (uint16_t*) data; break;
      case gio::Type::Uint32: delete[] (uint32_t*) data; break;
      case gio::Type::Uint64: delete[] (uint64_t*) data; break;
      default: return false;
    }
    data = nullptr;
  }

  return true;
}

/* -------------------------------------------------------------------------- */
double Memory::getMemoryInUseInB() {
  size_t after_size, after_rss;
  getMemorySize(after_size, after_rss);

  return static_cast<double>(after_size - before_size);
}

/* -------------------------------------------------------------------------- */
double Memory::getMemoryInUseInKB() {
  return getMemoryInUseInB() / kilobyte;
}

/* -------------------------------------------------------------------------- */
double Memory::getMemoryInUseInMB() {
  return getMemoryInUseInB() / megabyte;
}

/* -------------------------------------------------------------------------- */
void Memory::getMemorySize(size_t& size, size_t& rss) {
#if defined(__linux__)
  size = 0;
  rss  = 0;

  FILE* file = fopen("/proc/self/statm", "r");
  if (file == NULL)
    return;

  int count = fscanf(file, "%lu%lu", &size, &rss);
  if (count != 2) {
    fclose(file);
    return;
  }

  size_t const page_size = getpagesize();

  size *= page_size;
  rss  *= page_size;
  fclose(file);

#elif defined(WIN32)
  // virtual and physical memory by current process
  _PROCESS_MEMORY_COUNTERS_EX pmc;
  GetProcessMemoryInfo(
    GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc)
  );

  size_t virtualMemUsedByMe = pmc.PrivateUsage;
  size_t physMemUsedByMe = pmc.WorkingSetSize;

  size = virtualMemUsedByMe;
  rss = physMemUsedByMe;

#elif defined(__APPLE__) && defined(__MACH__)
  struct mach_task_basic_info info {};
  mach_msg_type_number_t infoCount = MACH_TASK_BASIC_INFO_COUNT;
  auto const status = task_info(
    mach_task_self(), MACH_TASK_BASIC_INFO,
    (task_info_t)&info, &infoCount
  );

  if (status != KERN_SUCCESS)
    rss = (size_t)0L;		/* Can't access? */

  rss = (size_t) info.resident_size;
  size = (size_t) info.virtual_size;
#endif
}
