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
  unsigned long after_size, after_rss;
  getMemorySize(after_size, after_rss);

  usage_size = after_size - before_size;
  usage_rss = after_rss - before_rss;
}

/* -------------------------------------------------------------------------- */
// TODO: use gio::TYPE
bool Memory::allocate(void *&data, std::string type, size_t nb_elems, int offset) {
  if (type == "int")
    data = new int[nb_elems + offset];
  else if (type == "float")
    data = new float[nb_elems + offset];
  else if (type == "double")
    data = new double[nb_elems + offset];
  else if (type == "int8_t")
    data = new int8_t[nb_elems + offset];
  else if (type == "int16_t")
    data = new int16_t[nb_elems + offset];
  else if (type == "int32_t")
    data = new int32_t[nb_elems + offset];
  else if (type == "int64_t")
    data = new int64_t[nb_elems + offset];
  else if (type == "uint8_t")
    data = new uint8_t[nb_elems + offset];
  else if (type == "uint16_t")
    data = new uint16_t[nb_elems + offset];
  else if (type == "uint32_t")
    data = new uint32_t[nb_elems + offset];
  else if (type == "uint64_t")
    data = new uint64_t[nb_elems + offset];
  else
    return false;

  return true;
}

/* -------------------------------------------------------------------------- */
bool Memory::release(void *&data, std::string type) {

  if (data == nullptr) // already deallocated!
    return true;

  if (type == "int")
    delete[](int *) data;
  else if (type == "float")
    delete[](float *) data;
  else if (type == "double")
    delete[](double *) data;
  else if (type == "int8_t")
    delete[](int8_t *) data;
  else if (type == "int16_t")
    delete[](int16_t *) data;
  else if (type == "int32_t")
    delete[](int32_t *) data;
  else if (type == "int64_t")
    delete[](int64_t *) data;
  else if (type == "uint8_t")
    delete[](uint8_t *) data;
  else if (type == "uint16_t")
    delete[](uint16_t *) data;
  else if (type == "uint32_t")
    delete[](uint32_t *) data;
  else if (type == "uint64_t")
    delete[](uint64_t *) data;
  else
    return false;

  data = nullptr;
  return true;
}

/* -------------------------------------------------------------------------- */
double Memory::getMemoryInUseInB() {
  unsigned long after_size, after_rss;
  getMemorySize(after_size, after_rss);

  return (after_size - before_size);
}

/* -------------------------------------------------------------------------- */
double Memory::getMemoryInUseInKB() {
  unsigned long after_size, after_rss;
  getMemorySize(after_size, after_rss);

  return (after_size - before_size) / (1024.0);
}

/* -------------------------------------------------------------------------- */
double Memory::getMemoryInUseInMB() {
  unsigned long after_size, after_rss;
  getMemorySize(after_size, after_rss);

  return (after_size - before_size) / (1024.0 * 1024.0);
}

/* -------------------------------------------------------------------------- */
#if defined(__unix__) || defined(__unix) || defined(unix)
// From VisIt avt/Pipeline/Pipeline/avtMemory.cpp
void Memory::getMemorySize(unsigned long &size, unsigned long &rss)
{
  size = 0;
  rss  = 0;

  FILE *file = fopen("/proc/self/statm", "r");
  if (file == NULL)
    return;

  int count = fscanf(file, "%lu%lu", &size, &rss);
  if (count != 2) {
    fclose(file);
    return;
  }
  size *= (unsigned long) getpagesize();
  rss  *= (unsigned long) getpagesize();
  fclose(file);
}
#elif defined(WIN32)
void Memory::getMemorySize(unsigned long &size, unsigned long &rss)
{
  //Virtual Memory by current process
  PROCESS_MEMORY_COUNTERS_EX pmc;
  GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
  SIZE_T virtualMemUsedByMe = pmc.PrivateUsage;

  //Physical Memory Used by Current Process
  SIZE_T physMemUsedByMe = pmc.WorkingSetSize;

  size = virtualMemUsedByMe;
  rss = physMemUsedByMe;
}
#elif defined(__APPLE__) && defined(__MACH__)
void Memory::getMemorySize(unsigned long &size, unsigned long &rss)
{
  struct mach_task_basic_info info;
  mach_msg_type_number_t infoCount = MACH_TASK_BASIC_INFO_COUNT;
  if ( task_info( mach_task_self( ), MACH_TASK_BASIC_INFO,
                  (task_info_t)&info, &infoCount ) != KERN_SUCCESS )
    rss = (size_t)0L;		/* Can't access? */

  rss = (size_t) info.resident_size;
  size = (size_t) info.virtual_size;
}
#endif