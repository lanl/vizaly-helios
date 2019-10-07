/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Authors: Pascal Grosset
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

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>


inline void writeFile(std::string filename, std::string log) {
  std::ofstream outputFile(filename.c_str(), std::ios::out);
  outputFile << log;
  outputFile.close();
}


inline void writeLog(std::string filename, std::string log) {
#ifndef NDEBUG
  std::ofstream outputFile((filename + ".log").c_str(), std::ios::out);
  outputFile << log;
  outputFile.close();
#endif
}


inline void writeLog(std::string filename, std::stringstream log) {
#ifndef NDEBUG
  std::ofstream outputFile((filename + ".log").c_str(), std::ios::out);
  outputFile << log.str();
  outputFile.close();
#endif
}


inline void appendLog(std::string filename, std::string log) {
#ifndef NDEBUG
  std::ofstream outputFile((filename + ".log").c_str(), std::ios::out | std::ios::app);
  outputFile << log;
  outputFile.close();
#endif
}

inline void appendLog(std::string filename, std::stringstream &log) {
#ifndef NDEBUG
  std::ofstream outputFile((filename + ".log").c_str(), std::ios::out | std::ios::app);
  outputFile << log.str();
  outputFile.close();

  log.str("");  // clears the log
#endif
}

