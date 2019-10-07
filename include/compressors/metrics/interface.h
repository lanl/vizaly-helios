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
#include <string>
#include <sstream>
#include <unordered_map>
#include <mpi.h>
/* -------------------------------------------------------------------------- */
class MetricInterface {

public:
  virtual void init(MPI_Comm _comm) = 0;
  virtual void execute(void *original, void *approx, size_t n) = 0;
  virtual void close() = 0;

  double getLocalValue() { return local_val; }
  double getGlobalValue() { return total_val; }
  std::string getName() { return name; }
  std::string getLog() { return log.str(); }
  void clearLog() { log.str(""); }

  std::string getMetricInfo() {
    std::stringstream info;
    info << std::endl << "Metric: " << name << std::endl;
    return info.str();
  }

protected:
  double local_val = 0.;
  double total_val = 0.;
  std::string name = "";
  std::stringstream log {};

  int rank = 0;
  int nb_ranks = 0;
  MPI_Comm comm = MPI_COMM_NULL;

public:
  std::unordered_map<std::string, std::string> parameters {};
  std::string additionalOutput = "";   // for histograms
};
/* -------------------------------------------------------------------------- */