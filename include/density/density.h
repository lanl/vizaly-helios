/*
 * Copyright (c) 2019, Los Alamos National Laboratory
 * All rights reserved.
 *
 * Author: Hoby Rakotoarivelo
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
#include <mpi.h>
#include <cassert>

#include "utils/json.h"
#include "utils/tools.h"
#include "io/interface.h"
#include "io/hacc.h"
/* -------------------------------------------------------------------------- */
class Density {

public:
  Density() = default;
  Density(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm);
  Density(Density const&) = delete;
  Density(Density&&) noexcept = delete;
  ~Density() = default;

  bool run();

private:
  bool computeFrequencies(int i, float* data);
  void filterParticles();
  void dumpLogs();
  void generateHistogram();

  // IO
  std::string json_path;
  std::string input_full;
  std::string output_log;
  std::string output_gnu;
  std::stringstream debug_log;
  std::unique_ptr<HACCDataLoader> ioMgr;

  int num_bins = 0;

  long local_parts = 0;
  long total_parts = 0;
  long local_updated = 0;
  long total_updated = 0;

  // per-scalar data
  std::vector<size_t> count;
  std::vector<double> entropy;
  std::vector<std::vector<double>> frequency;

  // MPI
  int my_rank  = 0;
  int nb_ranks = 0;
  MPI_Comm comm = MPI_COMM_NULL;

};