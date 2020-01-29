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
#include <iostream>
#include <fstream>
#include <cassert>

#include "utils/json.h"
#include "utils/tools.h"
#include "io/interface.h"
#include "io/hacc.h"
#include <compressors/kernels/factory.h>
/* -------------------------------------------------------------------------- */
class Density {

public:
  Density() = default;
  Density(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm);
  Density(Density const&) = delete;
  Density(Density&&) noexcept = delete;
  ~Density() = default;

  void run();

private:

  bool cacheData();
  void computeFrequencies();
  void dumpHistogram();
  void setCompressionFactors();

  // particle to density field mapping methods
  long deduceDensityIndex(const float* particle) const;
  int deduceBucketIndex(float const& rho) const;
  void bucketParticles();
  std::vector<float> process(std::vector<float> const& data);
  void dump();

  static int const dim = 3;

  // IO
  std::string json_path;
  std::string input_hacc;
  std::vector<std::pair<std::string,long>> inputs;    // local to this rank
  std::string output_plot;
  std::unique_ptr<HACCDataLoader> ioMgr;

  // particle meta-data
  int cells_per_axis = 0;                // cartesian grid
  float coords_min[dim] = { 0.,0.,0.};
  float coords_max[dim] = { 0.,0.,0.};
  long local_particles = 0;
  long total_particles = 0;

  // histogram
  int nb_bins = 0;
  long local_rho_count = 0;
  long total_rho_count = 0;
  double local_rho_min = 0.;
  double local_rho_max = 0.;
  double total_rho_min = 0.;
  double total_rho_max = 0.;

  // actual datasets
  std::vector<float> coords[dim];                  // size: local_particles
  std::vector<float> velocs[dim];                  // size: local_particles
  std::vector<float> density_field;                // size: local_rho_count
  std::vector<long> histogram;                     // size: nb_bins
  std::vector<std::vector<long>> buckets;          // size: nb_bins
  std::vector<int> bits;                           // size: nb_bins
  std::vector<float> decompressed[2 * dim];

  // MPI
  int my_rank  = 0;
  int nb_ranks = 0;
  MPI_Comm comm = MPI_COMM_NULL;

};