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

#include <cmath>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include "compressors/metrics/relative_error.h"
/* -------------------------------------------------------------------------- */
double relativeError::relError(float original, float approx, double tolerance) {
  double absolute_error = std::abs(original - approx);
  if (std::abs(original) < tolerance) {
    return absolute_error;
  }
  return absolute_error / std::abs(original);
}

/* -------------------------------------------------------------------------- */
void relativeError::execute(void *original, void *approx, size_t n) {
  std::vector<double> error(n);

  double local_sum_error = 0;

  for (std::size_t i = 0; i < n; ++i) {
    error[i] =
      relError(static_cast<float*>(original)[i], static_cast<float*>(approx)[i], 1);
    local_sum_error += error[i];
  }

  double local_max_error = *std::max_element(error.begin(), error.end());
  local_val = local_max_error;

  double total_max_error = 0;
  MPI_Allreduce(&local_max_error, &total_max_error, 1, MPI_DOUBLE, MPI_MAX, comm);
  total_val = total_max_error;

  log << "-Max Rel Error: " << total_max_error << std::endl;

  // Additional debug metrics, only in run_log
  // Global total sum of error
  double total_sum_error = 0;
  MPI_Allreduce(&local_sum_error, &total_sum_error, 1, MPI_DOUBLE, MPI_SUM, comm);

  // Global number of values
  size_t global_n = 0;
  MPI_Allreduce(&n, &global_n, 1, MPI_UNSIGNED_LONG_LONG, MPI_SUM, comm);

  // Compute mean
  double mean_rel_err = total_sum_error / global_n;
  log << " Total Rel Error: " << total_sum_error << std::endl;
  log << " Mean Rel Error: " << mean_rel_err << std::endl;

  MPI_Barrier(comm);
}
/* -------------------------------------------------------------------------- */