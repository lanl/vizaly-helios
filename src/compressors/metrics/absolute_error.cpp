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
#include "compressors/metrics/absolute_error.h"
/* -------------------------------------------------------------------------- */
void absoluteError::execute(void* original, void* approx, size_t n) {
  std::vector<double> error(n);

  double local_sum_error = 0;

  for (std::size_t i = 0; i < n; ++i) {
    error[i] = std::abs(static_cast<float*>(original)[i] - static_cast<float*>(approx)[i]);
    local_sum_error += error[i];
  }

  double total_max_error = 0;
  double local_max_error = *std::max_element(error.begin(), error.end());
  local_val = local_max_error;

  MPI_Allreduce(&local_max_error, &total_max_error, 1, MPI_DOUBLE, MPI_MAX, comm);
  total_val = total_max_error;

  log << "-Max Abs Error: " << total_max_error << std::endl;

  // Additional debug metrics, only in run_log
  // Global total sum of error
  double total_sum_error = 0;
  MPI_Allreduce(&local_sum_error, &total_sum_error, 1, MPI_DOUBLE, MPI_SUM, comm);

  // Global number of values
  size_t global_n = 0;
  MPI_Allreduce(&n, &global_n, 1, MPI_UNSIGNED_LONG_LONG, MPI_SUM, comm);

  // Compute mean
  double mean_error = total_sum_error / global_n;
  log << " Total Abs Error: " << total_sum_error << std::endl;
  log << " Mean Abs Error: " << mean_error << std::endl;

  MPI_Barrier(comm);

#if ENABLE_HISTOGRAM
  if (parameters.count("histogram")) {
    // Compute histogram of values
    if (total_max_error != 0) {
      std::vector<float> histogram;
      int numBins = 1024;
      std::vector<size_t> localHistogram(numBins, 0);
      double binSize = total_max_error / numBins;

      for (std::size_t i = 0; i < n; ++i) {
        double err = std::abs(
          static_cast<float*>(original)[i] - static_cast<float*>(approx)[i]
        );

        int binPos = static_cast<int>(err / binSize);
        if (binPos >= numBins)
          binPos = binPos - 1;
        localHistogram[binPos]++;
      }

      histogram.resize(numBins);

      std::vector<size_t> globalHistogram(numBins, 0);
      MPI_Allreduce(&localHistogram[0], &globalHistogram[0],
        numBins, MPI_UNSIGNED_LONG_LONG, MPI_SUM, comm);

      auto total_size = static_cast<float>(global_n);
      for (int i = 0; i < numBins; ++i)
        histogram[i] = static_cast<float>(globalHistogram[i]) / total_size;

      // Output histogram as a python script file
      if (rank == 0)
        additionalOutput = python_histogram(numBins, 0.0, total_max_error, histogram);
    }
  }
#endif
}
