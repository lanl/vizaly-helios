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
#include "compressors/metrics/min_max.h"
/* -------------------------------------------------------------------------- */
void minmaxMetric::execute(void *original, void *approx, size_t n) {

  auto* raw_data = static_cast<float*>(original);

  double local_max = -99999999999;
  double local_min = 99999999999;

  for (std::size_t i = 0; i < n; ++i) {
    if (raw_data[i] > local_max) local_max = raw_data[i];
    if (raw_data[i] < local_min) local_min = raw_data[i];
  }

  double global_max = 0;
  double global_min = 0;
  MPI_Allreduce(&local_max, &global_max, 1, MPI_DOUBLE, MPI_MAX, comm);
  MPI_Allreduce(&local_min, &global_min, 1, MPI_DOUBLE, MPI_MIN, comm);

  log << " local_minmax: " << local_min << " " << local_max << std::endl;
  log << "- min, max: (" << global_min << ", " << global_max << ")" << std::endl;

  MPI_Barrier(comm);

  // Just report max for now
  local_val = local_max;
  total_val = global_max;

#if ENABLE_HISTOGRAM
  auto* zip_data = static_cast<float*>(approx);

  if (parameters.count("histogram")) {
    // Compute histogram of values
    if (global_max != 0) {
      std::vector<float> histogram;
      int numBins = 1024;
      std::vector<size_t> localHistogram(numBins, 0);
      double binSize = (global_max - global_min) / numBins;

      for (std::size_t i = 0; i < n; ++i) {
        // Retrieve the "approximated" value
        // Lossless comp: Original data
        // Lossy comp: Approx data
        double const range = global_max - global_min;
        double const value = range * ((zip_data[i] - global_min) / range);
        int binPos = static_cast<int>(value / binSize);
        if (binPos >= numBins)
          binPos = binPos - 1;

        localHistogram[binPos]++;
      }

      histogram.resize(numBins);

      std::vector<size_t> globalHistogram(numBins, 0);
      MPI_Allreduce(&localHistogram[0], &globalHistogram[0], numBins,
        MPI_UNSIGNED_LONG_LONG, MPI_SUM, comm);

      for (int i = 0; i < numBins; ++i)
        histogram[i] = (float) globalHistogram[i];

      // Output histogram as a python script file
      if (rank == 0)
        additionalOutput = python_histogram(numBins, global_min, global_max, histogram);
    }
  }
#endif
}