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

#include <cmath>
#include <string>
#include <sstream>
#include "compressors/metrics/psnr_error.h"
/* -------------------------------------------------------------------------- */

inline void psnrError::execute(void *original, void *approx, size_t n) {

  auto* raw_data = static_cast<float*>(original);
  auto* zip_data = static_cast<float*>(approx);

  double local_max = -999999999;
  double local_mse = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (raw_data[i] > local_max) {
      local_max = raw_data[i]; // implicit cast
    }
    local_mse += std::pow(raw_data[i] - zip_data[i], 2.);
  }

  // Local quantity
  double local_psnr = 10 * std::log10(pow(local_max, 2.0) / (local_mse / n));
  local_val = local_psnr;

  // Global quantities
  double global_max = 0;
  double global_mse = 0;
  size_t global_n = 0;

  MPI_Allreduce(&local_max, &global_max, 1, MPI_DOUBLE, MPI_MAX, comm);
  MPI_Allreduce(&local_mse, &global_mse, 1, MPI_DOUBLE, MPI_SUM, comm);
  MPI_Allreduce(&n, &global_n, 1, MPI_UNSIGNED_LONG_LONG, MPI_SUM, comm);

  global_mse /= global_n;
  total_val = 10 * std::log10(std::pow(global_max, 2.0) / global_mse);

  log << " local_psnr: " << local_psnr << std::endl;
  log << "- psnr: " << total_val << std::endl;

  MPI_Barrier(comm);
}
/* -------------------------------------------------------------------------- */
