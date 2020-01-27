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

#include "density/density.h"
/* -------------------------------------------------------------------------- */

Density::Density(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm)
  : json_path(in_path),
    my_rank(in_rank),
    nb_ranks(in_nb_ranks),
    comm(in_comm) {
  assert(nb_ranks > 0);
  nlohmann::json json;
  std::string buffer;

  std::ifstream file(json_path);
  assert(file.is_open());
  assert(file.good());

  // parse params and do basic checks
  file >> json;

  assert(json["density"].count("input"));
  assert(json["density"].count("extents"));
  assert(json["density"]["extents"].count("min"));
  assert(json["density"]["extents"].count("max"));
  assert(json["density"].count("num_bins"));
  assert(json["density"].count("logs"));
  assert(json["density"].count("plots"));

  for (auto const& path : json["density"]["inputs"])
    inputs.emplace_back(path);

  int const partition_size = inputs.size();
  if (nb_ranks > 1 and partition_size % nb_ranks != 0)
    throw std::runtime_error("incompatible number of ranks and data partition");

  output_log = json["density"]["logs"];
  output_plot = json["density"]["plots"];
  num_bins = json["density"]["num_bins"];
  assert(num_bins > 0);

  extents[0] = json["density"]["extents"]["min"];
  extents[1] = json["density"]["extents"]["max"];
  assert(extents[0] < extents[1]);

  dataset.clear();
  histo.clear();
}

/* -------------------------------------------------------------------------- */
bool Density::load(std::string path, int offset) {

  // load binary file
  debug_log.clear();
  debug_log.str("");
  debug_log << "Loading density file '" << path << "' ... " << std::flush;

  // TODO: create

  return false;
}


/* -------------------------------------------------------------------------- */
bool Density::run() {




}
/* -------------------------------------------------------------------------- */