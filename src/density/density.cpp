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

  assert(json["density"].count("inputs"));
  assert(json["density"].count("extents"));
  assert(json["density"]["extents"].count("min"));
  assert(json["density"]["extents"].count("max"));
  assert(json["density"].count("nb_bins"));
  assert(json["density"].count("logs"));
  assert(json["density"].count("plots"));

  // dispatch files to MPI ranks
  int partition_size = json["density"]["inputs"].size();
  bool rank_mismatch = (partition_size < nb_ranks) or (partition_size % nb_ranks != 0);

  if (nb_ranks == 1 or not rank_mismatch) {
    int offset = static_cast<int>(partition_size / nb_ranks);
    assert(offset);

    local_count = 0;
    for (int i = 0; i < offset; ++i) {
      int index = i + my_rank * offset;
      auto&& current = json["density"]["inputs"][index];
      inputs.emplace_back(current["data"], current["count"]);
      std::cout << "["<< my_rank <<"]: \""<< inputs.back().first << "\""<< std::endl;
      local_count += inputs.back().second;
    }

    // resize buffer
    density.resize(local_count);
    //density.resize(4 * offset);
  } else
    throw std::runtime_error("mismatch on number of ranks and data partition");

  output_log = json["density"]["logs"];
  output_plot = json["density"]["plots"];
  nb_bins = json["density"]["nb_bins"];
  assert(nb_bins > 0);

  extents[0] = json["density"]["extents"]["min"];
  extents[1] = json["density"]["extents"]["max"];
  assert(extents[0] < extents[1]);

  histo.clear();
}

/* -------------------------------------------------------------------------- */
bool Density::load(std::string const& path, long count, long offset) {

  std::ifstream file(path, std::ios::binary);
  if (not file.good())
    return false;

  debug_log << "Loading density file ... " << std::flush;

  auto buffer = reinterpret_cast<char*>(density.data() + offset);
  file.seekg(0, std::ios::beg);
  file.read(buffer, count * sizeof(float));
  file.close();

  debug_log << "done" << std::endl;
  return true;
}

/* -------------------------------------------------------------------------- */
bool Density::run() {

  debug_log.clear();
  debug_log.str("");

  // step 1: load current rank dataset in memory
  long offset = 0;
  for (auto&& current : inputs) {
    auto&& path = current.first;
    auto&& count = current.second;
    load(path, count, offset);
    offset += count;
  }

  debug_log << "["<< my_rank <<"]: "<< density.size() << " values loaded." << std::endl;

  // step 2: compute frequencies and histogram



#if DEBUG

  for (float const& value : density) {
    std::cout << "rank["<< my_rank <<"]: density = "<< value << std::endl;
  }
#endif

  // TODO
  return false;
}
/* -------------------------------------------------------------------------- */