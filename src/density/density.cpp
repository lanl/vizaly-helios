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
    // retrieve the total number of elems
    MPI_Allreduce(&local_count, &total_count, 1, MPI_LONG, MPI_SUM, comm);

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
bool Density::loadFiles() {

  debug_log << "Loading density values ... " << std::flush;

  long offset = 0;
  long count = 0;
  std::string path;

  for (auto&& current : inputs) {
    std::tie(path, count) = current;

    std::ifstream file(path, std::ios::binary);
    if (not file.good())
      return false;

    auto buffer = reinterpret_cast<char *>(density.data() + offset);
    auto size = count * sizeof(float);

    file.seekg(0, std::ios::beg);
    file.read(buffer, size);
    file.close();

    // update offset
    offset += count;
  }

  debug_log << density.size() << " values loaded." << std::endl;
  return not density.empty();
}


/* -------------------------------------------------------------------------- */
bool Density::computeFrequencies() {

  debug_log << "Computing frequencies ... " << std::endl;

  if (not local_count or not total_count)
    return false;

  // determine data values extents
  total_max = 0;
  total_min = 0;
  double local_min = *std::min_element(density.data(), density.data()+local_count);
  double local_max = *std::max_element(density.data(), density.data()+local_count);
  MPI_Allreduce(&local_max, &total_max, 1, MPI_DOUBLE, MPI_MAX, comm);
  MPI_Allreduce(&local_min, &total_min, 1, MPI_DOUBLE, MPI_MIN, comm);

  debug_log << "= local_extents: [" << local_min << ", " << local_max << "]"<< std::endl;
  debug_log << "= total_extents: [" << total_min << ", " << total_max << "]"<< std::endl;
  MPI_Barrier(comm);

  if (total_max <= 0.0)
    return false;

  // compute histogram of values
  debug_log << "nb_bins: " << nb_bins << std::endl;

  long local_histo[nb_bins];
  long total_histo[nb_bins];

  std::fill(local_histo, local_histo + nb_bins, 0);
  std::fill(total_histo, total_histo + nb_bins, 0);

  double const range = total_max - total_min;
  double const capacity = range / nb_bins;

  for (auto k = 0; k < local_count; ++k) {
    double relative_value = (density[k] - total_min) / range;
    int index = static_cast<int>((range * relative_value) / capacity);

    if (index >= nb_bins)
      index--;

    local_histo[index]++;
  }

  MPI_Allreduce(local_histo, total_histo, nb_bins, MPI_LONG, MPI_SUM, comm);

  // fill frequency eventually
  frequency.clear();
  frequency.resize(nb_bins);

  // normalize and store data
  double total_values = 0.;
  for (int i = 0; i < nb_bins; ++i)
    total_values += static_cast<double>(total_histo[i]);

  for (int i = 0; i < nb_bins; ++i)
    frequency[i] = 100. * static_cast<double>(total_histo[i]) / total_values;

  if (my_rank == 0)
    dumpHistogram();

  MPI_Barrier(comm);
  debug_log << "done" << std::endl;

  return true;
}

/* -------------------------------------------------------------------------- */
void Density::dumpHistogram() {

  double const width = (total_max - total_min) / static_cast<double>(nb_bins);

  std::ofstream file(output_plot + ".dat", std::ios::trunc);
  assert(file.is_open());
  assert(file.good());

  file << "# density field histogram" << std::endl;
  file << "# nb_bins: " << std::to_string(nb_bins) << std::endl;

  int k = 1;
  for (auto&& value : frequency) {
    file << total_min + (k * width) << "\t"<< value << std::endl;
    k++;
  }

  file.close();
}

/* -------------------------------------------------------------------------- */
void Density::run() {

  debug_log.clear();
  debug_log.str("");

  // step 1: load current rank dataset in memory
  loadFiles();

  // step 2: compute frequencies and histogram
  computeFrequencies();

  if (my_rank == 0)
    dumpLogs();
}

/* -------------------------------------------------------------------------- */

void Density::dumpLogs() {

  std::ofstream logfile(output_log, std::ios::out);
  logfile << debug_log.str();
  logfile.close();
  std::cout << "Logs generated in "<< output_log << std::endl;

  debug_log.clear();
  debug_log.str("");
  MPI_Barrier(comm);
}
/* -------------------------------------------------------------------------- */