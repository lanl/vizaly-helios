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

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include "mpi.h"
#include "utils/tools.h"

// MPI stuffs
static int rank = 0;
static int nb_ranks = 0;
static MPI_Comm comm = MPI_COMM_WORLD;

// limit on wavenumber
static double const k_limit = 10.;

/**
 *
 */
class Data {
public:
  std::vector<double> k;
  std::vector<double> pk;
  double n_mode = 0.;
  double error = 0.;
  double pk_two = 0.;

  explicit Data(int capacity) {
    k.resize(capacity);
    pk.resize(capacity);
  }

  [[nodiscard]] int size() const { return k.size(); }
};

/**
 *
 */
void print_usage() {
  if (rank == 0) {
    std::cout << "Usage: ./stats pk-ref.dat pk-zip.dat" << std::endl;
    std::cout << "pk-ref.dat: power spectrum of reference HACC data." << std::endl;
    std::cout << "pk-zip.dat: power spectrum of modified HACC data." << std::endl;
  }
}

/**
 *
 * @param path
 * @return
 */
Data parse(std::string const& path) {

  std::ifstream file(path);
  assert(file.good());
  std::string line;

  // skip comments and header first
  while (std::getline(file, line)) {
    tools::ltrim(line);
    if (line[0] != '#' and line[0] != '%')
      break;
  }

  int i = 0;
  double k = 0.;
  Data data(900);

  while (file >> data.k[i] >> data.pk[i] >> data.error >> data.n_mode >> data.pk_two) {
    k = std::max(k, data.k[i++]);
    if (k > k_limit) {
      data.k.resize(i);
      data.pk.resize(i);
      break;
    }
  }

  file.close();
  return data;
}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char* argv[]) {

  // because other binaries will use MPI
  MPI_Init(&argc, &argv);
  MPI_Comm_size(comm, &nb_ranks);
  MPI_Comm_rank(comm, &rank);

  if (argc != 3) {
    print_usage();
    MPI_Finalize();
    return EXIT_FAILURE;
  }


  if (rank == 0) {

    std::cout << "Compute Pk discrepancy ..." << std::flush;
    // parse both files
    auto reference = parse(argv[1]);
    auto modified  = parse(argv[2]);

    assert(reference.size() == modified.size());
    int const nb_lines = reference.size();

    double discrepancy = 0.;
    for (int i = 0; i < nb_lines; ++i)
      discrepancy = std::max(modified.pk[i] / reference.pk[i], discrepancy);

    std::cout << " done -> ";
    std::cout.precision(3);
    std::cout << std::fixed;
    std::cout << discrepancy << std::endl;
  }

  MPI_Barrier(comm);
  MPI_Finalize();
  return EXIT_SUCCESS;
}