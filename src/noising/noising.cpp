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

#include "noising/noising.h"

/* -------------------------------------------------------------------------- */
Noising::Noising(const char* in_path, int in_rank, int in_nb_ranks, MPI_Comm in_comm)
  : json_path(in_path),
    my_rank(in_rank),
    nb_ranks(in_nb_ranks),
    comm(in_comm)
{
  assert(nb_ranks > 0);
  nlohmann::json json;
  std::string buffer;

  std::ifstream file(json_path);
  assert(file.is_open());
  assert(file.good());

  // parse params and do basic checks
  file >> json;

  assert(json["noising"].count("type"));
  assert(json["noising"].count("input"));
  assert(json["noising"].count("output"));
  assert(json["noising"].count("scalars"));
  assert(json["noising"].count("d_min"));
  assert(json["noising"].count("d_max"));
  assert(json["noising"].count("deviat"));
  assert(json["noising"].count("logs"));
  assert(json["noising"].count("plots"));

  std::string type = json["noising"]["type"];
  assert(type == "gaussian");

  input = json["noising"]["input"];
  output = json["noising"]["output"];
  output_log = json["noising"]["logs"];
  output_plot = json["noising"]["plots"];
  dist_min = json["noising"]["d_min"];
  dist_max = json["noising"]["d_max"];
  deviation = json["noising"]["deviat"];
  assert(dist_min < dist_max);
  assert(deviation < std::min(std::abs(dist_min), std::abs(dist_max)));

  for (auto&& name : json["noising"]["scalars"])
    scalars.push_back(name);

  assert(not scalars.empty());
  num_scalars = scalars.size();
  dataset.resize(num_scalars);
  histo.resize(num_scalars);

  // set the IO manager
  ioMgr = std::make_unique<HACCDataLoader>();
}

/* -------------------------------------------------------------------------- */
size_t Noising::cache(long offset) {

  debug_log << "Caching dataset ... ";

  // set 'physOrigin' and 'physScale'
  // and update MPI cart partition while loading file.
  ioMgr->saveParams();
  ioMgr->setSave(true);

  for (int i=0; i < num_scalars; ++i) {
    if (ioMgr->load(scalars[i])) {
      auto const n = ioMgr->getNumElements();
      auto const data = static_cast<float*>(ioMgr->data);
      dataset[i].resize(n + offset);
      std::copy(data, data + n, dataset[i].data() + offset);
    }
    MPI_Barrier(comm);
  }

  if (ioMgr->load("id")) {
    auto const n = ioMgr->getNumElements();
    auto const data = static_cast<long*>(ioMgr->data);
    particles_index.resize(n + offset);
    std::copy(data, data + n, particles_index.data() + offset);
  }

  if (my_rank == 0) {
    std::cout << "mpiCartPartitions: " << ioMgr->mpi_partition[0] << ", "
              << ioMgr->mpi_partition[1] << ", "
              << ioMgr->mpi_partition[2] << std::endl;
    std::cout << "physOrig: " << ioMgr->phys_orig[0] << ", "
              << ioMgr->phys_orig[1] << ", "
              << ioMgr->phys_orig[2] << std::endl;
    std::cout << "physScale: " << ioMgr->phys_scale[0] << ", "
              << ioMgr->phys_scale[1] << ", "
              << ioMgr->phys_scale[2] << std::endl;
  }

  debug_log << " done." << std::endl;
  MPI_Barrier(comm);

  size_t const copied_count = particles_index.size() - offset;
  return copied_count;
};


/* -------------------------------------------------------------------------- */
void Noising::dump() {

  debug_log << "Dumping dataset ... ";

  assert(local_count > 0);

  int periods[3] = {0,0,0};
  auto dim_size = ioMgr->mpi_partition;
  MPI_Cart_create(comm, 3, dim_size, periods, 0, &comm);

  // init writer and open file
  gio::GenericIO gioWriter(comm, output);
  gioWriter.setNumElems(local_count);

  // init physical params
  for (int d=0; d < 3; ++d) {
    gioWriter.setPhysOrigin(ioMgr->phys_orig[d], d);
    gioWriter.setPhysScale(ioMgr->phys_scale[d], d);
  }

  MPI_Barrier(comm);

  unsigned default_flag = gio::GenericIO::VarHasExtraSpace;

  // populate params now
  for (int i=0; i < num_scalars; ++i) {
    unsigned flag = default_flag;
    switch (i) {
      case 0: flag |= gio::GenericIO::VarIsPhysCoordX; break;
      case 1: flag |= gio::GenericIO::VarIsPhysCoordY; break;
      case 2: flag |= gio::GenericIO::VarIsPhysCoordZ; break;
      default: break;
    }
    gioWriter.addVariable(scalars[i].data(), dataset[i].data(), flag);
  }

  gioWriter.addVariable("id", particles_index.data(), default_flag);
  gioWriter.write();

  debug_log << " done." << std::endl;
  MPI_Barrier(comm);
}

/* -------------------------------------------------------------------------- */
std::vector<float> Noising::computeGaussianNoise(int field) {

  assert(field < num_scalars);

  debug_log << "Applying gaussian noise to '"<< scalars[field] <<"' ... ";

  // first gather the dataset size per rank.
  int nb_local = dataset[field].size();
  int nb_per_rank[nb_ranks];
  MPI_Gather(&nb_local, 1, MPI_INT, nb_per_rank, 1, MPI_INT, 0, comm);

  std::vector<float> noise(nb_local);

  // generate noise only on master rank
  if (my_rank == 0) {

    // generate a seed for random engine
    std::random_device device;

    // use Mersenne twister engine
    std::mt19937 engine { device() };

    // define a normal distribution generator
    auto const mean = static_cast<float>(0.5 * (dist_min + dist_max));

    std::normal_distribution<float> distrib(mean, deviation);

    for (auto& val : noise)
      val = distrib(engine);

    // generate number for current rank and send it
    for (int i = 1; i < nb_ranks; ++i) {
      // generate data
      std::vector<float> dist_noise(nb_per_rank[i]);
      for (int j = 0; j < nb_per_rank[i]; ++j) {
        dist_noise[j] = distrib(engine);
      }
      // send to correct rank
      MPI_Send(dist_noise.data(), nb_per_rank[i], MPI_FLOAT, i, 0, comm);
    }
  } else {
    MPI_Recv(noise.data(), nb_local, MPI_FLOAT, 0, 0, comm, MPI_STATUS_IGNORE);
  }

  debug_log << "done" << std::endl;
  return noise;
}

/* -------------------------------------------------------------------------- */
bool Noising::computeHistogram(int i, std::vector<float> const& noise) {

  int const nb_particles = noise.size();
  assert(nb_particles > 0);
  assert(num_bins > 0);
  assert(dist_max > dist_min);

  debug_log << "Computing histogram for '"<< scalars[i] <<"' ";
  debug_log << "using "<< num_bins << " bins ... ";

  // step 1. determine lower and upper bounds on data
  double local_max = std::numeric_limits<float>::min();
  double local_min = std::numeric_limits<float>::max();
  double total_max = 0;
  double total_min = 0;

  for (auto j=0; j < nb_particles; ++j) {
    if (noise[j] > local_max) { local_max = noise[j]; }
    if (noise[j] < local_min) { local_min = noise[j]; }
  }

  MPI_Allreduce(&local_max, &total_max, 1, MPI_DOUBLE, MPI_MAX, comm);
  MPI_Allreduce(&local_min, &total_min, 1, MPI_DOUBLE, MPI_MIN, comm);
  MPI_Barrier(comm);

  long local_histo[num_bins];
  long total_histo[num_bins];

  std::fill(local_histo, local_histo + num_bins, 0);
  std::fill(total_histo, total_histo + num_bins, 0);

  double range = total_max - total_min;
  double capacity = range / num_bins;

  for (auto k=0; k < nb_particles; ++k) {
    int index = static_cast<int>((noise[k] - total_min) / capacity);
    if (index >= num_bins)
      index--;

    local_histo[index]++;
  }

  MPI_Allreduce(local_histo, total_histo, num_bins, MPI_LONG, MPI_SUM, comm);

  // fill result array eventually
  histo[i].clear();
  histo[i].resize(num_bins);

  // normalize and store data
  long sum_values = 0;
  for (int j=0; j < num_bins; ++j)
    sum_values += total_histo[j];

  for (int j=0; j < num_bins; ++j)
    histo[i][j] = static_cast<float>(100. * double(total_histo[j]) / sum_values);

  if (my_rank == 0)
    dumpHistogram(i);

  MPI_Barrier(comm);
  debug_log << "done" << std::endl;
  return true;
}

/* -------------------------------------------------------------------------- */
void Noising::computeSpectralDensity(std::vector<float> const& noise) {
#if HAVE_FFTW
  int local_size = noise.size();
  int total_size = 0;
  MPI_Allreduce(&local_size, &total_size, 1, MPI_INT, MPI_SUM, comm);


  fftw_plan plan;
  fftw_complex *signal, *output;

  ptrdiff_t const n0 = total_size;
  ptrdiff_t local_alloc;
  ptrdiff_t local_ni, local_i_start;
  ptrdiff_t local_no, local_o_start;


  //fftw_complex data[local_size];
  // get local data size and allocate
  if (my_rank == 0)
    std::cout << "getting local data size ... ";
  local_alloc = fftw_mpi_local_size_1d(n0, comm,
                                       FFTW_FORWARD, FFTW_ESTIMATE,
                                       &local_ni, &local_i_start,
                                       &local_no, &local_o_start);
  if (my_rank == 0)
    std::cout << "done" << std::endl;

  if (my_rank == 0)
    std::cout << "allocating memory ... ";
  signal = fftw_alloc_complex(local_alloc);
  output = fftw_alloc_complex(local_alloc);
  MPI_Barrier(comm);

  if (my_rank == 0)
    std::cout << "done" << std::endl;

  // create plan and copy dataset
  if (my_rank == 0)
    std::cout << "creating plan and copying dataset ... ";

  plan = fftw_mpi_plan_dft_1d(n0, signal, output, comm, FFTW_FORWARD, FFTW_ESTIMATE);

  for (int i = 0; i < local_size; ++i) {
    signal[i][0] = noise[i];
    signal[i][1] = 0;
  }

  MPI_Barrier(comm);
  if (my_rank == 0)
    std::cout << "done" << std::endl;

  // compute discrete fourier transform
  if (my_rank == 0)
    std::cout << "compute discrete fourier transform ... ";
  fftw_execute(plan);

  MPI_Barrier(comm);
  if (my_rank == 0)
    std::cout << "done" << std::endl;

  // compute power spectral density
  double real = 0;
  double imag = 0;


  if (my_rank == 0)
    std::cout << "compute power spectral density ... ";

  int const nb_freq = std::ceil(local_size / 2);
  //int const nb_freq = local_size;
  std::vector<float> magnitude(nb_freq);

  for (int i = 0; i < nb_freq; ++i) {

    real = output[i][0] / total_size;
    imag = output[i][1] / total_size;
    magnitude[i] = (real * real + imag * imag) / 2.;

    if (my_rank == 0)
      std::cout << "real="<< real <<", imag="<< imag <<", magnitude["<< i<<"]=" << magnitude[i] << std::endl;
  }

  //magnitude[0] /= 2;
  MPI_Barrier(comm);
  if (my_rank == 0)
    std::cout << "done" << std::endl;

  // output result and finalize
  fftw_free(signal);
  fftw_free(output);
  fftw_destroy_plan(plan);

  // dump file
  int mpi_tag = 0;
  int size = 0;
  int count_per_rank[nb_ranks];
  int offsets[nb_ranks];
  std::vector<float> spectrum;

  MPI_Gather(&nb_freq, 1, MPI_INT, count_per_rank, 1, MPI_INT, 0, comm);

  if (my_rank == 0) {
    // compute data offsets and total size
    offsets[0] = 0;
    size = count_per_rank[0];
    for (int i = 1; i < nb_ranks; ++i) {
      offsets[i] = offsets[i - 1] + count_per_rank[i - 1];
      size += count_per_rank[i];
    }
    // resize global magnitude array accordingly
    spectrum.resize(size);
  }

  MPI_Gatherv(magnitude.data(), nb_freq, MPI_FLOAT, spectrum.data(), count_per_rank, offsets, MPI_FLOAT, 0, comm);

  if (my_rank == 0) {
    std::string path = "../../results/noising/power_spectrum.dat";
    std::ofstream file(path, std::ios::out|std::ios::trunc);
    assert(file.is_open());
    assert(file.good());

    for (int i = 0; i < size; ++i) {
      file << i << "\t" << spectrum[i] << std::endl;
    }

    file.close();
  }

#else
  std:cerr << "Warning: cannot compute noise spectral density without FFTW" << std::endl;
#endif
}


/* -------------------------------------------------------------------------- */
void Noising::run() {

  ioMgr->init(input, comm);
  local_count = cache();
  total_count = 0;
  MPI_Allreduce(&local_count, &total_count, 1, MPI_LONG, MPI_SUM, comm);

  debug_log << "Parameters: range: ["<< dist_min << ", "<< dist_max << "], ";
  debug_log << "deviation: " << deviation << ", ";
  debug_log << "count: "<< total_count << " particles."<< std::endl;
  MPI_Barrier(comm);

  for (int i = 0; i < num_scalars; ++i) {

    // a) compute and apply noise on current dataset
    int const nb_particles = dataset[i].size();
    auto const noise = computeGaussianNoise(i);
//    for (int j = 0; j < nb_particles; ++j) {
//      dataset[i][j] += noise[j];
//    }
//    MPI_Barrier(comm);
//
//    // b) compute histogram only for first scalar
//    if (i == 0) {
//      computeHistogram(i, noise);
//      MPI_Barrier(comm);
//    }

    // c) compute signal spectrum
    computeSpectralDensity(noise);
    MPI_Barrier(comm);
  }

  // now dump everything
  dump();

  if (my_rank == 0)
    dumpLogs();
}

/* -------------------------------------------------------------------------- */
void Noising::dumpHistogram(int field) {

  assert(field < num_scalars);

  auto const& scalar = scalars[field];
  std::string path = output_plot + "_" + scalar + ".dat";

  std::ofstream file(path, std::ios::out|std::ios::trunc);
  assert(file.is_open());
  assert(file.good());

  file << "# scalar: " << scalar << std::endl;
  file << "# num_bins: " << std::to_string(num_bins) << std::endl;

  auto const width = static_cast<float>((dist_max - dist_min) / num_bins);
  int k = 1;
  for (auto const& value : histo[field]) {
    file << dist_min + (k * width) << "\t"<< value << std::endl;
    k++;
  }

  file.close();
}

/* -------------------------------------------------------------------------- */
void Noising::dumpLogs() {

  std::ofstream logfile(output_log, std::ios::out);
  logfile << debug_log.str();
  logfile.close();
  std::cout << "Logs generated in "<< output_log << std::endl;

  if (my_rank == 0)
    std::cout << debug_log.str();

  debug_log.clear();
  debug_log.str("");
}
/* -------------------------------------------------------------------------- */