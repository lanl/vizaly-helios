#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition skylake-gold
#SBATCH --job-name density

# shellcheck disable=SC2034
timestep=499
build="/projects/exasky/hoby-projects/helios/build"
hacc="/projects/exasky/HACC"
input_json="../inputs/density_darwin_${timestep}.json"
power_spectrum="${hacc}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
particles="/home/hoby/dev/exasky/data/decompressed-${timestep}.gio"
output="/home/hoby/dev/exasky/data/pk-decomp-${timestep}.dat"

compress() {
  # compute density histogram and compress particle bins
  source "/home/hoby/.bashrc" && cd ${build} &&
  mpirun ./density ${input_json}
}

assess_power_spectrum() {
  # shellcheck disable=SC2034
  source "${hacc}.darwin_setup" && cd "${hacc}/run" &&
  mpirun ${power_spectrum} inputs/indat.params -n ${particles} ${output} ${timestep}
}

# main
compress && assess_power_spectrum


