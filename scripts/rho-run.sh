#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition skylake-gold
#SBATCH --job-name density

# shellcheck disable=SC2034
# shellcheck disable=SC1090

timestep=499
build="/projects/exasky/hoby-projects/helios/build"
hacc="/projects/exasky/HACC"
input_json="../inputs/density_darwin_${timestep}.json"
power_spectrum="${hacc}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
particles="/home/hoby/dev/exasky/data/decompressed-${timestep}.gio"
pk_decompress="/home/hoby/dev/exasky/data/pk-decomp-${timestep}.dat"
pk_reference="/home/hoby/dev/exasky/data/pk-orig-${timestep}.dat"

compress() {
  source "/home/hoby/.bashrc" && cd ${build} &&
  mpirun ./density ${input_json}
}

assess_power_spectrum() {
  source "${hacc}.darwin_setup" && cd "${hacc}/run" &&
  mpirun ${power_spectrum} inputs/indat.params -n ${particles} ${pk_decompress} ${timestep} &&
  mv "${pk_decompress}.pk" "${pk_decompress}"
}

assess_discrepancy() {
  source "/home/hoby/.bashrc" && cd "${build}" &&
  mpirun ./stats "${pk_reference}" "${pk_decompress}"
}

# main
compress && assess_power_spectrum && assess_discrepancy