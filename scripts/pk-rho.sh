#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition skylake-gold
#SBATCH --job-name spectrum

timestep=499
hacc="/projects/exasky/HACC"
power_spectrum="${hacc}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
particles="/home/hoby/dev/exasky/data/decompressed-${timestep}.gio"
output="/home/hoby/dev/exasky/data/pk-decomp-${timestep}.dat"

# shellcheck disable=SC1090
source "${hacc}.darwin_setup" && cd "${hacc}/run" &&
mpirun ${power_spectrum} inputs/indat.params -n ${particles} ${output} ${timestep}
