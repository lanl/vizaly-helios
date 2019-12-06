#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name spectrum

HACC="/projects/exasky/HACC"
POWER_SPECTRUM="${HACC}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
PARTICLES_DATA="/projects/exasky_data/hoby/analysis/data-full-noised"
OUTPUT_DATA="/projects/exasky_data/hoby/analysis/pk-noised.dat"
TIMESTEP=499

# shellcheck disable=SC1090
source "${HACC}.darwin_setup" && cd "${HACC}/run" &&
mpirun ${POWER_SPECTRUM} inputs/indat.params -n ${PARTICLES_DATA} ${OUTPUT_DATA} ${TIMESTEP}
