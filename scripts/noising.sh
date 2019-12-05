#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name analysis

# toggle these variables
ADD_WHITE_NOISE=true
COMPUTE_POWER_SPECTRUM=false

NRANKS=8
TIMESTEP=499
SUFFIX=""

HACC="/projects/exasky/HACC"
BUILD="/projects/exasky/hoby-projects/helios/build"
INPUT_JSON="../inputs/analysis_pipeline${SUFFIX}.json"
POWER_SPECTRUM="${HACC}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
PARTICLES_DATA="/projects/exasky_data/hoby/analysis/data-full-noised${SUFFIX}"
OUTPUT_DATA="/projects/exasky_data/hoby/analysis/pk-noised${SUFFIX}.dat"

if ${ADD_WHITE_NOISE}; then
  source "/home/hoby/.bashrc" && cd ${BUILD} &&
  mpirun -np ${NRANKS} ./noising ${INPUT_JSON}
fi

if ${COMPUTE_POWER_SPECTRUM}; then
  # shellcheck disable=SC1090
  source "${HACC}.darwin_setup" && cd "${HACC}/run" &&
  mpirun ${POWER_SPECTRUM} inputs/indat.params -n ${PARTICLES_DATA} ${OUTPUT_DATA} ${TIMESTEP}
fi
