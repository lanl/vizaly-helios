#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition skylake-gold
#SBATCH --job-name analysis

# enable or disable steps
EXTRACT_NON_HALOS=true
COMPRESS_NON_HALOS=false
MERGE_DATASETS=false
COMPUTE_POWER_SPECTRUM=false

# parameters
NRANKS=8
TIMESTEP=499
SUFFIX=""
HACC="/projects/exasky/HACC"
BUILD="/projects/exasky/hoby-projects/helios/build"
INPUT_JSON="../inputs/analysis_pipeline${SUFFIX}.json"
POWER_SPECTRUM="${HACC}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
PARTICLES_DATA="/home/hoby/dev/exasky/data/data-merged-zip${SUFFIX}.dat"
OUTPUT_DATA="/home/hoby/dev/exasky/data/pk-merged-zip${SUFFIX}.dat"

# extract non-halos and compute entropy if required
if ${EXTRACT_NON_HALOS}; then
  source "/home/hoby/.bashrc" && cd ${BUILD} &&
  mpirun -np ${NRANKS} ./analyzer ${INPUT_JSON} 
fi

# compress non-halo particles dataset
if ${COMPRESS_NON_HALOS}; then
  source "/home/hoby/.bashrc" && cd ${BUILD} &&
  mpirun -np ${NRANKS} ./CBench ${INPUT_JSON} 
fi

# combine it with halo ones
if ${MERGE_DATASETS}; then
  source "/home/hoby/.bashrc" && cd ${BUILD} &&
  mpirun -np ${NRANKS} ./merger ${INPUT_JSON}
fi

# compute power spectrum eventually
if ${COMPUTE_POWER_SPECTRUM}; then
  # shellcheck disable=SC1090
  source "${HACC}.darwin_setup" && cd "${HACC}/run" &&
  mpirun ${POWER_SPECTRUM} inputs/indat.params -n ${PARTICLES_DATA} ${OUTPUT_DATA} ${TIMESTEP}
fi
