#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name spectrum

HACC="/home/hoby/dev/exasky/hacc"
POWER_SPECTRUM="${HACC}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
PARTICLES_DATA="/projects/groups/vizproject/dssdata/cosmo/Argonne_L360_HACC001/STEP499/m000.full.mpicosmo.499"
OUTPUT_DATA="/projects/groups/vizproject/dssdata/Exasky/helios/analysis/pk-full.dat"
TIMESTEP=499

# shellcheck disable=SC1090
source "${HACC}.darwin_setup" && cd "${HACC}/run" &&
mpirun ${POWER_SPECTRUM} inputs/indat.params -n ${PARTICLES_DATA} ${OUTPUT_DATA} ${TIMESTEP}