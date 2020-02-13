#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name spectrum

timestep=499
hacc="/projects/exasky/hacc"
power_spectrum="${hacc}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
particles="/projects/groups/vizproject/dssdata/Exasky/helios/analysis/data-full-noised-${timestep}"
output="/projects/groups/vizproject/dssdata/Exasky/helios/analysis/pk-noised-${timestep}.dat"

# shellcheck disable=SC1090
source "${hacc}.darwin_setup" && cd "${hacc}/run" &&
mpirun ${power_spectrum} inputs/indat.params -n ${particles} ${output} ${timestep}
