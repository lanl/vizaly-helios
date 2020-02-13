#!/bin/bash

#SBATCH --nodes 16
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name density

timestep=272
hacc="/home/hoby/dev/exasky/hacc"
power_spectrum="${hacc}/trunk/Darwin/mpi/bin/hacc_pk_gio_auto"
particles="/projects/groups/vizproject/dssdata/cosmo/Argonne_L360_hacc001/STEP${timestep}/m000.full.mpicosmo.${timestep}"
output="/projects/groups/vizproject/dssdata/Exasky/helios/analysis/pk-full_${timestep}.dat"

# shellcheck disable=SC1090
source "${hacc}/HACC.darwin_setup" && cd "${hacc}/run" &&
mpirun ${power_spectrum} inputs/indat.params -n ${particles} ${output} ${timestep}
