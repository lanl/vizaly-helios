#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name density

RANKS=8
BUILD="/projects/exasky/hoby-projects/helios/build"
INPUT_JSON="../inputs/density_darwin.json"

source "/home/hoby/.bashrc" && cd ${BUILD} &&
mpirun -np ${RANKS} ./density ${INPUT_JSON}