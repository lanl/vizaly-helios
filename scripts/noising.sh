#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name noising

NRANKS=8

BUILD="/projects/exasky/hoby-projects/helios/build"
INPUT_JSON="../inputs/noising.json"

source "/home/hoby/.bashrc" && cd ${BUILD} &&
mpirun -np ${NRANKS} ./noising ${INPUT_JSON}

