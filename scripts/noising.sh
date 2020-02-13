#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition scaling
#SBATCH --job-name noising

ranks=8
build="/projects/exasky/hoby-projects/helios/build"
input_json="../inputs/noising.json"

source "/home/hoby/.bashrc" && cd ${build} &&
mpirun -np ${ranks} ./noising ${input_json}

