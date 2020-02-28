#!/bin/bash

#SBATCH --nodes 8
#SBATCH --ntasks-per-node 1
#SBATCH --partition skylake-gold
#SBATCH --job-name density

ranks=8
timestep=499
build="/projects/exasky/hoby-projects/helios/build"
input_json="../inputs/density_darwin_${timestep}.json"

source "/home/hoby/.bashrc" && cd ${build} &&
mpirun -np ${ranks} ./density ${input_json}