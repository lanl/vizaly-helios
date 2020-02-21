#!/bin/bash

timestep=${1}
suffix=${2}

# check timestep
if [[ ! "${timestep}" =~ ^[0-9]+$ ]]; then
  echo -e "invalid timestep"
  exit 1
fi

darwin_build="/projects/exasky/hoby-projects/helios/build"
darwin_data="/home/hoby/dev/exasky/data"
destination="/Users/hoby/dev/exasky/helios/plots/data/STEP_${timestep}"

if [ -d "${destination}" ]; then
  if [ -z "${suffix}" ]; then
    echo -e "empty suffix"
    exit 1
  fi
  destination="${destination}_${suffix}"
fi

mkdir ${destination}
scp "darwin:${darwin_build}/bits_distrib.dat" "${destination}"
scp "darwin:${darwin_build}/density_distrib-${timestep}.dat" "${destination}"
scp "darwin:${darwin_build}/particle_distrib-${timestep}.dat" "${destination}"
scp "darwin:${darwin_data}/pk-decomp-${timestep}.dat.pk" "${destination}"