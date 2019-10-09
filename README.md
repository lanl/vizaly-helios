# vizaly-hélios

[![Build Status](https://travis-ci.com/lanl/VizAly-Helios.svg?branch=master)](https://travis-ci.com/lanl/VizAly-Helios)

### Summary

###### Purpose

**hélios** is a research code for large cosmological dataset compression.  
It relies on particle clouds categorization or filtering, and includes three tools:

- compress: inflate or deflate distributed particle datasets, forked from [cbench](https://github.com/lanl/VizAly-Foresight).
- combine: merge distributed particle datasets.
- analysis: compute entropy, filter and extract non-halos and generate plot scripts.


###### Usage 

**hélios** can be built on Linux or macOS using CMake.  
It requires a C++17 compiler and MPI as well.  
It relies on a set of compression kernels with can be built from [here](https://github.com/hobywan/compressors).  
They require [gsl](https://www.gnu.org/software/gsl/), [zlib](https://zlib.net) and [fftw](http://www.fftw.org) libraries as well.  
Related installation paths should then be saved within environment variables like `FOO_INSTALL_DIR`.  
**hélios** can finally built as follow:

```bash
mkdir build && cd build                    # avoid in-source builds
cmake -DENABLE_FOO=ON ..                   # enable all kernels you want
make -j                                    # use multiple compilation jobs 
```

Tools parameters are supplied through a JSON file.    
They can be run like below: 

```mpirun -np [nranks] ./foo input.json```

###### Project scope

**hélios** is part of the **vizaly** project.  

### License

This is an open source software available under the BSD-3 license.  
(c) 2019, Los Alamos National Laboratory. All rights reserved.  
See the license file for more details.  

