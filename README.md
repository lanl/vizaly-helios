# VizAly-Helios

[![Build Status](https://travis-ci.com/lanl/VizAly-Helios.svg?branch=master)](https://travis-ci.com/lanl/VizAly-Helios)

### hélios

**hélios** is a research code for large cosmological dataset compression.  
It relies on particle clouds categorization or filtering, and includes three tools:

- compress: inflate or deflate distributed particle datasets, forked from [CBench](https://github.com/lanl/VizAly-Foresight).
- combine: merge distributed particle datasets.
- analysis: compute entropy, filter and extract non-halos and generate plot scripts.

<!-- ###### Usage -->

**hélios** can be built on Linux or macOS using CMake.  
It requires a C++17 compiler endowed with OpenMP and MPI.  
It requires [GSL](https://www.gnu.org/software/gsl/), [zlib](https://zlib.net) and [fftw](http://www.fftw.org) libraries as well.  
It relies on a set of compression kernels with can be built from [here](https://github.com/hobywan/compressors).  
Related directories should then be saved within environment variables:

```bash
export BLOSC_INSTALL_DIR="path/to/blosc/install"
export FPZIP_INSTALL_DIR="path/to/fpzip/install"
export ISABELA_INSTALL_DIR="path/to/isabela/install"
export SZ_INSTALL_DIR="path/to/sz/install"
export ZFP_INSTALL_DIR="path/to/zfp/install"
```
**hélios** can finally built as follow:

```bash
mkdir build && cd build                    # avoid in-source builds
cmake -DENABLE_SZ=ON -DENABLE_FPZIP=ON ..  # enable all kernels you want
make -j                                    # use multiple compilation jobs 
```


### License

**hélios** is part of the **lanl/vizaly** project.  
It is an open source software available under the BSD-3 license.  
See the [license](LICENSE) file for more details.  
(c) 2019, Los Alamos National Laboratory. All rights reserved.


