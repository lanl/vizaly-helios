# VizAly-Helios

[![Build Status](https://travis-ci.com/lanl/VizAly-Helios.svg?branch=master)](https://travis-ci.com/lanl/VizAly-Helios)

### hélios

**hélios** is a research code for large cosmological dataset compression, based on particle clouds categorization and subsampling. It includes three tools:

- compression: inflate and deflate distributed particle datasets, forked from [CBench](https://github.com/lanl/VizAly-Foresight).
- combine: merge distributed particle datasets.
- analysis: compute particles entropy, filter and extract non-halos and generate plot scripts.

###### Usage

**hélios** can be built on Linux or macOS using [CMake](https://cmake.org).  
It requires a [C++17](https://en.cppreference.com/w/cpp/compiler_support#C.2B.2B17_library_features) compiler endowed with [OpenMP](https://www.openmp.org) and [MPI](https://en.wikipedia.org/wiki/Message_Passing_Interface) as well.  
It relies on a set of compression kernels with can be built from [here](https://github.com/hobywan/compressors).  
Once installed, related directories should be saved within environment variables:

```bash
export BLOSC_INSTALL_DIR="${compressors}/blosc/install"
export FPZIP_INSTALL_DIR="${compressors}/fpzip/install"
export ISABELA_INSTALL_DIR="${compressors}/isabela/install"
export SZ_INSTALL_DIR="${compressors}/sz/install"
export ZFP_INSTALL_DIR="${compressors}/zfp/install"
```
**hélios** can finally built as follow:

```bash
mkdir build                                # avoid in-source builds
cd build
cmake -DENABLE_SZ=ON -DENABLE_FPZIP=ON ..  # enable all kernels you want
make -j                                    # use multiple compilation jobs 
```

###### project scope

**hélios** is part of the **vizaly** project. The latter is a general framework for Analysis and Visualization of simulation data. As supercomputing resources increase, cosmological scientists are able to run more detailed and larger simulations generating massive amounts of data. Analyzing these simulations with an available open-source toolkit is important for collaborative Department of Energy scientific discovery across labs, universities, and other partners. Developed software as a part of this collection include: comparing data with other existing simulations, verifying and validating results with observation databases, new halo finder algorithms, and using analytical tools to get insights into the physics of the cosmological universe. The goal of this software project is to provide a set of open-source libraries, tools, and packages for large-scale cosmology that allows scientists to visualize, analyze, and compare large-scale simulation and observational data sets. Developed software will provide a variety of methods for processing, visualization, and analysis of astronomical observation and cosmological simulation data. These tools are intended for deployment on multiple scientific computing platforms, including but not limited to personal computers, cloud computing, experimental sites (telescopes) and high-performance supercomputers.
