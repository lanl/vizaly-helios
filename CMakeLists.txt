#
# Copyright (c) 2019, Los Alamos National Laboratory
# All rights reserved.
#
# Author: Hoby Rakotoarivelo
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0048 NEW)

project(helios VERSION 1.0.0 LANGUAGES CXX)

# windows not supported
if (WIN32)
	message(FATAL_ERROR "Sorry, Windows is not supported.")
# prevent in source builds since they are evil
elseif (CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
	message(FATAL_ERROR "Sorry, in-source build is not allowed")
endif()

# set build-type dependent options
set(DEFAULT_BUILD_TYPE "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	message(STATUS "Setting build type to '${DEFAULT_BUILD_TYPE}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${DEFAULT_BUILD_TYPE}" CACHE STRING "Choose the type of build." FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

# manage dependencies
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# main executables
add_executable(compress)
add_executable(analysis)
add_executable(combine)
add_executable(noising)
add_executable(density)
add_executable(stats)

target_include_directories(compress PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(analysis PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(combine  PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(noising  PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(density  PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(stats    PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_sources(compress PRIVATE
		src/utils/tools.cpp
		src/utils/memory.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/compressors/kernels/blosc.cpp
		src/compressors/kernels/fpzip.cpp
		src/compressors/kernels/isabela.cpp
		src/compressors/kernels/sz.cpp
		src/compressors/kernels/zfp.cpp
		src/compressors/metrics/absolute_error.cpp
		src/compressors/metrics/relative_error.cpp
		src/compressors/metrics/mean_square_error.cpp
		src/compressors/metrics/psnr_error.cpp
		src/compressors/metrics/min_max.cpp
		src/compressors/run.cpp)

target_sources(analysis PRIVATE
		src/utils/tools.cpp
		src/utils/memory.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/analysis/analyzer.cpp
		src/analysis/run.cpp)

target_sources(combine PRIVATE
		src/utils/tools.cpp
		src/utils/memory.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/combine/merge.cpp
		src/combine/run.cpp)

target_sources(noising PRIVATE
		src/utils/tools.cpp
		src/utils/memory.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/noising/noising.cpp
		src/noising/run.cpp)

target_sources(density PRIVATE
		src/utils/tools.cpp
		src/utils/memory.cpp
		src/compressors/kernels/fpzip.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/compressors/kernels/blosc.cpp
		src/compressors/kernels/fpzip.cpp
		src/compressors/kernels/isabela.cpp
		src/compressors/kernels/sz.cpp
		src/compressors/kernels/zfp.cpp
		src/density/density.cpp
		src/density/run.cpp)

target_sources(stats PRIVATE
		src/utils/tools.cpp
		src/stats/run.cpp)

# hacc data io
add_library(gio STATIC)
target_compile_features(gio PUBLIC cxx_std_17)
target_sources(gio PRIVATE src/io/GenericIO.cpp)
target_include_directories(gio PRIVATE include/io)
target_compile_definitions(gio PUBLIC -DGENERICIO_NO_COMPRESSION)
set_target_properties(gio PROPERTIES POSITION_INDEPENDENT_CODE ON)

# MPI and OpenMP
find_package(MPI REQUIRED)
find_package(OpenMP REQUIRED)

# For supporting CMake < 3.9:
if(NOT TARGET MPI::MPI_CXX)
	add_library(MPI::MPI_CXX IMPORTED INTERFACE)
	set_target_properties(MPI::MPI_CXX PROPERTIES INTERFACE_COMPILE_OPTIONS "${MPI_COMPILE_FLAGS}")
	set_target_properties(MPI::MPI_CXX PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${MPI_INCLUDE_PATH}")
	set_target_properties(MPI::MPI_CXX PROPERTIES INTERFACE_LINK_LIBRARIES "${MPI_LINK_FLAGS}")
endif()

# propagated to all target linked to gio
target_link_libraries(gio PUBLIC MPI::MPI_CXX)
target_link_libraries(gio PUBLIC OpenMP::OpenMP_CXX)
target_link_libraries(compress PRIVATE gio)
target_link_libraries(analysis PRIVATE gio)
target_link_libraries(combine PRIVATE gio)
target_link_libraries(noising PRIVATE gio)
target_link_libraries(density PRIVATE gio)
target_link_libraries(stats PRIVATE gio)

# link to fftw
find_package(FFTW)
if (FFTW_FOUND)
	target_include_directories(noising PRIVATE ${FFTW_INCLUDES})
	target_link_libraries(noising PRIVATE ${FFTW_LIBRARIES})
	target_compile_definitions(noising PRIVATE -DHAVE_FFTW=1)
endif()

# enable/disable compressors and debug options
option(ENABLE_BLOSC    "Enable BLOSC"   ON)
option(ENABLE_FPZIP    "Enable FPZIP"   ON)
option(ENABLE_ISABELA  "Enable ISABELA" OFF)
option(ENABLE_SZ       "Enable SZ"      ON)
option(ENABLE_ZFP      "Enable ZFP"     OFF)
option(DEBUG_DENSITY   "Debug density"  OFF)
option(ENABLE_LOSSLESS "Use lossy+lossless" OFF)

# link to external compressors
foreach(binary compress density)
	if (ENABLE_BLOSC)
		find_package(BLOSC REQUIRED)
		target_compile_definitions(${binary} PRIVATE -DENABLE_BLOSC=1)
		target_include_directories(${binary} PRIVATE ${BLOSC_INCLUDE_DIRS})
		target_link_libraries(${binary} PRIVATE ${BLOSC_LIBRARIES})
	endif()

	if (ENABLE_FPZIP)
		find_package(FPZIP REQUIRED)
		target_compile_definitions(${binary} PRIVATE -DENABLE_FPZIP=1)
		target_include_directories(${binary} PRIVATE ${FPZIP_INCLUDE_DIRS})
		target_link_libraries(${binary} PRIVATE ${FPZIP_LIBRARIES})
	endif()

	if (ENABLE_ISABELA)
		find_package(Isabela REQUIRED)
		target_compile_definitions(${binary} PRIVATE -DENABLE_ISABELA=1)
		target_include_directories(${binary} PRIVATE ${ISABELA_INCLUDE_DIRS})
		target_link_libraries(${binary} PRIVATE ${ISABELA_LIBRARIES})
	endif()

	if (ENABLE_SZ)
		find_package(SZ REQUIRED)
		target_compile_definitions(${binary} PRIVATE -DENABLE_SZ=1)
		target_include_directories(${binary} PRIVATE ${SZ_INCLUDE_DIRS})
		target_link_libraries(${binary} PRIVATE ${SZ_LIBRARIES})
	endif()

	if (ENABLE_ZFP)
		find_package(ZFP REQUIRED)
		target_compile_definitions(${binary} PRIVATE -DENABLE_ZFP=1)
		target_include_directories(${binary} PRIVATE ${ZFP_INCLUDE_DIRS})
		target_link_libraries(${binary} PRIVATE ${ZFP_LIBRARIES})
	endif()
endforeach()

# define macros if necessary
if (DEBUG_DENSITY)
	target_compile_definitions(density PRIVATE -DDEBUG_DENSITY=1)
endif()
if (ENABLE_LOSSLESS)
	target_compile_definitions(density PRIVATE -DENABLE_LOSSLESS=1)
endif()

# install instructions
install(TARGETS stats analysis compress combine density gio DESTINATION .)

