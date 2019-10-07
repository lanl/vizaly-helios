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
# 3. Neither the name of mosquitto nor the names of its
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
add_executable(cbench)
add_executable(merger)
add_executable(helios)

target_include_directories(cbench PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(merger PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(helios PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_sources(cbench PRIVATE
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

target_sources(helios PRIVATE
		src/utils/memory.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/analysis/analyzer.cpp
		src/analysis/run.cpp)

target_sources(merger PRIVATE
		src/utils/memory.cpp
		src/io/data.cpp
		src/io/hacc.cpp
		src/merge/merge.cpp
		src/merge/run.cpp)

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
target_link_libraries(cbench PRIVATE gio)
target_link_libraries(helios PRIVATE gio)
target_link_libraries(merger PRIVATE gio)

# enable/disable compressors
option(ENABLE_BLOSC   "Enable BLOSC"   OFF)
option(ENABLE_FPZIP   "Enable FPZIP"   ON)
option(ENABLE_ISABELA "Enable ISABELA" OFF)
option(ENABLE_SZ      "Enable SZ"      ON)
option(ENABLE_ZFP     "Enable ZFP"     OFF)

# link to external compressors
if (ENABLE_BLOSC)
	find_package(blosc REQUIRED)
	target_compile_definitions(cbench PRIVATE -DENABLE_BLOSC=1)
	target_include_directories(cbench PRIVATE ${BLOSC_INCLUDE_DIRS})
	target_link_libraries(cbench PRIVATE ${BLOSC_LIBRARIES})
endif()

if (ENABLE_FPZIP)
	find_package(fpzip REQUIRED)
	target_compile_definitions(cbench PRIVATE -DENABLE_FPZIP=1)
	target_include_directories(cbench PRIVATE ${FPZIP_INCLUDE_DIRS})
	target_link_libraries(cbench PRIVATE ${FPZIP_LIBRARIES})
endif()

if (ENABLE_ISABELA)
	find_package(isabela REQUIRED)
	target_compile_definitions(cbench PRIVATE -DENABLE_ISABELA=1)
	target_include_directories(cbench PRIVATE ${ISABELA_INCLUDE_DIRS})
	target_link_libraries(cbench PRIVATE ${ISABELA_LIBRARIES})
endif()

if (ENABLE_SZ)
	find_package(sz REQUIRED)
	target_compile_definitions(cbench PRIVATE -DENABLE_SZ=1)
	target_include_directories(cbench PRIVATE ${SZ_INCLUDE_DIRS})
	target_link_libraries(cbench PRIVATE ${SZ_LIBRARIES})
endif()

if (ENABLE_ZFP)
	find_package(zfp REQUIRED)
	target_compile_definitions(cbench PRIVATE -DENABLE_ZFP=1)
	target_include_directories(cbench PRIVATE ${ZFP_INCLUDE_DIRS})
	target_link_libraries(cbench PRIVATE ${ZFP_LIBRARIES})
endif()

# install instructions
install(TARGETS helios cbench merger gio DESTINATION .)

