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

include(FindPackageHandleStandardArgs)

find_path(SZ_INCLUDE_DIR sz.h HINTS $ENV{SZ_INSTALL_DIR}/include)

# find library and dependencies
find_library(SZ_LIBRARY SZ HINTS $ENV{SZ_INSTALL_DIR}/lib)
find_library(ZLIB_LIBRARY zlib HINTS $ENV{SZ_INSTALL_DIR}/lib)
find_library(ZSTD_LIBRARY zstd HINTS $ENV{SZ_INSTALL_DIR}/lib)

find_package_handle_standard_args(sz DEFAULT_MSG SZ_INCLUDE_DIR SZ_LIBRARY)
find_package_handle_standard_args(zlib DEFAULT_MSG ZLIB_LIBRARY)
find_package_handle_standard_args(zstd DEFAULT_MSG ZSTD_LIBRARY)

set(SZ_INCLUDE_DIRS ${SZ_INCLUDE_DIR})
list(APPEND SZ_LIBRARIES ${SZ_LIBRARY} ${ZLIB_LIBRARY} ${ZSTD_LIBRARY})
mark_as_advanced(SZ_INCLUDE_DIR SZ_LIBRARY ZLIB_LIBRARY ZSTD_LIBRARY)
