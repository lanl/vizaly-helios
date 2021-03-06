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

find_path(BLOSC_INCLUDE_DIR blosc.h PATHS $ENV{BLOSC_INSTALL_DIR}/include)
find_library(BLOSC_LIBRARY libblosc.a PATHS $ENV{BLOSC_INSTALL_DIR}/lib)
find_package_handle_standard_args(blosc DEFAULT_MSG BLOSC_INCLUDE_DIR BLOSC_LIBRARY)

set(BLOSC_INCLUDE_DIRS ${BLOSC_INCLUDE_DIR})
set(BLOSC_LIBRARIES ${BLOSC_LIBRARY})
mark_as_advanced(BLOSC_INCUDE_DIR BLOSC_LIBRARY)
