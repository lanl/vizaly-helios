# - Find FFTW
# Find the native FFTW includes and library
#
#  FFTW_INCLUDES    - where to find fftw3.h
#  FFTW_LIBRARIES   - List of libraries when using FFTW.
#  FFTW_FOUND       - True if FFTW found.

if (FFTW_INCLUDES)
  # Already in cache, be silent
  set (FFTW_FIND_QUIETLY TRUE)
endif (FFTW_INCLUDES)

find_path(FFTW_INCLUDES fftw3.h
        HINTS "/projects/opt/centos7/fftw/3.3.4/include")

find_library(FFTW_LIB
        NAMES libfftw3.a
        PATHS "/projects/opt/centos7/fftw/3.3.4/lib")
find_library(FFTW_MPI
        NAMES libfftw3_mpi.a
        PATHS "/projects/opt/centos7/fftw/3.3.4/lib")

set(FFTW_LIBRARIES ${FFTW_LIB} ${FFTW_MPI})

# handle the QUIETLY and REQUIRED arguments and set FFTW_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args(FFTW DEFAULT_MSG FFTW_LIBRARIES FFTW_INCLUDES)

mark_as_advanced(FFTWF_LIBRARIES FFTW_LIBRARIES FFTW_INCLUDES)
