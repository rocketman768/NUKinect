# http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
#
# Defines:
#   FLANN_FOUND - system has libpcl
#   FLANN_INCLUDE_DIRS - include dirs for pcl
#   FLANN_LIBRARIES - library linker flags

INCLUDE( LibFindMacros )

# Dependencies
# No dependencies for FLANN.
# LIBFIND_PACKAGE()

# Use pkg-config for hints.
LIBFIND_PKG_CHECK_MODULES( FLANN_PKGCONF "flann" )

# Include dir
FIND_PATH( FLANN_INCLUDE_DIR
  NAMES "flann.h"
  PATHS ${FLANN_PKGCONF_INCLUDE_DIRS} "${CMAKE_INCLUDE_PATH}/flann"
)

# Library itself
FIND_LIBRARY( FLANN_LIBRARY
  NAMES "flann_cpp"
  PATHS ${FLANN_PKGCONF_LIBRARY_DIRS}
)

# Set variables so that LIBFIND_PROCESS() will set the final
# output variables.
SET( FLANN_PROCESS_INCLUDES FLANN_INCLUDE_DIR )
SET( FLANN_PROCESS_LIBS FLANN_LIBRARY )
LIBFIND_PROCESS(FLANN)