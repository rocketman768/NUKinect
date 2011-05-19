# http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
#
# Defines:
#   CMINPACK_FOUND - system has libpcl
#   CMINPACK_INCLUDE_DIRS - include dirs for pcl
#   CMINPACK_LIBRARIES - library linker flags

INCLUDE( LibFindMacros )

# Dependencies
# No dependencies for CMINPACK.
# LIBFIND_PACKAGE()

# Use pkg-config for hints.
LIBFIND_PKG_CHECK_MODULES( CMINPACK_PKGCONF "cminpack" )

# Include dir
FIND_PATH( CMINPACK_INCLUDE_DIR
  NAMES "cminpack.h"
  PATHS ${CMINPACK_PKGCONF_INCLUDE_DIRS}
)

# Library itself
FIND_LIBRARY( CMINPACK_LIBRARY
  NAMES "cminpack"
  PATHS ${CMINPACK_PKGCONF_LIBRARY_DIRS}
)

# Set variables so that LIBFIND_PROCESS() will set the final
# output variables.
SET( CMINPACK_PROCESS_INCLUDES CMINPACK_INCLUDE_DIR )
SET( CMINPACK_PROCESS_LIBS CMINPACK_LIBRARY )
LIBFIND_PROCESS(CMINPACK)

# We require the standard math library.
SET( CMINPACK_LIBRARIES ${CMINPACK_LIBRARIES} "m" )