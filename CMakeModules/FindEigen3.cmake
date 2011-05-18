# http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
#
# Defines:
#   EIGEN3_FOUND - system has libpcl
#   EIGEN3_INCLUDE_DIRS - include dirs for pcl
#   EIGEN3_LIBRARIES - library linker flags

INCLUDE( LibFindMacros )

# Dependencies
# No dependencies for Eigen3.
# LIBFIND_PACKAGE()

# Use pkg-config for hints.
LIBFIND_PKG_CHECK_MODULES( EIGEN3_PKGCONF "eigen3" )

# Include dir
FIND_PATH( EIGEN3_INCLUDE_DIR
  NAMES "signature_of_eigen3_matrix_library"
  PATHS ${EIGEN3_PKGCONF_INCLUDE_DIRS}
)

# Library itself
# Eigen3 apparently has no library binary?
#FIND_LIBRARY( EIGEN3_LIBRARY
#  NAMES "eigen3"
#  PATHS ${EIGEN3_PKGCONF_LIBRARY_DIRS}
#)

# Set variables so that LIBFIND_PROCESS() will set the final
# output variables.
SET( EIGEN3_PROCESS_INCLUDES EIGEN3_INCLUDE_DIR )
#SET( EIGEN3_PROCESS_LIBS EIGEN3_LIBRARY )
SET( EIGEN3_LIBRARIES "" )
LIBFIND_PROCESS(EIGEN3)