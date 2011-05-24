# http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
#
# Defines:
#   QHULL_FOUND - has library
#   QHULL_INCLUDE_DIRS - include dirs
#   QHULL_LIBRARIES - library linker flags

INCLUDE( LibFindMacros )

# Dependencies
# No dependencies for QHULL.
# LIBFIND_PACKAGE()

# Use pkg-config for hints.
#LIBFIND_PKG_CHECK_MODULES( QHULL_PKGCONF "qhull" )

# Include dir
FIND_PATH( QHULL_INCLUDE_DIR
  NAMES "qhull.h"
#  NAMES "qhull/qhull.h"
  PATHS ${QHULL_PKGCONF_INCLUDE_DIRS} "${CMAKE_INCLUDE_PATH}/qhull"
#  PATHS ${QHULL_PKGCONF_INCLUDE_DIRS}
)

# Library itself
FIND_LIBRARY( QHULL_LIBRARY
  NAMES "qhull"
  PATHS ${QHULL_PKGCONF_LIBRARY_DIRS}
)

# Set variables so that LIBFIND_PROCESS() will set the final
# output variables.
SET( QHULL_PROCESS_INCLUDES QHULL_INCLUDE_DIR )
SET( QHULL_PROCESS_LIBS QHULL_LIBRARY )
LIBFIND_PROCESS(QHULL)