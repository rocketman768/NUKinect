# http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
#
# Defines:
#   OPENNI_FOUND - has library
#   OPENNI_INCLUDE_DIRS - include dirs
#   OPENNI_LIBRARIES - library linker flags

INCLUDE( LibFindMacros )

# Dependencies
LIBFIND_PACKAGE( OPENNI "Usb" )

SET( OPENNI_INCLUDE_DIRS ${OPENNI_INCLUDE_DIRS} ${USB_INCLUDE_DIRS} )
SET( OPENNI_LIBRARIES ${OPENNI_LIBRARIES} ${USB_LIBRARIES} )

# Use pkg-config for hints.
LIBFIND_PKG_CHECK_MODULES( OPENNI_PKGCONF "openni-dev" )

# Include dir
FIND_PATH( OPENNI_INCLUDE_DIR
  NAMES "XnVersion.h"
  PATHS ${OPENNI_PKGCONF_INCLUDE_DIRS}
)

# Library itself
FIND_LIBRARY( OPENNI_LIBRARY
  NAMES "OpenNI"
  PATHS ${OPENNI_PKGCONF_LIBRARY_DIRS}
)

# Set variables so that LIBFIND_PROCESS() will set the final
# output variables.
SET( OPENNI_PROCESS_INCLUDES OPENNI_INCLUDE_DIR )
SET( OPENNI_PROCESS_LIBS OPENNI_LIBRARY )
LIBFIND_PROCESS(OPENNI)