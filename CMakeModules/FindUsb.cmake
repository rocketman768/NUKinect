# http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
#
# Defines:
#   USB_FOUND - has library
#   USB_INCLUDE_DIRS - include dirs
#   USB_LIBRARIES - library linker flags

INCLUDE( LibFindMacros )

# Dependencies
# No dependencies for USB.
# LIBFIND_PACKAGE()

# Use pkg-config for hints.
LIBFIND_PKG_CHECK_MODULES( USB_PKGCONF "libusb" )

# Include dir
FIND_PATH( USB_INCLUDE_DIR
  NAMES "libusb.h"
  PATHS ${USB_PKGCONF_INCLUDE_DIRS}
)

# Library itself
FIND_LIBRARY( USB_LIBRARY
  NAMES "usb"
  PATHS ${USB_PKGCONF_LIBRARY_DIRS}
)

# Set variables so that LIBFIND_PROCESS() will set the final
# output variables.
SET( USB_PROCESS_INCLUDES USB_INCLUDE_DIR )
SET( USB_PROCESS_LIBS USB_LIBRARY )
LIBFIND_PROCESS(USB)