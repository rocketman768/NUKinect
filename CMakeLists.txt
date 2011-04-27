PROJECT( NUKinect )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#==============Set Directories=================
SET( src_DIR "src/" )

INCLUDE_DIRECTORIES( ${src_DIR} )

#================Find Packages==================
# For pthread
FIND_PACKAGE( Threads REQUIRED )
INCLUDE_DIRECTORIES( Threads_INCLUDE_DIRS )

#===========Set release/debug settings==========
SET( CMAKE_CXX_FLAGS_DEBUG "-g -Wall" )
SET( LINK_FLAGS_DEBUG "-rdynamic" )

SET( CMAKE_CXX_FLAGS_RELEASE "-g -Wall -O2" )
SET( LINK_FLAGS_RELEASE "-rdynamic" )

SET( CMAKE_BUILD_TYPE Release )

#=============Process Subdirectories============
ADD_SUBDIRECTORY( ${src_DIR} )
