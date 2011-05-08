PROJECT( NUKinect )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#==============Set Directories=================
SET( src_DIR "src/" )
SET( examples_DIR "examples/" )

INCLUDE_DIRECTORIES( ${src_DIR} )

#=================Options=======================
OPTION( BUILD_EXAMPLES
        "If on, will build example programs"
        ON )

#================Find Packages==================
# For pthread
FIND_PACKAGE( Threads REQUIRED )
INCLUDE_DIRECTORIES( Threads_INCLUDE_DIRS )
FIND_PACKAGE( GLUT REQUIRED )
INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIRS} )
FIND_PACKAGE( OpenGL REQUIRED )
INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIRS} )
#===========Set release/debug settings==========
SET( CMAKE_CXX_FLAGS_DEBUG "-g -Wall" )
SET( LINK_FLAGS_DEBUG "-rdynamic" )
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SRCROOT})
SET( CMAKE_CXX_FLAGS_RELEASE "-g -Wall " )
SET( LINK_FLAGS_RELEASE "-rdynamic" )

SET( CMAKE_BUILD_TYPE Release )

#=============Process Subdirectories============
ADD_SUBDIRECTORY( ${src_DIR} )

IF( ${BUILD_EXAMPLES} )
  ADD_SUBDIRECTORY( ${examples_DIR} )
ENDIF()

