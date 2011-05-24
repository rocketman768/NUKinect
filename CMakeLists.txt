PROJECT( NUKinect )
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#==============Set Directories=================
SET( src_DIR "src/" )
SET( examples_DIR "examples/" )
SET( doc_DIR "doc/" )

INCLUDE_DIRECTORIES( ${src_DIR} )

#=================Options=======================
OPTION( BUILD_EXAMPLES
        "If on, will build example programs"
        ON )
OPTION( DEBUG_MODE
        "If on, will build everything with debug flags."
        OFF )

#==============Custom Cmake Modules=============
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules" )
#================Find Packages==================
# For pthread
FIND_PACKAGE( Threads REQUIRED )
INCLUDE_DIRECTORIES( Threads_INCLUDE_DIRS )
FIND_PACKAGE( GLUT REQUIRED )
INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIRS} )
FIND_PACKAGE( OpenGL REQUIRED )
INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIRS} )
FIND_PACKAGE( Qt4 4.5.0 REQUIRED COMPONENTS QtCore QtGui QtOpenGL)
INCLUDE( ${QT_USE_FILE} )
FIND_PACKAGE( OpenGL REQUIRED )
INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIRS} )
FIND_PACKAGE( PCL REQUIRED )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIRS} )

#===========Set release/debug settings==========
SET( CMAKE_CXX_FLAGS_DEBUG "-g -Wall" )
SET( LINK_FLAGS_DEBUG "-rdynamic" )
SET( CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall " )
SET( LINK_FLAGS_RELEASE "-rdynamic" )

IF( DEBUG_MODE )
  SET( CMAKE_BUILD_TYPE Debug )
ELSE()
  SET( CMAKE_BUILD_TYPE Release )
ENDIF()

#=============Process Subdirectories============
ADD_SUBDIRECTORY( ${src_DIR} )
ADD_SUBDIRECTORY( ${doc_DIR} )

IF( ${BUILD_EXAMPLES} )
  ADD_SUBDIRECTORY( ${examples_DIR} )
ENDIF()

