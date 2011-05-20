SET( viewdemo_SRC
  "pcldemo_neareast.cpp"
)

FIND_PACKAGE( "PCL" REQUIRED )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )

ADD_EXECUTABLE( pcldemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( pcldemo
  NUKinect
  ${PCL_LIBRARIES}
)
