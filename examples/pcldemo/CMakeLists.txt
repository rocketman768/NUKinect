SET( viewdemo_SRC
  "pcldemo.cpp"
)

ADD_EXECUTABLE( pcldemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( pcldemo
  NUKinect
  ${PCL_LIBRARIES}
)
