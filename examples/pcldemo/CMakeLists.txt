SET( viewdemo_SRC
  "pcldemo.cpp"
)

ADD_EXECUTABLE( viewdemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( viewdemo NUKinect )
