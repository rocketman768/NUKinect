SET( viewdemo_SRC
  "IOdemo.cpp"
)

ADD_EXECUTABLE( IOdemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( IOdemo NUKinect )
