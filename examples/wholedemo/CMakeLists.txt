SET( viewdemo_SRC
  "wholedemo.cpp"
)

ADD_EXECUTABLE( wholedemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( wholedemo NUKinect )
