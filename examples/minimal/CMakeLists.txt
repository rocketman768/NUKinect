SET( minimal_SRC
  "minimal.cpp"
)

ADD_EXECUTABLE( minimal ${minimal_SRC} )
TARGET_LINK_LIBRARIES( minimal NUKinect )
