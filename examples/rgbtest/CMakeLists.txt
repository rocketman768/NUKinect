SET( src_files
  "rgbtest.cpp"
)

ADD_EXECUTABLE( rgbtest ${src_files} )
TARGET_LINK_LIBRARIES( rgbtest
  NUKinect
)

