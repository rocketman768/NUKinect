SET( viewdemo_SRC
  "wholedemo.cpp"
)

#QT4_WRAP_CPP( SAMPLE_MOC_SRCS ${SAMPLE_MOC_HDRS} )

ADD_EXECUTABLE( wholedemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( wholedemo 
  NUKinect  
  ${PCL_LIBRARIES}
 # ${QT_LIBRARIES}
 # ${OPENGL_LIBRARIES}
 # ${OpenCV_LIBS}  
)
