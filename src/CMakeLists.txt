SET( SRC_FILES
  "Kinect.cpp"
  "KinectIO.cpp"
  "nuKinectView.cpp"
  "nuKinectView4Buffer.cpp"
)

ADD_LIBRARY( NUKinect ${SRC_FILES} )
TARGET_LINK_LIBRARIES( NUKinect freenect ${CMAKE_THREAD_LIBS_INIT} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} )
