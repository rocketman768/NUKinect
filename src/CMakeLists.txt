SET( SRC_FILES
  "Kinect.cpp"
  "KinectIO.cpp"
  "nuKinectView.cpp"
  "nuKinectView4Buffer.cpp"
  "MouseController.cpp"
  "HandTracker.cpp"
)

ADD_LIBRARY( NUKinect ${SRC_FILES} )
#I use some hard coded lib name here because CMake is not configured to use this lib
TARGET_LINK_LIBRARIES( NUKinect freenect ${CMAKE_THREAD_LIBS_INIT} ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} "-lopencv_core -lopencv_highgui")
