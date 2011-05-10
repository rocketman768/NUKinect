SET( SRC_FILES
  "Kinect.cpp"
  "KinectIO.cpp"
  "nuKinectView.cpp"
  "nuKinectView4Buffer.cpp"
  "MouseController.cpp"
  "HandTracker.cpp"
  "KinectWindow.cpp"
  "GLWidget.cpp"
  "qtlogo.cpp"
  "DepthMapBinFileIO.cpp"
  "DepthMap.cpp"
)

  # another list, this time it includes all header files that should be treated with moc
  SET( SAMPLE_MOC_HDRS
                GLWidget.h
                KinectWindow.h
                qtlogo.h  
  )
  

SET(QT_USE_QTOPENGL TRUE)

FIND_PACKAGE( Qt4 REQUIRED )
FIND_PACKAGE( OpenGL REQUIRED )  
FIND_PACKAGE( OpenCV REQUIRED ) 

INCLUDE( ${QT_USE_FILE} )
INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIRS} 
     ${OpenCV_INCLUDE_DIR}
  ${QT_INCLUDE_DIR}/QtOpenGL
      )  

QT4_WRAP_CPP( SAMPLE_MOC_SRCS ${SAMPLE_MOC_HDRS} )



ADD_LIBRARY( NUKinect ${SRC_FILES} ${SAMPLE_MOC_SRCS})

TARGET_LINK_LIBRARIES( NUKinect 
  freenect 
  ${CMAKE_THREAD_LIBS_INIT} 
  ${GLUT_LIBRARIES} 
  ${QT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS} 
  )
