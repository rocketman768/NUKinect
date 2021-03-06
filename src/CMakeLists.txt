SET( SRC_FILES
  "Kinect.cpp"
  "KinectIO.cpp"
  "MouseController.cpp"
  "HandTracker.cpp"
  "KinectWindow.cpp"
  "GLWidget.cpp"
  "PtCloud.cpp"
  "DepthMapBinFileIO.cpp"
  "DepthMap.cpp"
  "FileKinectInputWidget.cpp"
  "LiveKinectInputWidget.cpp"
  "VerboseSlider.cpp"
)

# All header files that should be moc'd.
SET( MOC_HDRS
  "LiveKinectInputWidget.h"
  "FileKinectInputWidget.h"
  "KinectInputWidget.h"
  "VerboseSlider.h"
  "GLWidget.h"
  "KinectWindow.h"
)
  

SET(QT_USE_QTOPENGL TRUE)

#FIND_PACKAGE( Qt4 REQUIRED )
#FIND_PACKAGE( OpenGL REQUIRED )  
#FIND_PACKAGE( OpenCV REQUIRED ) 
#FIND_PACKAGE( "PCL" REQUIRED )

QT4_WRAP_CPP( MOC_SRCS ${MOC_HDRS} )

ADD_LIBRARY( NUKinect ${SRC_FILES} ${MOC_SRCS})

TARGET_LINK_LIBRARIES( NUKinect 
  freenect 
  ${CMAKE_THREAD_LIBS_INIT} 
  ${GLUT_LIBRARIES} 
  ${QT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBS} 
)

