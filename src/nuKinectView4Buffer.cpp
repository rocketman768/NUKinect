#include "./nuKinectView4Buffer.hpp"


NUKinectView & NUKinectView4Buffer::instance() {
  if (pInstance == NULL) {
    pInstance = new NUKinectView4Buffer;
    
    numCols = 2; // number of cols, you may change it
    numRows = 2; // number of rows, you may change it

    width = 640 * numCols;
    height = 480 * numRows;

    setNumBuffers(numCols*numRows);
    buffer_ptrs.resize(getNumBuffers(), NULL);

    initOpenGL();
    initWindows(); 
  }

  return *pInstance;
}

int NUKinectView4Buffer::initWindows() {

  glutInitWindowSize(width,height);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("NUKinect"); 
  
  return 0;
}
