#ifndef _NU_KINECT_VIEW_
#define _NU_KINECT_VIEW_

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "Mutex.h"
#include <vector>

class NUBufferSpec {
public:
  enum BufferMode {
    RGB,
    LUMINANCE
  };

  NUBufferSpec(BufferMode m) { mode = m; }
  BufferMode getMode() const { return mode; }

protected:
  BufferMode mode;

};


// NUKinectView is a singleton interface class
class NUKinectView {
public:

  static int destroy();

  int setBuffer(char * buffer_ptr, const NUBufferSpec & spec, int buffer_ind);
  //int setBuffer(char * buffer_ptr, int buffer_ind);
  static int startDrawLoop();
  
protected:
  static int getNumBuffers() { return numBuffers; }
  static void setNumBuffers(int n) {numBuffers = n;}

  static int initOpenGL();

  static std::vector <char *> buffer_ptrs;
  static std::vector <NUBufferSpec> buffer_specs;
  
  static int width;
  static int height;
  static GLuint gl_depth_tex; // temporarily use for debu
  static GLuint gl_rgb_tex; // temporarily use for debu
  static int window; // temporarily use for debu
  static int numBuffers;

  static int numCols; // number of cols
  static int numRows; // number of rows

  NUKinectView(){}
  virtual ~NUKinectView(){}

  static NUKinectView * pInstance;
  static Mutex _mutex;

private:// called by openGL callback functions
  static void NUInitGL(int Width, int Height);
  static void NUDrawGLScene();
  static void NUReSizeGLScene(int Width, int Height);

};

#endif // _NU_KINECT_VIEW_

