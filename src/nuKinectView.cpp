#include "./nuKinectView.hpp"

#include "pthread.h"

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif


int NUKinectView::width(0);
int NUKinectView::height(0);
GLuint NUKinectView::gl_depth_tex(0); // temporarily use for debu
GLuint NUKinectView::gl_rgb_tex(0); // temporarily use for debu
int NUKinectView::window(0); // temporarily use for debu

int NUKinectView::numBuffers(0);
std::vector <char *> NUKinectView::buffer_ptrs(0);
NUKinectView * NUKinectView::pInstance(NULL);

int NUKinectView::numCols(0);
int NUKinectView::numRows(0);
Mutex NUKinectView::_mutex;

int NUKinectView::destroy() {
  if (pInstance != NULL) {
    delete pInstance;
    pInstance = NULL;
  }

  return 0;
}

int NUKinectView::setBuffer(char * buffer_ptr, const NUBufferSpec & spec, int buffer_ind) {
  if (buffer_ind >= getNumBuffers()) {
    return 1;
  }
  
  _mutex.lock();

  buffer_ptrs[buffer_ind] = buffer_ptr;
  //buffer_specs[buffer_ind] = spec;

  _mutex.unlock();

  return 0;
}

int NUKinectView::initOpenGL() {
  int argc(0);
  char ** argv(0);
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  return 0;
}


void NUKinectView::NUInitGL(int Width, int Height)
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_SMOOTH);
  glGenTextures(1, &gl_depth_tex);
  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glGenTextures(1, &gl_rgb_tex);
  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  NUReSizeGLScene(Width, Height);
}


void NUKinectView::NUDrawGLScene() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);


  for (int i = 0; i < getNumBuffers(); i++) {
    if (!buffer_ptrs[i]) {
      continue;
    }
	  
    const int col = i % numCols;
    const int row = i / numCols;

    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);

    _mutex.lock();
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, buffer_ptrs[i]);
    _mutex.unlock();

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(0+col*640,0+row*480,0);
    glTexCoord2f(1, 0); glVertex3f(640+col*640,0+row*480,0);
    glTexCoord2f(1, 1); glVertex3f(640+col*640,480+row*480,0);
    glTexCoord2f(0, 1); glVertex3f(0+col*640,480+row*480,0);
    glEnd();

  } // for i

  glutSwapBuffers();
}

void NUKinectView::NUReSizeGLScene(int Width, int Height) {
  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho (0, width, height, 0, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
}

static void *threadGlutMainLoop(void * useless) {
  glutMainLoop();
  pthread_exit(NULL);
  return NULL;
}

int NUKinectView::startDrawLoop() {
  
  NUInitGL(width, height);

  glutDisplayFunc(&NUDrawGLScene);
  glutIdleFunc(&NUDrawGLScene);
  glutReshapeFunc(&NUReSizeGLScene);

 
  // pthread_t thread;
  //long tmp(0);
  //pthread_create(&thread, NULL, threadGlutMainLoop, (void*)tmp);
  
  glutMainLoop();

  return 0;
}
