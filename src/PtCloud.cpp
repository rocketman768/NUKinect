#include <QGLWidget>
#include <QtOpenGL>
#include <string.h> // memcpy()
#include <stdio.h>
#include "PtCloud.h"

const int PtCloud::FloatsPerPt = 3;

PtCloud::PtCloud() {
  _numPts = 0;
}

int PtCloud::setPoints(const float* src, int numPts) {
  _mutex.lock();

  _numPts = numPts;
  _points.resize(FloatsPerPt*_numPts);
  memcpy(&(_points[0]), src, numPts*FloatsPerPt*sizeof(GLfloat));

  _mutex.unlock();

  return 0;
}

int PtCloud::getPoint(const int ind, float & x, float & y, float &z) const {

  if (ind < 0 || ind >= _numPts) {
    printf("Error in PtCloud::getPoint, ind out of range.\n");
    return 1;
  }
 
  const int base = ind * FloatsPerPt;
  x = _points[base+0];
  y = _points[base+1];
  z = _points[base+2];

  return 0;
}


int PtCloud::loadPtCloud(const PtCloud & cloud) {
  _mutex.lock();

  _numPts = cloud.getNumPts();
  _points.resize(FloatsPerPt*_numPts);
  
  float x(0.f);
  float y(0.f);
  float z(0.f);
  float* p(&(_points[0]));
  for (int i = 0; i < _numPts; i++) {
    cloud.getPoint(i, x, y, z);
    *p = x; 
    p++;
    *p = y;
    p++;
    *p = z;
    p++;
  }

  _mutex.unlock();

  return 0;
}

void PtCloud::draw() {
  glPushMatrix();
  
  glPointSize(3.0);
  glColor3f(1.0, 0.0, 0.0);

  glBegin(GL_POINTS);
  
  _mutex.lock();

  const GLfloat* pt(static_cast<GLfloat*>(&(_points[0])));
  for (int i = 0; i < _numPts; i++) {
    glVertex3fv(pt);
    pt += FloatsPerPt;
  }

  _mutex.unlock();

  glEnd();
  

  glPopMatrix();
}
