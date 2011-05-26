#include <QGLWidget>
#include <string.h> // memcpy()
#include "PtCloud.h"

const int PtCloud::FloatsPerPt = 3;

PtCloud::PtCloud(QObject* parent) : QObject(parent) {
  _numPts = 0;
  //_bufferVec.clear();
}

//PtCloud::~PtCloud() {
//  ;
//}

int PtCloud::setPoints(const GLfloat* src, int numPts) {
  _mutex.lock();

  _numPts = numPts;
  _points.resize(FloatsPerPt*_numPts);
  memcpy(&(_points[0]), src, numPts*FloatsPerPt*sizeof(GLfloat));

  _mutex.unlock();

  return 0;
}

void PtCloud::draw() {
  glPushMatrix();
  
  glPointSize(3.0);
  glColor3f(1.0, 0.0, 0.0);

  glBegin(GL_POINTS);
  
  _mutex.lock();

  const GLfloat* pt(&(_points[0]));
  for (int i = 0; i < _numPts; i++) {
    glVertex3fv(pt);
    pt += FloatsPerPt;
  }

  _mutex.unlock();

  glEnd();
  

  glPopMatrix();
}
