#include <QGLWidget>

#include "PtCloud.h"

const int FloatsPerPt = 3;

PtCloud::PtCloud(QObject* parent) : QObject(parent) {
  _numPts = 0;
  //_bufferVec.clear();
}

//PtCloud::~PtCloud() {
//  ;
//}

int PtCloud::copyToBuffer(const GLfloat* src, int numPts) {
  _numPts = numPts;
  _bufferVec.resize(FloatsPerPt*_numPts);
  memcpy(&(_bufferVec[0]), src, numPts*FloatsPerPt*sizeof(float));

  return 0;
}

void PtCloud::draw() const {
  glPushMatrix();
  
  glPointSize(3.0);
  glColor3f(1.0, 0.0, 0.0);

  glBegin(GL_POINTS);
  
  const GLfloat* pt(&(_bufferVec[0]));
  for (int i = 0; i < _numPts; i++) {
    glVertex3fv(pt);
    pt += FloatsPerPt;
  }

  glEnd();
  

  glPopMatrix();
}
