#ifndef PTCLOUD_H
#define PTCLOUD_H

#include "Mutex.h"
#include <QtOpenGL>
#include <QObject>
#include <vector>

class PtCloud : public QObject {
 public:
  PtCloud(QObject *parent);
  //~PtCloud();
  void draw();
  int copyToBuffer(const GLfloat* src, int numPts);

 protected:

  Mutex _mutex;
  int _numPts;
  std::vector <GLfloat> _bufferVec;

};

#endif // PTCLOUD_H
