#ifndef PTCLOUD_H
#define PTCLOUD_H

#include <QtOpenGL>
#include <QObject>
#include <vector>

class PtCloud : public QObject {
 public:
  PtCloud(QObject *parent);
  //~PtCloud();
  void draw() const;
  int copyToBuffer(const GLfloat* src, int numPts);

 protected:

  int _numPts;
  std::vector <GLfloat> _bufferVec;

};

#endif // PTCLOUD_H
