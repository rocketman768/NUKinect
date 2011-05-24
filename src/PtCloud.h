#ifndef PTCLOUD_H
#define PTCLOUD_H

#include <QtOpenGL>
#include <QObject>
#include <vector>
#include "Mutex.h"

class PtCloud : public QObject {
 public:
  PtCloud(QObject *parent);
  //~PtCloud();

  //! Draw the point cloud by OpenGL functions.
  void draw();

  /*! Sets the points in the cloud.
   *  \b src is a vector of points arranged as [x1,y1,z1,x2,y2,z2,...].
   *  \b numPts is how many points are contained in \b src.
   */
  int setPoints(const GLfloat* src, int numPts);

  static const int FloatsPerPt;
  
 protected:

  Mutex _mutex;
  int _numPts;
  std::vector <GLfloat> _points;

};

#endif // PTCLOUD_H
