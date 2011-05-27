#ifndef PTCLOUD_H
#define PTCLOUD_H

#include <vector>
#include "Mutex.h"

class PtCloud {
 public:
  PtCloud();

  //! Draw the point cloud by OpenGL functions.
  void draw();

  /*! Sets the points in the cloud.
   *  \b src is a vector of points arranged as [x1,y1,z1,x2,y2,z2,...].
   *  \b numPts is how many points are contained in \b src.
   */
  int setPoints(const float* src, int numPts);

  //! Gets the coordinate of point \b ind.
  int getPoint(const int ind, float & x, float & y, float & z) const;

  //! Deep copy another point cloud.
  int loadPtCloud(const PtCloud & cloud_in);

  //! Get the number of points.
  int getNumPts() const {return _numPts;}

  static const int FloatsPerPt;
  
 protected:

  Mutex _mutex;
  int _numPts;

  //! Caution: the float is casted to GLFloat in some functions. So if float != GLFloat, some changes are needed.
  std::vector <float> _points;

};

#endif // PTCLOUD_H
