#ifndef _NU_KINECT_VIEW_4_BUFFER_
#define _NU_KINECT_VIEW_4_BUFFER_

#include "./nuKinectView.hpp"


class NUKinectView4Buffer:public NUKinectView {
public:
  static NUKinectView & instance();

protected:

  NUKinectView4Buffer(){}
  virtual ~NUKinectView4Buffer(){}
  static int initWindows();
};

#endif // _NU_KINECT_VIEW_4_BUFFER_
