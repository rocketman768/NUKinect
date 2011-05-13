#ifndef _KINECTIO_H
#define _KINECTIO_H

#include <libfreenect.hpp>
#include "Kinect.h"
#include "Mutex.h"

/*! \b KinectIO is a singleton class.
 */
class KinectIO
{
public:
  ~KinectIO();
  
  //! Get the \b KinectIO instance.
  static KinectIO& instance();
  
  //! Get the \b Kinect variable.
  Kinect& kinect();
  
protected:
  //! Hidden constructor.
  KinectIO();
  //! Hidden copy constructor. Do not use.
  KinectIO( const KinectIO& other );
  //! Hidden assignment operator. Do not use.
  const KinectIO& operator = ( KinectIO const& other );
  
private:
  
  static KinectIO* _instance;
  static Mutex _instanceMutex;
  Freenect::Freenect _freenect;
  Kinect& _kinect;
};

#endif /* _KINECTIO_H */