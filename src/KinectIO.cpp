#include "KinectIO.h"

KinectIO* KinectIO::_instance = 0;
Mutex KinectIO::_instanceMutex;

KinectIO::~KinectIO()
{
  // Shut down _kinect.
  _kinect.stopDepth();
  _kinect.stopVideo();
}

KinectIO::KinectIO()
  : _kinect( _freenect.createDevice<Kinect>(0) )
{  
  // Start the kinect video stuff.
  _kinect.startDepth();
  _kinect.startVideo();
}

KinectIO::KinectIO( const KinectIO& other )
  : _kinect( _freenect.createDevice<Kinect>(0) )
{
  // Do nothing.
}

const KinectIO& KinectIO::operator = ( KinectIO const& other )
{
  // Do nothing
  return *this;
}

KinectIO& KinectIO::instance()
{
  // Double-checking avoids having to
  // get the lock every time.
  if( _instance == 0 )
  {
    _instanceMutex.lock();
    
    if( _instance == 0 )
      _instance = new KinectIO();
    
    _instanceMutex.unlock();
  }
  
  return *(_instance);
}

Kinect& KinectIO::kinect()
{
  return _kinect;
}

