#include "KinectIO.h"

static KinectIO* KinectIO::_instance = 0;

KinectIO::~KinectIO()
{
  // Shut down and delete _kinect.
}

KinectIO::KinectIO()
{
  // Initialize the Kinect variable.
  
  // Start the kinect video stuff.
}

KinectIO::KinectIO( KinectIO& other )
{
  // Do nothing.
}

KinectIO& KinectIO::operator = ( KinectIO const& other )
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