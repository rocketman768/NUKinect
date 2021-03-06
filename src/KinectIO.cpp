/*
* This file is part of NUKinect.
* 
* Copyright 2011 by the Authors:
* Jiang Wang, <wangjiangb@gmail.com>
* Jiang Xu, <jiangxu2011@u.northwestern.edu>
* Philip G. Lee, <rocketman768@gmail.com>
* 
* NUKinect is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* NUKinect is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with NUKinect.  If not, see <http://www.gnu.org/licenses/>.
*/

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

