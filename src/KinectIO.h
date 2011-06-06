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