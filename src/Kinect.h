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

#ifndef _KINECT_H
#define _KINECT_H

#include <libfreenect.hpp>
#include <inttypes.h>
#include <boost/shared_array.hpp> // For shared pointers.
#include "Mutex.h"
#include "PtCloud.h"

class Kinect : public Freenect::FreenectDevice
{
public:
  
  Kinect(freenect_context *_ctx, int _index);
  ~Kinect();
  
  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  bool getDepth(uint32_t& lastTimestamp,
                boost::shared_array<uint8_t>& ret);
                
  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret by reference that points to a COPY of the BGR data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   *  The bool return value is false if the data in \b ret may be invalid.
   */
  bool getBgr(uint32_t& lastTimestamp,
              boost::shared_array<uint8_t>& ret);
  
  //! Returns the width of a depth frame.
  static int getDepthWidth();

  //! Returns the height of a depth frame.
  static int getDepthHeight();
  
  //! Returns the width of an rgb frame.
  static int getBgrWidth();
  
  //! Returns the height of an rgb frame.
  static int getBgrHeight();
  
  /*! Translate \b depth to a point cloud \b cloud.
   *  All point locations are in meters.
   */
  static void getPointCloud( PtCloud& cloud, boost::shared_array<uint8_t> depth );
private:
  
  //! Only called by libfreenect!
  void VideoCallback(void* bgr, uint32_t timestamp);
  
  //! Only called by libfreenect!
  void DepthCallback(void* depth, uint32_t timestamp);

  uint8_t* _depth;
  uint8_t* _bgr;

  Mutex _mutex_depth;
  Mutex _mutex_bgr;

  uint32_t _depthTimestamp;
  uint32_t _bgrTimestamp;
};

#endif /* _KINECT_H */
