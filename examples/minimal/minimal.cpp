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

#include <unistd.h>
#include <inttypes.h>
#include <boost/shared_array.hpp>
#include <iostream>
#include "KinectIO.h"

int main()
{
  uint32_t lastTimestamp = 0;
  boost::shared_array<uint8_t> depth;
  
  while(true)
  {
    KinectIO::instance().kinect().getDepth(lastTimestamp, depth);
    std::cerr << "Timestamp: " << lastTimestamp << std::endl;
    usleep(33e3); // Sleep 33 ms.
  }
}
