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

#include <unistd.h> // usleep()
#include <inttypes.h>
#include <boost/shared_array.hpp>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "KinectIO.h"
#include "Kinect.h"

int main()
{
  uint32_t lastTimestamp = 0;
  boost::shared_array<uint8_t> rgb;
  cv::Mat rgbMat( cv::Size(Kinect::getBgrWidth(),Kinect::getBgrHeight()), CV_8UC3, cv::Scalar(0) );
  
  cv::namedWindow("rgb");
  
  // Throw away some frames.
  while(lastTimestamp == 0)
  {
    KinectIO::instance().kinect().getBgr(lastTimestamp, rgb);
    //std::cerr << "Timestamp: " << lastTimestamp << std::endl;
    usleep(33e3); // Sleep 33 ms.
  }
  
  // End when user presses enter.
  while( cv::waitKey(33) != '\n' )
  {
    KinectIO::instance().kinect().getBgr(lastTimestamp, rgb);
    rgbMat.data = rgb.get();
    cv::imshow( "rgb", rgbMat );
    std::cerr << "Timestamp: " << lastTimestamp << std::endl;
    //usleep(33e3); // Sleep 33 ms.
  }
  
  return 0;
}
