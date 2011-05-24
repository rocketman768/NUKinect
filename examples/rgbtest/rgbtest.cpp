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
