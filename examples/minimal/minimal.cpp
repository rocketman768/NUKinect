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
