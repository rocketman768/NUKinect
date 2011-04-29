#include <unistd.h>
#include <inttypes.h>
#include <boost/shared_array.hpp>
#include <stdio.h>
#include "KinectIO.h"
#include <iostream>

int main()
{
  uint32_t lastTimestamp = 0;
  boost::shared_array<uint8_t> depth;
  
  while(true)
  {
    std::cout << "Getting one frame...";
    KinectIO::instance().kinect().getDepth(lastTimestamp, depth);
    std::cout << lastTimestamp << "Finished.\n";
  }
}
