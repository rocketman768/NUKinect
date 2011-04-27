#include <unistd.h>
#include <inttypes.h>
#include <boost/shared_array.hpp>
#include <stdio.h>
#include "KinectIO.h"

int main()
{
  uint32_t lastTimestamp = 0;
  boost::shared_array<uint8_t> depth;
  
  while(true)
  {
    KinectIO::instance().kinect().getDepth(lastTimestamp, depth);
  }
}