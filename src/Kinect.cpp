#include "Kinect.h"
#include <string.h> // For memcpy()

Kinect::Kinect(freenect_context *_ctx, int _index)
  : Freenect::FreenectDevice(_ctx,_index)
{
  _mutex_depth.lock();
  
  _depth = new uint8_t[ FREENECT_DEPTH_11BIT_SIZE ];
  _depthTimestamp = 0;
    
  _mutex_depth.unlock();
}

Kinect::~Kinect()
{
  _mutex_depth.lock();
  
  delete[] _depth;
  
  _mutex_depth.unlock();
}

// Returns a shared pointer that points to a COPY of the depth data.
// Also, return 'retFrameNo' by reference.
void Kinect::getDepth(uint32_t lastTimestamp,
                        boost::shared_array<uint8_t>& ret,
                        uint32_t& retTimestamp)
{
  // Lock b/c we need to access _depth and _depthTimestamp.
  _mutex_depth.lock();
  
  retTimestamp = _depthTimestamp;
    
  // Don't change 'ret' if it is the same frame.
  if( lastTimestamp == _depthTimestamp )
  {
    _mutex_depth.unlock();
    return;
  }
  
  // Return a shared array that points to a COPY
  // of our _depth data. Since it is shared, it will
  // automatically de-allocate when no more references
  // to the memory location exist.
  uint8_t* doNotCopy = new uint8_t[FREENECT_DEPTH_11BIT_SIZE];
  ret = boost::shared_array<uint8_t>(
          reinterpret_cast<uint8_t*>(
            memcpy(doNotCopy, _depth, FREENECT_DEPTH_11BIT_SIZE * sizeof(uint8_t))
          )
        );
  doNotCopy = 0; // Protect from stupidity.
    
  // Release lock.
  _mutex_depth.unlock();
}
  
// Only gets called by libfreenect.
void Kinect::DepthCallback(void* depth, uint32_t timestamp)
{
  // Lock b/c we are going to modify _depth and _depthTimestamp.
  _mutex_depth.lock();
    
  _depth = reinterpret_cast<uint8_t*>(depth);
  _depthTimestamp = timestamp;
    
  // Release lock.
  _mutex_depth.unlock();
}

void Kinect::VideoCallback(void* rgb, uint32_t timestamp)
{
}