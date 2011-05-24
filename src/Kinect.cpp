#include "Kinect.h"
#include <string.h> // For memcpy()

Kinect::Kinect(freenect_context *_ctx, int _index)
  : Freenect::FreenectDevice(_ctx,_index)
{
  _mutex_depth.lock();
  
  _depth = 0;
  _depthTimestamp = 0;
    
  _mutex_depth.unlock();

  _mutex_bgr.lock();

  _bgr = 0;
  _bgrTimestamp = 0;

  _mutex_bgr.unlock();
}

Kinect::~Kinect()
{
}

// Returns a shared pointer that points to a COPY of the depth data.
// Also, return 'lastTimestamp' by reference.
bool Kinect::getDepth(uint32_t& lastTimestamp,
                      boost::shared_array<uint8_t>& ret)
{
  // Lock b/c we need to access _depth and _depthTimestamp.
  _mutex_depth.lock();
    
  // Don't change 'ret' if it is the same frame.
  if( lastTimestamp == _depthTimestamp || _depth == 0 )
  {
    _mutex_depth.unlock();
    return (_depth!=0);
  }
  
  // Otherwise, change the last timestamp.
  lastTimestamp = _depthTimestamp;
  
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
  return true;
}
 
// Returns a shared pointer that points to a COPY of the bgr data.
// Also, return 'lastTimestamp' by reference.
bool Kinect::getBgr(uint32_t& lastTimestamp,
                    boost::shared_array<uint8_t>& ret)
{
  // Lock b/c we need to access _rgb and _rgbTimestamp.
  _mutex_bgr.lock();
    
  // Don't change 'ret' if it is the same frame.
  if( lastTimestamp == _bgrTimestamp || _bgr == 0 )
  {
    _mutex_bgr.unlock();
    return (_bgr != 0);
  }
  
  // Otherwise, change the last timestamp.
  lastTimestamp = _bgrTimestamp;
  
  // Return a shared array that points to a COPY
  // of our _rgb data. Since it is shared, it will
  // automatically de-allocate when no more references
  // to the memory location exist.
  uint8_t* doNotCopy = new uint8_t[FREENECT_VIDEO_RGB_SIZE];
  ret = boost::shared_array<uint8_t>(
          reinterpret_cast<uint8_t*>(
            memcpy(doNotCopy, _bgr, FREENECT_VIDEO_RGB_SIZE * sizeof(uint8_t))
          )
        );
  doNotCopy = 0; // Protect from stupidity.
    
  // Release lock.
  _mutex_bgr.unlock();
  return true;
}

int Kinect::getDepthWidth()
{
  return 640;
}

int Kinect::getDepthHeight()
{
  return 480;
}

int Kinect::getBgrWidth()
{
  return 640;
}

int Kinect::getBgrHeight()
{
  return 480;
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

void Kinect::VideoCallback(void* bgr, uint32_t timestamp)
{
  _mutex_bgr.lock();

  _bgr = reinterpret_cast<uint8_t*>(bgr);
  _bgrTimestamp = timestamp;

  _mutex_bgr.unlock();
}
