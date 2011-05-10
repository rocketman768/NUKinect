#include "Kinect.h"
#include <string.h> // For memcpy()

Kinect::Kinect(freenect_context *_ctx, int _index)
  : Freenect::FreenectDevice(_ctx,_index)
{
  _mutex_depth.lock();
  
  _depth = 0;
  _depthTimestamp = 0;
    
  _mutex_depth.unlock();

  _mutex_rgb.lock();

  _rgb = 0;
  _rgbTimestamp = 0;

  _mutex_rgb.unlock();
}

Kinect::~Kinect()
{
}

// Returns a shared pointer that points to a COPY of the depth data.
// Also, return 'lastTimestamp' by reference.
void Kinect::getDepth(uint32_t& lastTimestamp,
                      boost::shared_array<uint8_t>& ret)
{
  // Lock b/c we need to access _depth and _depthTimestamp.
  _mutex_depth.lock();
    
  // Don't change 'ret' if it is the same frame.
  if( lastTimestamp == _depthTimestamp || _depth == 0 )
  {
    _mutex_depth.unlock();
    return;
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
}
 
// Returns a shared pointer that points to a COPY of the rgb data.
// Also, return 'lastTimestamp' by reference.
void Kinect::getRgb(uint32_t& lastTimestamp,
                    boost::shared_array<uint8_t>& ret)
{
  // Lock b/c we need to access _rgb and _rgbTimestamp.
  _mutex_rgb.lock();
    
  // Don't change 'ret' if it is the same frame.
  if( lastTimestamp == _rgbTimestamp || _rgb == 0 )
  {
    _mutex_rgb.unlock();
    return;
  }
  
  // Otherwise, change the last timestamp.
  lastTimestamp = _rgbTimestamp;
  
  // Return a shared array that points to a COPY
  // of our _rgb data. Since it is shared, it will
  // automatically de-allocate when no more references
  // to the memory location exist.
  uint8_t* doNotCopy = new uint8_t[FREENECT_VIDEO_RGB_SIZE];
  ret = boost::shared_array<uint8_t>(
          reinterpret_cast<uint8_t*>(
            memcpy(doNotCopy, _rgb, FREENECT_VIDEO_RGB_SIZE * sizeof(uint8_t))
          )
        );
  doNotCopy = 0; // Protect from stupidity.
    
  // Release lock.
  _mutex_rgb.unlock();
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
  _mutex_rgb.lock();

  _rgb = reinterpret_cast<uint8_t*>(rgb);
  _rgbTimestamp = timestamp;

  _mutex_rgb.unlock();
}
