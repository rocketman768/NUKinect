#ifndef _KINECT_H
#define _KINECT_H

#include <libfreenect.hpp>
#include <inttypes.h>
#include <boost/shared_array.hpp> // For shared pointers.
#include "Mutex.h"

class Kinect : public Freenect::FreenectDevice
{
public:
  
  Kinect(freenect_context *_ctx, int _index);
  ~Kinect();
  
  /*! Takes \b lastTimestamp as input.
   *  Returns a shared pointer \b ret that points to a COPY of the depth data.
   *  Also, update  \b lastTimestamp by reference. If the internal timestamp
   *  is the same as \b lastTimestamp , \b ret does not change.
   */
  void getDepth(uint32_t& lastTimestamp,
                boost::shared_array<uint8_t>& ret);
  
private:
  
  //! Only called by libfreenect!
  void VideoCallback(void* rgb, uint32_t timestamp);
  
  //! Only called by libfreenect!
  void DepthCallback(void* depth, uint32_t timestamp);

  uint8_t * _depth;
  Mutex _mutex_depth;
  uint32_t _depthTimestamp;
};

#endif /* _KINECT_H */
