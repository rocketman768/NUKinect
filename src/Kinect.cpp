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

#include <QtOpenGL>
#include <string.h> // For memcpy()
#include "Kinect.h"

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

// http://nicolas.burrus.name/index.php/Research/KinectCalibration
void Kinect::getPointCloud( PtCloud& cloud, boost::shared_array<uint8_t> depth )
{
  int rows = getDepthHeight();
  int cols = getDepthWidth();
  int i,j,k;
  int n = rows*cols;
  int numpoints = 0;
  
  /****** RGB camera parameters ****
  fx_rgb 5.2921508098293293e+02
  fy_rgb 5.2556393630057437e+02
  cx_rgb 3.2894272028759258e+02
  cy_rgb 2.6748068171871557e+02
  k1_rgb 2.6451622333009589e-01
  k2_rgb -8.3990749424620825e-01
  p1_rgb -1.9922302173693159e-03
  p2_rgb 1.4371995932897616e-03
  k3_rgb 9.1192465078713847e-01
  */
  
  // Depth camera parameters.
  float fx_d = 5.9421434211923247e+02; // Focal length for x (in pixels?).
  float fy_d = 5.9104053696870778e+02; // Focal length for y (in pixels?).
  float cu_d = 3.3930780975300314e+02; // u center in pixels.
  float cv_d = 2.4273913761751615e+02; // v center in pixels.
  //float p1_d = -7.6275862143610667e-04;
  //float p2_d = 5.0350940090814270e-03;
  //float k1_d = -2.6386489753128833e-01; // Some lens distortion parameters.
  //float k2_d = 9.9966832163729757e-01;
  //float k3_d = -1.3053628089976321e+00;
  
  //boost::shared_array<GLfloat> points = boost::shared_array<GLfloat>(new GLfloat[ PtCloud::FloatsPerPt * n ]);
  GLfloat* points = new GLfloat[ PtCloud::FloatsPerPt * n ];
  GLfloat* point = points;
  
  // k = i*cols + j
  for( i = 0, k=0; i < rows; ++i )
  {
    for( j = 0; j < cols; ++j,++k )
    {
      // Ensure valid depth.
      if( depth[k] >= 2047 )
        continue;
      
      // Translate depth to z.
      point[2] = 1.0 / ( static_cast<float>(depth[k]) * -0.0030711016 + 3.3309495161);
      
      // Translate i,j to x,y.
      point[0] = (j - cu_d) * point[2] / fx_d;
      point[1] = (i - cv_d) * point[2] / fy_d;
      
      point += PtCloud::FloatsPerPt;
      numpoints++;
    }
  }
  
  cloud.setPoints( points, numpoints );
  
  delete[] points;
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
