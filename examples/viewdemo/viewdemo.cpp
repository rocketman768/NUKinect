/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include "nuKinectView4Buffer.hpp"
#include "viewdemo.hpp"

static std::vector<uint8_t> depth(640*480*4);
static std::vector<uint8_t> rgb(640*480*4);


int main(int argc, char **argv) {


  //----------------
  // Fill the two buffers: "depth" and "rgb" with some data
  Freenect::Freenect freenect;

  MyFreenectDevice * mydevice = &freenect.createDevice<MyFreenectDevice>(0);
  mydevice->startVideo();
  mydevice->startDepth();

  for (int i = 0; i < 100; i ++) {
    mydevice->updateState();
    printf("\r device tilt angle: %+4.2f", mydevice->getState().getTiltDegs());
    fflush(stdout);

    mydevice->getDepth(depth);
    mydevice->getRGB(rgb);
  }


  //----------------
  // Drawing data in "depth" and "rgb" buffers
  NUKinectView & myview = NUKinectView4Buffer::instance();
  NUBufferSpec spec(NUBufferSpec::RGB);  


  myview.setBuffer((char*)&(rgb[0]), spec, 0);
  myview.setBuffer((char*)&(depth[0]), spec, 0);
  myview.setBuffer((char*)&(depth[0]), spec, 2);
  myview.setBuffer((char*)&(rgb[0]), spec, 3);

  myview.startDrawLoop();

  while (1) {

  }




  //----------------
  // Destroy
  NUKinectView::destroy();

  mydevice->stopVideo();
  mydevice->stopDepth();

  printf("\n");
  return 0;
}
