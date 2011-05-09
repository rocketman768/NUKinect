#include <unistd.h>
#include <inttypes.h>
#include <boost/shared_array.hpp>
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include "KinectIO.h"
#include "wholedemo.hpp"
#include "KinectWindow.h"
#include "HandTracker.h"
#include "MouseController.h"


pthread_t freenect_thread;

void* freenect_threadfunc(void* arg)
{
  uint32_t lastTimestamp = 0;
  boost::shared_array<uint8_t> depth;
  cv::Mat depthMat(cv::Size(640,480),CV_16UC1),depthf,depthColor;
  KinectWindow & myview = KinectWindow::instance();
  HandTracker tracker;
  NUBufferSpec spec(NUBufferSpec::RGB);
  cv::namedWindow("test",CV_WINDOW_AUTOSIZE);
  while(true)
    {
      std::cout << "Getting one frame...";
      KinectIO::instance().kinect().getDepth(lastTimestamp, depth);
      std::cout << lastTimestamp << "Finished.\n";
      depthMat.data = (uchar*) depth.get();
      depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
      cv::cvtColor(depthf,depthColor,CV_GRAY2RGB);
      myview.setBuffer(depthColor.data,spec,0);
      myview.setBuffer(depthColor.data,spec,1);
      tracker.SetNewFrame(depthf);
      cv::Vec3d pt = tracker.getCurrentPosition();
      cv::circle(depthColor,cv::Point(pt(0),pt(1)), 10, cv::Scalar(0,0,255));
    }
  return NULL;
}



int main(int argc, char *argv[]) {

  QApplication app(argc, argv);
  KinectWindow::instance();

  cv::namedWindow("abc",CV_WINDOW_AUTOSIZE);
  int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc,NULL);
  if (res)
    {
      std::cout<<"pthread_create failed\n";
      return 1;
    }


  app.exec();

  return 0;
}
