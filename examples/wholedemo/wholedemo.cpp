#include <unistd.h>
#include <inttypes.h>
#include <boost/shared_array.hpp>
#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include "KinectIO.h"
#include "KinectWindow.h"
#include "HandTracker.h"
#include "MouseController.h"


pthread_t freenect_thread;
Tracker* tracker;
TrajectoryIntepretator* intepretator;
GestureExecutor* executor;

void* freenect_threadfunc(void* arg) {
  uint32_t lastTimestampDepth = 99999;
  //uint32_t lastTimestampRGB = 99999;
  boost::shared_array<uint8_t> depth;
  boost::shared_array<uint8_t> rgb;
  cv::Mat depthMat(cv::Size(640,480),CV_16UC1),depthf,depthColor;
  cv::Mat rgbMat(cv::Size(640,480),CV_8UC3);
  KinectWindow & myviewcontrol = KinectWindow::instance();
  //HandTracker tracker;
  NUBufferSpec spec(NUBufferSpec::RGB);

  //cv::namedWindow("test",CV_WINDOW_AUTOSIZE);
  
  bool isDataValid(false);
  int lastLastStampDepth = 0;
  
  while(true)  {
    //std::cout << "Getting one frame...";
    isDataValid = myviewcontrol.getDepth(lastTimestampDepth, depth);
    if (!isDataValid||lastTimestampDepth==lastLastStampDepth) {
      continue;
    }
    lastLastStampDepth = lastTimestampDepth;    
    // isDataValid =  myviewcontrol.getRgb(lastTimestampRGB, rgb);
    //if (!isDataValid) {
    //  continue;
    //}
    //std::cout << lastTimestamp << "Finished.\n";
    
    depthMat.data = (uchar*) depth.get();
	tracker->SetNewFrame(depthMat);
	ObjectState	state = tracker->getCurrentPosition();
	cv::Vec3f  position = state.position;
    depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
    cv::cvtColor(depthf,depthColor,CV_GRAY2RGB);
    if (state.exist)
    {    
      cv::circle(depthColor,cv::Point2f(position[1],position[0]),100,cv::Scalar(0,255,0));
    }    
    intepretator->SetNewPoint(state);
	executor->ExecuteGesture(intepretator->GetGestureState());
    
        
    
    myviewcontrol.loadBuffer(depthColor.data,spec,0);

    //rgbMat.data = (uchar*) rgb.get();
    //myviewcontrol.loadBuffer(depthColor.data,spec,1);
    //myview.setBuffer(rgbMat.data,spec,1);
 

    static float ptc[] = {-.5,-.5,-.5,
			  .5,.5,.5,
			  0,0,0};

    myviewcontrol.loadPtCloud(ptc, 3);
    //tracker.SetNewFrame(depthf);
    //cv::Vec3d pt = tracker.getCurrentPosition();
    //cv::circle(depthColor,cv::Point(pt(0),pt(1)), 10, cv::Scalar(0,0,255));
  }
  return NULL;
}



int main(int argc, char *argv[]) {
  tracker = new HandTracker();
  intepretator = new ObjectExistSmoother();
  executor  = new ClickMouseExecutor();
  QApplication app(argc, argv);
  KinectWindow::instance();


  //cv::namedWindow("abc",CV_WINDOW_AUTOSIZE);
  int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc,NULL);
  if (res)
    {
      std::cout<<"pthread_create failed\n";
      return 1;
    }


  app.exec();
  delete tracker;
  delete intepretator;
  delete executor;
  return 0;
}
