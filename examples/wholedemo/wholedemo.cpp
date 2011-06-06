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

void ControlMouse(int x, int y, int maxX, int maxY, int minX, int minY, int maxInX, int maxInY)
{
	MouseController& controller = MouseController::instance();
	float output_x = (float)(maxInX-x)/maxInX*(maxX-minX) + minX;
	float output_y = (float)(y)/maxInY*(maxY-minY) + minY;	
	controller.SetCursorPos((int)output_x, (int)output_y);
}
void ProcessFrame( HandTracker* handTracker, int numForegoundPoints, int numForegoundPointsValidation, int depthThreshold, cv::Mat &depthMat, cv::Mat& depthf, cv::Mat& depthColor ) 
{
	handTracker->NumForegroundPoints(numForegoundPoints);
	handTracker->NumForegroundPointsValidation(numForegoundPointsValidation);
	handTracker->DepthDifferenceThreshold(depthThreshold);
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
	VelocityEstimator* traj_int = (VelocityEstimator*) intepretator;
	cv::Vec3f pos_filtered;
	MouseController &controller = MouseController::instance();
	if (traj_int->GetLocation(pos_filtered))
	{
		cv::circle(depthColor,cv::Point2f(pos_filtered[1],pos_filtered[0]),3,cv::Scalar(255,0,0));
		controller.mousePress(1);
		ControlMouse(pos_filtered[1],pos_filtered[0], 1180, 924, 0, 100,640,480);
	}else
	{
		controller.mouseRelease(1);
	}
	//intepretator->GetGestureState();
	//executor->ExecuteGesture(intepretator->GetGestureState());

}


void* freenect_threadfunc(void* arg) {
  uint32_t lastTimestampDepth = 99999;
  //  uint32_t lastTimestampRgb = 99999;
  boost::shared_array<uint8_t> depth;
  //  boost::shared_array<uint8_t> rgb;
  cv::Mat depthMat(cv::Size(640,480),CV_16UC1),depthf,depthColor;
  cv::Mat rgbMat(cv::Size(640,480),CV_8UC3);
  KinectWindow & myviewcontrol = KinectWindow::instance();
  NUBufferSpec spec(NUBufferSpec::RGB);

  //cv::namedWindow("test",CV_WINDOW_AUTOSIZE);
  
  bool isDataValid(false);
  uint32_t lastLastStampDepth = 0;

  PtCloud cloud;
  
  
  while(true)  {


    isDataValid = myviewcontrol.getDepth(lastTimestampDepth, depth);
    if (!isDataValid||lastTimestampDepth==lastLastStampDepth) {
      continue;
    }
    Kinect::getPointCloud(cloud,depth);  
    //myviewcontrol.loadPtCloud(cloud);

    lastLastStampDepth = lastTimestampDepth;    
    
    //   isDataValid =  myviewcontrol.getRgb(lastTimestampRgb, rgb);
    //if (!isDataValid) {
    //  continue;
    //}
    
    depthMat.data = (uchar*) depth.get();
   
    int numForegoundPoints, numForegoundPointsValidation, depthThreshold;
    myviewcontrol.getControlSliderValue(0, numForegoundPoints);
    myviewcontrol.getControlSliderValue(1, numForegoundPointsValidation);
    myviewcontrol.getControlSliderValue(2, depthThreshold);

	HandTracker* handTracker = (HandTracker*) tracker;
	ProcessFrame(handTracker, numForegoundPoints, numForegoundPointsValidation, depthThreshold, depthMat, depthf, depthColor);

    myviewcontrol.loadBuffer(depthColor.data,spec,0);

    //rgbMat.data = (uchar*) rgb.get();
    myviewcontrol.loadBuffer(depthColor.data,spec,1);
    //myviewcontrol.loadBuffer(rgb.get(),spec,1);
    //    myviewcontrol.loadBuffer(rgbMat.data, spec, 1);

    // test code for control sliders
    //int val;
    //for (int i = 0; i < myviewcontrol.getNumControlSlider(); i++) {
     // myviewcontrol.getControlSliderValue(i, val);
    //  printf("%d, ", val);
    //}
   // printf("\n");
   

  }
  return NULL;
}



int main(int argc, char *argv[]) {
  tracker = new HandTracker();
  intepretator = new VelocityEstimator();
  executor  = new ClickMouseExecutor();
  QApplication app(argc, argv);  
  KinectWindow & myviewcontrol = KinectWindow::instance();
  myviewcontrol.setControlSliderFormat(0, QString::fromAscii("Foreground pixels"), 20, 200, 1, 5, 5);
  myviewcontrol.setControlSliderValue(0,80);
  myviewcontrol.setControlSliderFormat(1, QString::fromAscii("Foreground pixels validation"), 500, 2000, 8, 100, 100);
  myviewcontrol.setControlSliderValue(1,800);
  myviewcontrol.setControlSliderFormat(3, QString::fromAscii("depth threshhold"), 1, 10, 1, 1, 1);
  myviewcontrol.setControlSliderValue(2,6);
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
