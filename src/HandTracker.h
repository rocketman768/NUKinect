#ifndef _HAND_TRACKER_H
#define _HAND_TRACKER_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

class HandTracker
{
public:
	HandTracker();
	~HandTracker();
	void SetParameters(double threshold, double blobMin, double blobMax);
	//assume 8bit gray-scale input now
	int SetNewFrame(cv::Mat& newFrame);		
	//only 2d tracking now, the 3rd element is always 0 
	cv::Vec3d getCurrentPosition();
	cv::Vec3d getDifference();
private:
	cv::Vec3d _currentPosition, _lastPosition;
	double _threshold,_blobMin, _blobMax;		
};

#endif