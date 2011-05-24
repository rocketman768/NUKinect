#ifndef _HAND_TRACKER_H
#define _HAND_TRACKER_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "pcl/point_types.h"

class HandState
{
public:
	HandState()
	{
		exist = false;
	}
	cv::Vec3f position;
	bool exist;
};
class HandTracker
{
public:
	HandTracker();
	~HandTracker();
	//assume 8bit gray-scale input now
	int SetNewFrame(cv::Mat& newFrame);		
	//only 2d tracking now, the 3rd element is always 0 
	bool getCurrentPosition(cv::Vec3f& position);
	bool getDifference(cv::Vec3f& difference);
	double _numForegroundPoints, _numForegroundPointsValidation,_depthDifferenceThreshold;
	HandState _currentPosition, _lastPosition;
private:

	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > _points;
};

#endif