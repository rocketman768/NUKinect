#ifndef _HAND_TRACKER_H
#define _HAND_TRACKER_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "pcl/point_types.h"
class GestureState
{
public:
	int state;
};
class ObjectState
{
public:
	ObjectState()
	{
		exist = false;
	}
	cv::Vec3f position;
	bool exist;
};

class Tracker
{
public:
	virtual ~Tracker(){}
	virtual int SetNewFrame(const cv::Mat& newFrame)=0;	
	virtual ObjectState getCurrentPosition()=0;
};

class TrajectoryIntepretator
{
public:
	TrajectoryIntepretator()
	{

	}
	virtual ~TrajectoryIntepretator()
	{

	}
	virtual bool SetNewPoint(ObjectState)=0;
	virtual GestureState GetGestureState()=0;
protected:
	
private:
};

class ObjectExistSmoother:public TrajectoryIntepretator
{
public:
	ObjectExistSmoother():_existCounter(0),_unexistCounter(0)
	{

	}
	virtual ~ObjectExistSmoother()
	{
	}
	virtual bool SetNewPoint(ObjectState state);
	virtual GestureState GetGestureState();
private:
	int _existCounter,_unexistCounter;
	GestureState _state;
};
class GestureExecutor
{
public:
	GestureExecutor()
	{

	}
	virtual ~GestureExecutor()
	{

	}
	virtual bool ExecuteGesture(const GestureState& state)=0;
protected:
	
private:
};

class ClickMouseExecutor:public GestureExecutor
{
public:
	ClickMouseExecutor()
	{

	}
	virtual ~ClickMouseExecutor()
	{

	}
	virtual bool ExecuteGesture(const GestureState& state);
protected:
	
private:
};

class HandTracker:public Tracker
{
public:
	HandTracker();
	virtual ~HandTracker();
	//assume 8bit gray-scale input now
	virtual int SetNewFrame(const cv::Mat& newFrame);		
    virtual ObjectState getCurrentPosition();
	virtual bool getDifference(cv::Vec3f& difference);
	double _numForegroundPoints, _numForegroundPointsValidation,_depthDifferenceThreshold;
	ObjectState _currentPosition, _lastPosition;
private:

	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > _points;
};

#endif
