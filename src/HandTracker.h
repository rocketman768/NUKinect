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

#ifndef _HAND_TRACKER_H
#define _HAND_TRACKER_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "pcl/point_types.h"
#include <list>

//! the class for the gesture state, currently use integer to represent the states
class GestureState
{
public:
	GestureState(int s):state(s){}	
	GestureState(){}
	int state;	
};

//! the object state class, curently have the location of a object and whether this object exists in this frame
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

//! pure virtual class for tracker
class Tracker
{
public:
	virtual ~Tracker(){}
    /*! takes \b newframe as input, returns whether this frame is succesfully processed
        returns -1 if the current frame is not successfully processed, otherwise it is successfully processed
     */
	virtual int SetNewFrame(const cv::Mat& newFrame)=0;	
    //! returns the object state of the last processed frame.
	virtual ObjectState getCurrentPosition()=0;
};

//! pure virtual class for trajectory intepretator, which has a stream of trajectory as input and output a gesture state.
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

//! Detects whether an object exist in the frame by performing some smoothing
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
	int ExistCounter() const { return _existCounter; }
	int UnexistCounter() const { return _unexistCounter; }
private:
	int _existCounter,_unexistCounter;
	
	void UnexistCounter(int val) { _unexistCounter = val; }

	void ExistCounter(int val) { _existCounter = val; }
	GestureState _state;
};

class VelocityEstimator:public TrajectoryIntepretator
{
public:
	VelocityEstimator();
	~VelocityEstimator()
	{}
	virtual bool SetNewPoint(ObjectState state);
	virtual GestureState GetGestureState();
	bool GetLocation(cv::Vec3f& pos);	
private:
	cv::KalmanFilter _kf;
	cv::Mat _currentState;
	list<cv::Vec2f> _velocities;
	double _framecout;
	ObjectExistSmoother _smoother;

};
//! The pure virtual Gesture Executor class, which execute some actions according to some gesture states
class GestureExecutor
{
public:
	GestureExecutor()
	{

	}
	virtual ~GestureExecutor()
	{

	}
    //! \b state is a input gesture state, and returns whether the operations is succesful
	virtual bool ExecuteGesture(const GestureState& state)=0;
protected:
	
private:
};

//! click the left mouse button if the gesture state is 1.
class ClickMouseExecutor:public GestureExecutor
{
public:
	ClickMouseExecutor()
	{

	}
	virtual ~ClickMouseExecutor()
	{

	}
    //! click the left mouse button if the gesture state is 1.
	virtual bool ExecuteGesture(const GestureState& state);
protected:
	
private:
};

//! Hand tracker class
class HandTracker:public Tracker
{
public:
	HandTracker();
	virtual ~HandTracker();
	//! input \b newFrame is an new frame to be prossed, assume 8bit gray-scale input now, the current algorithm find the hand by looking at the nearest \b _numForegroundPoints points and \b _numForegroundPointsValidation, if their depth difference exceeds \b _depthDiferenceThreshold, the nearest _numForegoundPoints are considered to be a hand
	virtual int SetNewFrame(const cv::Mat& newFrame);		
   
    virtual ObjectState getCurrentPosition();
	virtual bool getDifference(cv::Vec3f& difference);
    //! getter class for _depthDifferenceThreshold
	double DepthDifferenceThreshold() const { return _depthDifferenceThreshold; }
    //! setter class for _depthDifferenceThreshold
	void DepthDifferenceThreshold(double val) { _depthDifferenceThreshold = val; }
    //! getter class for _numForegroundPointsValidation
	double NumForegroundPointsValidation() const { return _numForegroundPointsValidation; }
    //! setter class for  _numForegroundPointsValidation
	void NumForegroundPointsValidation(double val) { _numForegroundPointsValidation = val; }
    //! getter class for _numForegroundPoints
	double NumForegroundPoints() const { return _numForegroundPoints; }
    //! setter class for  _numForegroundPoints
	void NumForegroundPoints(double val) { _numForegroundPoints = val; }
	ObjectState _currentPosition, _lastPosition;
private:
	double _numForegroundPoints, _numForegroundPointsValidation,_depthDifferenceThreshold;
	
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > _points;
};

#endif
