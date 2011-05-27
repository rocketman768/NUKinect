#ifndef _HAND_TRACKER_H
#define _HAND_TRACKER_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "pcl/point_types.h"
//! the class for the gesture state, currently use integer to represent the states
class GestureState
{
public:
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
private:
	int _existCounter,_unexistCounter;
	GestureState _state;
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
