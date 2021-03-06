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

#include "HandTracker.h"
#include <pcl/visualization/cloud_viewer.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <MouseController.h>
using namespace std;
using namespace pcl;

void print_matrix( cv::Mat &mat ) 
{
	for (int i=0;i<mat.rows;++i)
	{
		for (int j=0;j<mat.cols;++j)
		{
			cout<<mat.at<float>(i,j)<<" ";
		}
		cout<<endl;
	}
}
bool ComparePoint (PointXYZ i, PointXYZ j)
{
	return (i.z<j.z);  
}
HandTracker::HandTracker() :_numForegroundPoints(80),_numForegroundPointsValidation(800),_depthDifferenceThreshold(6)
{
	_lastPosition.exist =false;
	_currentPosition.exist = false;
}

HandTracker::~HandTracker()
{

}

//TODO add adaptive points number adjustment
int HandTracker::SetNewFrame( const cv::Mat& depthf )
{
	size_t rows= depthf.rows;
	size_t cols = depthf.cols;
	_points.resize(depthf.rows*depthf.cols);
	int count=0;
	for (size_t  i=0;i<rows;++i)
		for (size_t j=0;j<cols;++j)
			_points[count++] = PointXYZ(i,j,depthf.at<uint16_t>(i,j));
// 	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
// 	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
// 	cloud->points = _points;	
// 	viewer.showCloud (cloud,"cloud");
	nth_element(_points.begin(),_points.begin()+_numForegroundPointsValidation,_points.end(),ComparePoint);
	nth_element(_points.begin(),_points.begin()+_numForegroundPoints,_points.begin()+_numForegroundPointsValidation,ComparePoint);	
	_lastPosition = _currentPosition;
	//cv::Mat testMat(depthf.size(),CV_8UC3);
	if (_points[_numForegroundPointsValidation].z-_points[_numForegroundPoints].z>_depthDifferenceThreshold)
	{
		float mean_X=0, mean_Y =0,mean_Z=0;
		for (size_t i=0;i<_numForegroundPoints;++i)
		{
			mean_X += _points[i].x;
			mean_Y += _points[i].y;
			//testMat.at<cv::Scalar>(_points[i].y,_points[i].x) =cv::Scalar(255,0,0);
			mean_Z += _points[i].z;
			//cout<<_points[i].x<<" "<<_points[i].y<<" "<<_points[i].z<<" "<<endl;
		}
		mean_X /= _numForegroundPoints;
		mean_Y /= _numForegroundPoints;
		mean_Z /= _numForegroundPoints;
		//cv::imwrite("test.jpg",testMat);
        #ifdef DEBUG
		cout<<_points[_numForegroundPointsValidation].z-_points[_numForegroundPoints].z<<endl;
        #endif
		_currentPosition.position = cv::Vec3f(mean_X,mean_Y,mean_Z);
        _currentPosition.exist = true;
        
	}else
	{
      _currentPosition.exist = false;      
	  _currentPosition.position = _lastPosition.position; //for kalman filtering
	}
	return 1;
}

ObjectState HandTracker::getCurrentPosition()
{
	return (_currentPosition);
}

bool HandTracker::getDifference(cv::Vec3f& difference)
{

	difference =  _currentPosition.position - _lastPosition.position;
	return (_currentPosition.exist&&_lastPosition.exist);
}



bool ObjectExistSmoother::SetNewPoint( ObjectState state )
{
	if (state.exist)
	{
		_existCounter++;		
	}    
	else
	{
		_unexistCounter++;
		if (_unexistCounter>10)			
			_existCounter = 0;
	}	

	if (_existCounter ==10&& _unexistCounter>10)
	{
		_unexistCounter =0;      
		_state.state = 1;
	}else
	{
		_state.state=0;
	}
    return true;    
}

GestureState ObjectExistSmoother::GetGestureState()
{
	return _state;
}

bool ClickMouseExecutor::ExecuteGesture( const GestureState& state )
{
	if (state.state==1)
	{
		MouseController& controller   = MouseController::instance();
		controller.mousePress(1);
		controller.mouseRelease(1);
	}
    return true;    
}

bool VelocityEstimator::SetNewPoint( ObjectState state )
{
	_smoother.SetNewPoint(state);
	cv::Mat measurement  = (cv::Mat_<float>(2,1)<<state.position[0],state.position[1]);
	_kf.predict();
// 	print_matrix(measurement);
// 	print_matrix(_kf.measurementMatrix);
// 	print_matrix(_kf.transitionMatrix);
// 	print_matrix(_kf.processNoiseCov);
// 	print_matrix(_kf.measurementNoiseCov);
	_kf.correct(measurement);
#if DEBUG
	print_matrix(_kf.statePost);
#endif
	float v_x = _kf.statePost.at<float>(3,0);
	float v_y = _kf.statePost.at<float>(4,0);
	if (_velocities.size()<10)
		_velocities.push_back(cv::Vec2f(v_x,v_y));
	else
	{
		_velocities.erase(_velocities.begin());
		_velocities.push_back(cv::Vec2f(v_x,v_y));
	}
	_framecout++;
	return true;
}

GestureState VelocityEstimator::GetGestureState()
{    
	GestureState state;
	if (_velocities.size()>=10)
	{
		list<cv::Vec2f>::iterator iter = _velocities.begin();
		float sum =0;
		int count =0;
		for (;iter!=_velocities.end();++iter)
		{
			sum +=(*iter)[0];
			count++;
		}
		cout<<"averge velocity:"<<sum/count<<endl;
		if (sum/count>5&&_framecout>30)
		{
			state.state=1;
			_framecout=0;

		}else
		{
			state.state=0;
		}
	}
	else
	{
		state.state= 0;
	}

	return state;
}

bool VelocityEstimator::GetLocation(cv::Vec3f & pos)
{	
	pos = cv::Vec3f(_kf.statePost.at<float>(0,0),_kf.statePost.at<float>(1,0),-1);
	if (_smoother.ExistCounter()>=10||_smoother.UnexistCounter()<10)
		return true;
	else
		return false;
}

VelocityEstimator::VelocityEstimator() :_kf(4,2),_currentState(4,1,CV_32F),_framecout(0)
{
	_kf.transitionMatrix =*(cv::Mat_<float>(4,4)<<1,0,1,0,0,1,0,1,0,0,1,0,0,0,1);
	_kf.measurementMatrix =*(cv::Mat_<float>(2,4)<<1,0,0,0,0,1,0,0);
	cv::setIdentity(_kf.processNoiseCov, cv::Scalar::all(1));
	cv::setIdentity(_kf.measurementNoiseCov, cv::Scalar::all(1));
	cv::setIdentity(_kf.errorCovPost,cv::Scalar::all(1));
	cv::Mat temp(4,1,CV_32F);
	cv::randn(temp, cv::Scalar::all(100), cv::Scalar::all(0.1));
	_kf.statePost = temp;
}
