#include "HandTracker.h"
#include <pcl/visualization/cloud_viewer.h>
#include <algorithm>
#include <iostream>
#include <vector>
using namespace std;
using namespace pcl;


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
int HandTracker::SetNewFrame( cv::Mat& depthf )
{
	size_t rows= depthf.rows;
	size_t cols = depthf.cols;
	_points.resize(depthf.rows*depthf.cols);
	int count=0;
	for (int i=0;i<rows;++i)
		for (int j=0;j<cols;++j)
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
		cout<<_points[_numForegroundPointsValidation].z-_points[_numForegroundPoints].z<<endl;
		_currentPosition.position = cv::Vec3f(mean_X,mean_Y,mean_Z);
        _currentPosition.exist = true;
        
	}else
	{
      _currentPosition.exist = false;      
	}
	return 1;
}

bool HandTracker::getCurrentPosition(cv::Vec3f& position)
{
	position = _currentPosition.position;
	return (_currentPosition.exist);
}

bool HandTracker::getDifference(cv::Vec3f& difference)
{

	difference =  _currentPosition.position - _lastPosition.position;
	return (_currentPosition.exist&&_lastPosition.exist);
}


