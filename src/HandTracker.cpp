#include "HandTracker.h"
using namespace std;

#include <iostream>
HandTracker::HandTracker() :_threshold(170), _blobMin(20),_blobMax(90)
{

}

HandTracker::~HandTracker()
{

}

int HandTracker::SetNewFrame( cv::Mat& depthf )
{
	cv::Point2f center;
	float radius=0;
	cv::Mat blurred, thresholded, thresholded2, output;
	depthf = 255-depthf;       
	cv::blur(depthf, blurred, cv::Size(10,10));       
	cv::threshold( blurred, thresholded, _threshold, 255, CV_THRESH_BINARY);	
	cv::threshold( blurred, thresholded2, _threshold, 255, CV_THRESH_BINARY);
	cv::cvtColor( thresholded2, output, CV_GRAY2RGB );
	vector<vector<cv::Point> > contours;
	// find em
	cv::findContours(thresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	// loop the stored contours
	for (vector<vector<cv::Point> >::iterator it=contours.begin() ; it < contours.end(); it++ ){

		// center abd radius for current blob
		// convert the cuntour point to a matrix 
		vector<cv::Point> pts = *it;
		cv::Mat pointsMatrix = cv::Mat(pts);
		// pass to min enclosing circle to make the blob 
		cv::minEnclosingCircle(pointsMatrix, center, radius);

		cv::Scalar color( 0, 255, 0 );
		std::cout<<radius<<std::endl;          
		if (radius > _blobMin && radius < _blobMax) {
			// draw the blob if it's in range
			std::cout<<"Yes"<<std::endl;            
			cv::circle(output, center, radius, color);
		}
	} 
	_lastPosition = _currentPosition;
	if (radius > _blobMin && radius < _blobMax)
	{    		
		_currentPosition = cv::Vec3d(center.x, center.y,0);
	}else
	{
		_currentPosition = cv::Vec3d(0,0,0);
	}
    return 1;    
}

cv::Vec3d HandTracker::getCurrentPosition()
{
	return _currentPosition;
}

cv::Vec3d HandTracker::getDifference()
{
	return _currentPosition - _lastPosition;
}

void HandTracker::SetParameters( double threshold, double blobMin, double blobMax )
{
	_threshold = threshold;
	_blobMax = blobMax;
	_blobMin = blobMin;
}
