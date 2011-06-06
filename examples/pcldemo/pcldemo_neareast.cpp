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

#include <iostream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "KinectIO.h"
#include "DepthMapBinFileIO.h"
 #include <pcl/visualization/cloud_viewer.h>
#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/common/eigen.h>
#include <algorithm>


using namespace std;
using namespace pcl;
bool ComparePoint (PointXYZ i, PointXYZ j)
{
  return (i.z<j.z);  
}

int main()
{    
	//... populate cloud
	FILE* file = fopen("test.bin","rb");
	int frames, ncols, nrows;
	ReadDepthMapBinFileHeader(file,frames,ncols,nrows);
	cout<<frames<<" "<<ncols<<" "<<nrows<<endl;
	CDepthMap map;
	map.SetSize(ncols, nrows); 
	int frameCount = 0;
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	cv::Mat depthMat(cv::Size(ncols,nrows), CV_8UC1);
	//cv::namedWindow("test");
    int numForegroundPoints= 80;
    int numForegroundPointsValidation= 400;    
	while (frameCount<frames)
	{
      ReadDepthMapBinFileNextFrame(file,ncols,nrows,map);
      frameCount++;
       if (frameCount<100)
          continue;
      map.convertToChar(depthMat.data);		
      PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
      cloud->points.resize (nrows*ncols);
      int count=0;
      int center_X, center_Y, center_Z;
      
      for (size_t i=0;i<nrows;++i)  
        for (size_t j=0;j<ncols;++j)
        {
          cloud->points[count].x=-j;
          cloud->points[count].y=-i;
          cloud->points[count].z=map.GetItem(i,j);
          if (cloud->points[count].z==0)
            cloud->points[count].z = 10000;				
          				// if (map.GetItem(i,j)!=0)
          				// 	cout<<i<<" "<<j<<"\n";                  
          count++;
        }
            nth_element(cloud->points.begin(), cloud->points.begin()+numForegroundPointsValidation,cloud->points.end(), ComparePoint);

      nth_element(cloud->points.begin(), cloud->points.begin()+numForegroundPoints,cloud->points.end(), ComparePoint);
      cout<<cloud->points[numForegroundPointsValidation].z-cloud->points[numForegroundPoints].z<<endl;      
      for (int i=0;i<numForegroundPoints;++i)
        depthMat.at<uint8_t>(-cloud->points[i].y, -cloud->points[i].x) = 255;

      cv::imwrite("test.jpg",depthMat);      
        viewer.showCloud (cloud,"cloud");
			char ch;
			cin>>ch;
			
	}
	
	
    
	return 1;
}
