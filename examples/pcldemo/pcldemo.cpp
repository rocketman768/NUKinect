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
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace pcl;
int main()
{    
	//... populate cloud
	FILE* file = fopen("test.bin","rb");
	int frames, ncols, nrows;
	ReadDepthMapBinFileHeader(file,frames,ncols,nrows);
	
	CDepthMap map;	
	int frameCount = 0;
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	cv::Mat depthMat(cv::Size(ncols,nrows), CV_8UC1), depthf;    
	cv::namedWindow("test");
	int boxSize =15;
	float scale = 6;
    map.SetSize(ncols, nrows);
	while (frameCount<frames)
	{         
		ReadDepthMapBinFileNextFrame(file,ncols,nrows,map);
        cout<<frames<<" "<<ncols<<" "<<nrows<<endl;
		frameCount++;
		if (frameCount<10)
			continue;
		map.convertToChar(depthMat.data);
        depthMat.convertTo(depthf, CV_8UC1, 255/255);
        cv::Mat depthColor;
       cv::cvtColor(depthf,depthColor,CV_GRAY2RGB);
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
		cloud->points.resize (nrows*ncols);
		int count=0;
		int center_X, center_Y, center_Z;

		for (size_t i=0;i<nrows;++i)  
			for (size_t j=0;j<ncols;++j)
			{
              if (map.GetItem(i,j)>850)
              {
                continue;                
              }
              else
              {                
				cloud->points[count].x=-j;
				cloud->points[count].y=-i;
				cloud->points[count].z=-map.GetItem(i,j);
              }
              count++;              
                if (cloud->points[count].z!=0)
                  cloud->points[count].z = cloud->points[count].z - 200;                
				center_Z = map.GetItem(i,j);
				center_Y =i;
				center_X = j;
				if (center_X<boxSize||center_X>=ncols-boxSize||center_Y<boxSize||center_Y>=nrows-boxSize)
                {
                  
					continue;
                }
                
				Eigen::Matrix3f covariance_matrix;
				covariance_matrix.setZero();
				for (int x = center_X-boxSize; x<center_X+boxSize; x++)
					for (int y = center_Y-boxSize;y<center_Y+boxSize;y++)
					{
                        int diff_z = map.GetItem(y,x) - center_Z;;
                        if  (abs(map.GetItem(y,x)-center_Z)>boxSize)
                      	  continue;
						int diff_x = x-center_X;
						int diff_y = y-center_Y;

						covariance_matrix(0,0) +=diff_x*diff_x;
						covariance_matrix(0,1) +=diff_x*diff_y;
						covariance_matrix(0,2) +=diff_x*diff_z;
						covariance_matrix(1,0) +=diff_x*diff_y;
						covariance_matrix(1,1) +=diff_y*diff_y;
						covariance_matrix(1,2) +=diff_y*diff_z;
						covariance_matrix(2,0) +=diff_x*diff_z;
						covariance_matrix(2,1) +=diff_y*diff_z;
						covariance_matrix(2,2) +=diff_z*diff_z;						
					}
					EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
					EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance_matrix);
                    eigen_values = eigensolver.eigenvalues();                   						
                    
					if (abs(log((eigen_values(2,0) + 0.1)/(eigen_values(1,0)+0.1)))>log(scale)&&eigen_values(0,0)!=0)
					{
                        cout<<center_X<<" "<<center_Y<<" "<<eigen_values(0,0)<<" "<<eigen_values(1,0)<<" "<<eigen_values(2,0)<<endl;						
						//depthMat.at<uint8_t>(center_Y,center_X) = 255;
                        
                        //depthColor.at<cv::Scalar>(center_Y,center_X) = cv::Scalar(255,0,0);
                        cv::circle(depthColor,cv::Point2f(center_X,center_Y),2,cv::Scalar(0,255,0));
						
					}					
				
// 				if (map.GetItem(i,j)!=0)
// 					cout<<i<<" "<<j<<"\n";
			}

            cv::imwrite("test.jpg",depthColor);
           viewer.showCloud (cloud,"cloud");
			char ch;
			//cin>>ch;
            cout<<frameCount<<endl;
            
			
	}
	
	while (1)
    {
    }
    
    
	return 1;
}
