
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
	FILE* file = fopen("a01_s01_e02_sdepth.bin","rb");
	int frames, ncols, nrows;
	ReadDepthMapBinFileHeader(file,frames,ncols,nrows);
	cout<<frames<<" "<<ncols<<" "<<nrows<<endl;
	CDepthMap map;
	map.SetSize(ncols, nrows); 
	int frameCount = 0;
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	cv::Mat depthMat(cv::Size(320,240), CV_8UC1);
	cv::namedWindow("test");
	int boxSize =30;
	float scale = 5;
	while (frameCount<frames)
	{
		ReadDepthMapBinFileNextFrame(file,ncols,nrows,map);
		frameCount++;
		if (frameCount<20)
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
				cloud->points[count].z=-map.GetItem(i,j);
                if (cloud->points[count].z!=0)
                  cloud->points[count].z = cloud->points[count].z - 200;                
				center_Z = map.GetItem(i,j);
				center_Y = i;
				center_X = j;
				if (center_X<boxSize||center_X>=ncols-boxSize||center_Y<boxSize||center_Y>=nrows-boxSize)
					continue;
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
						depthMat.at<uint8_t>(center_Y,center_X) = 255;
						
					}
					
				
// 				if (map.GetItem(i,j)!=0)
// 					cout<<i<<" "<<j<<"\n";                  
				count++;
			}
            cv::imwrite("test.jpg",depthMat);
           viewer.showCloud (cloud,"cloud");
			char ch;
			//cin>>ch;
            cout<<frameCount<<endl;
            
			
	}
	
	
    
	return 1;
}
