#include <iostream>
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


using namespace std;
using namespace pcl;
int main()
{
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	//... populate cloud
	FILE* file = fopen("a01_s01_e01_sdepth.bin","rb");
	int frames, ncols, nrows;
	ReadDepthMapBinFileHeader(file,frames,ncols,nrows);
	cout<<frames<<" "<<ncols<<" "<<nrows<<endl;
	CDepthMap map;
	map.SetSize(ncols, nrows); 
	int frameCount = 0;
	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	
	while (frameCount<frames)
	{
		ReadDepthMapBinFileNextFrame(file,ncols,nrows,map);
		cloud->points.resize (nrows*ncols);
		int count=0;
		for (size_t i=0;i<nrows;++i)  
			for (size_t j=0;j<ncols;++j)
			{
				cloud->points[count].x=-j;
				cloud->points[count].y=-i;
				cloud->points[count].z=-map.GetItem(i,j);
				count++;
			}			
			pcl::ModelCoefficients coefficients;
			pcl::PointIndices inliers;
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold (0.01);
			seg.setInputCloud (cloud->makeShared());
			seg.segment (inliers, coefficients);
			std::cout << "Model coefficients: " << coefficients.values[0] << " "
				<< coefficients.values[1] << " "
				<< coefficients.values[2] << " "
				<< coefficients.values[3] << std::endl;

			std::cout << "Model inliers: " << inliers.indices.size () << std::endl;
			// for (size_t i = 0; i < inliers.indices.size (); ++i)
			// 	std::cout << inliers.indices[i] << "    " << cloud->points[inliers.indices[i]].x << " "
			// 	<< cloud->points[inliers.indices[i]].y << " "
			// 	<< cloud->points[inliers.indices[i]].z << std::endl;
			viewer.showCloud (cloud);
			char ch;
			cin>>ch;
			frameCount++;
	}
	
	
    
	return 1;
}
