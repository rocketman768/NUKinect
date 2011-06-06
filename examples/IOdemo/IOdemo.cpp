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
#include <vector>
#include <stdio.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "KinectIO.h"
#include <unistd.h>
#include "DepthMapBinFileIO.h"
using namespace std;
bool beginCapture = false;


 int main(int argc, char *argv[])
{  
	uint32_t lastTimestamp = 0;
	boost::shared_array<uint8_t> depth;	
	FILE* file = fopen("test.bin","w");
    vector<CDepthMap*> depthMaps;
    cv::namedWindow("new");
    cv::Mat depthMat(cv::Size(640,480),CV_16UC1),depthf;        
	while(1)
	{
      //std::cout << "Getting one frame...";
        uint32_t tmp = lastTimestamp;
        while(tmp==lastTimestamp)
        {
          KinectIO::instance().kinect().getDepth(lastTimestamp, depth);
          usleep(15e3); // Sleep 15 ms.
        }
		//std::cout << lastTimestamp << "Finished.\n";
        depthMat.data = (uchar*) depth.get();
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        cv::imshow("new",depthf);
        //sleep(1);        
        char guiInput  = cv::waitKey(5);       
        if (guiInput =='B')
        {
          cout<<"beginCapture"<<endl;
          beginCapture = true;          
        }
        else if (guiInput =='Q')
        {
          cout<<"endCapture"<<endl;          
          break;          
        }
        
        if (beginCapture)
        {          
          CDepthMap* map = new CDepthMap(640,480,(uchar*) depth.get());
          std::cout << "writing one frame..."<<endl;
          depthMaps.push_back(map);          
        }
        
	}
    WriteDepthMapBinFileHeader(file,depthMaps.size(),640,480);
    for (int i=0;i<depthMaps.size();++i)
    {
      WriteDepthMapBinFileNextFrame(file,*depthMaps[i]);
      delete depthMaps[i];      
    }
    
	fclose(file);
	file = fopen("test.bin","rb");
	int frames, ncols, nrows;
	ReadDepthMapBinFileHeader(file,frames,ncols,nrows);
	cout<<frames<<" "<<ncols<<" "<<nrows<<endl;
    cv::Mat depthMat2(cv::Size(ncols,nrows),CV_8UC1);
	for (int i=0;i<frames+1;++i)
	{
        
		CDepthMap map;
		map.SetSize(ncols, nrows); 
		ReadDepthMapBinFileNextFrame(file,ncols,nrows,map);
		map.convertToChar(depthMat2.data);
		char filename[255];
		sprintf(filename,"image%d.jpg",i);
		cv::imwrite(filename,depthMat2);
	}
	fclose(file);
	
	
	
	return 0;
}
