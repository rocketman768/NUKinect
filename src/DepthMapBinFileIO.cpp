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

#include <stdlib.h>
#include <stdio.h>
#include "DepthMapBinFileIO.h"

int ReadDepthMapBinFileHeader(FILE * fp, int &retNumFrames, int &retNCols, int &retNRows)
{
	if(fp == NULL)
		return 0;


	fread(&retNumFrames, 4, 1, fp); //read 4 bytes 
	fread(&retNCols, 4, 1, fp);
	fread(&retNRows, 4, 1, fp);
	//fscanf(fp, "%i", &retNumFrames);
	//fscanf(fp, "%i", &retWidth);
	//fscanf(fp, "%i", &retHeight);

	return 1;
}

//the caller needs to allocate space for <retDepthMap>
int ReadDepthMapBinFileNextFrame(FILE * fp, int numCols, int numRows, CDepthMap & retDepthMap)
{
	int r,c;
	//for(h=0; h<height; h++) //for each row
	int * tempRow = new int[numCols];
	for(r=0;r<numRows;r++) //one row at a time
	{
		fread(tempRow, 4, numCols, fp);
		for(c=0; c<numCols; c++) //for each colume	
		{
			//int temp=0;
			//fread(&temp, 4, 1, fp);
			//retDepthMap.SetItem(r,c,temp);
			int temp = tempRow[c];
			retDepthMap.SetItem(r,c,(float) temp);
		}
	}
	delete[] tempRow;
	tempRow = NULL;
	return 1;	
}

//<fp> must be opened with "wb"
int WriteDepthMapBinFileHeader(FILE * fp, int nFrames, int nCols, int nRows)
{
	if(fp == NULL)
		return 0;


	fwrite(&nFrames, 4, 1, fp); //read 4 bytes 
	fwrite(&nCols, 4, 1, fp);
	fwrite(&nRows, 4, 1, fp);
	//fscanf(fp, "%i", &retNumFrames);
	//fscanf(fp, "%i", &retWidth);
	//fscanf(fp, "%i", &retHeight);

	return 1;
}

//<fp> must be opened with "wb"
int WriteDepthMapBinFileNextFrame(FILE * fp, const CDepthMap & depthMap)
{
	int numCols = depthMap.GetNCols();
	int numRows = depthMap.GetNRows();
	
	int r,c;
	//for(h=0; h<height; h++) //for each row
	int * tempRow = new int[numCols];
	for(r=0;r<numRows;r++) //one row at a time
	{
		for(c=0; c<numCols; c++) //for each colume
		{
			int temp = (int) (depthMap.GetItem(r,c));
			tempRow[c] = temp;
		}
		fwrite(tempRow, 4, numCols, fp);
	}
	delete[] tempRow;
	tempRow = NULL;
	return 1;	
}
