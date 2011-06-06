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

#ifndef _DEPTH_MAP_BIN_FILE_IO_H
#define _DEPTH_MAP_BIN_FILE_IO_H

#include "DepthMap.h"

int ReadDepthMapBinFileHeader(FILE * fp, int &retNumFrames, int &retNCols, int &retNRows);

//the caller needs to allocate space for <retDepthMap>
int ReadDepthMapBinFileNextFrame(FILE * fp, int numCols, int numRows, CDepthMap & retDepthMap);

//<fp> must be opened with "wb"
int WriteDepthMapBinFileHeader(FILE * fp, int nFrames, int nCols, int nRows);

//<fp> must be opened with "wb"
int WriteDepthMapBinFileNextFrame(FILE * fp, const CDepthMap & depthMap);

#endif