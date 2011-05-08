#include <math.h>
#include "DepthMap.h"
#include <cstring>
#include <stdint.h>

CDepthMap::CDepthMap(void)
{
	m_ncols = 0;
	m_nrows = 0;
	m_depthVals = NULL;
}

CDepthMap::CDepthMap( int ncols, int nrows, uchar* raw_data )
{
	m_nrows = nrows;
	m_ncols = ncols;

	m_depthVals = new float[nrows*ncols];	
	uint16_t *data  = (uint16_t*)raw_data;
	for (int i=0;i<nrows;i++)
		for (int j=0;j<ncols;++j)
		{
			m_depthVals[i*ncols+j] = data[i*ncols+j];
		}
}

CDepthMap::~CDepthMap(void)
{
	if(m_depthVals)
	{
		delete[] m_depthVals;
		m_depthVals = NULL;
	}
}

void CDepthMap::SetSize(int ncols, int nrows) //allocate space
{
	if(m_depthVals != NULL)
	{
		delete[] m_depthVals;
		m_depthVals = NULL;
	}
	m_nrows = nrows;
	m_ncols = ncols;
	
	m_depthVals = new float[nrows*ncols];
	return;
}

void CDepthMap::SetItem(int r, int c, float val)
{
	m_depthVals[r * m_ncols + c] = val;
}

float CDepthMap::GetItem(int r, int c) const
{
	return m_depthVals[r * m_ncols + c] ;
}

//scale the depth values
void CDepthMap::Scale(float s)
{
	int r,c;
	for(r=0; r<GetNRows(); r++)
	{
		for(c=0; c<GetNCols(); c++)
		{
			float temp = GetItem(r,c);
			SetItem(r,c, temp*s);
		}
	}
}

//scale the dimensions:
//ncols = floor(scaleFactor * ncols), nrows = floor(scaleFactor * nrows)
void CDepthMap::ScaleSizeNN(float scaleFactor)
{
	//buffer the original depthmap
	CDepthMap bufDepthMap;
	CopyDepthMapTo(bufDepthMap);

	int new_ncols = floor(m_ncols * scaleFactor);
	int new_nrows = floor(m_nrows * scaleFactor);

	SetSize(new_ncols, new_nrows);

	int r,c;
	for(r=0; r<GetNRows(); r++)
	{
		for(c=0; c<GetNCols(); c++)
		{
			int origR = floor(r/scaleFactor);
			int origC = floor(c/scaleFactor);
			float origVal = bufDepthMap.GetItem(origR, origC);
			SetItem(r,c, origVal);
		}
	}
}

void CDepthMap::CopyDepthMapTo(CDepthMap & dstMap) const
{
	dstMap.SetSize(m_ncols, m_nrows);
	int r,c;
	for(r=0; r<GetNRows(); r++)
	{
		for(c=0; c<GetNCols(); c++)
		{
			float temp = GetItem(r,c);
			dstMap.SetItem(r,c, temp);
		}
	}
}

int CDepthMap::NumberOfNonZeroPoints() const
{
	int count=0;
	int r,c;
	for(r=0; r<GetNRows(); r++)
	{
		for(c=0; c<GetNCols(); c++)
		{
			float temp = GetItem(r,c);
			if(temp != 0)
			{
				count++;
			}
		}
	}
	return count;
}

float CDepthMap::AvgNonZeroDepth() const
{
	int count=0;
	float sum = 0;
	int r,c;
	for(r=0; r<GetNRows(); r++)
	{
		for(c=0; c<GetNCols(); c++)
		{
			float temp = GetItem(r,c);
			if(temp != 0)
			{
				count++;
				sum += temp;
			}
		}
	}
	if(count == 0)
		return 0;
	else
		return sum/count;
}

void CDepthMap::convertToChar( uchar* dst ) const
{
	for (int i=0;i<m_nrows;i++)
		for (int j=0;j<m_ncols;++j)
		{
			dst[i*m_ncols+j] = (uint8_t)(m_depthVals[i*m_ncols+j]*255.0/2048.0);
		}
}

