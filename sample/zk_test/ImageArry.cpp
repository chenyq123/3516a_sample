#include "ImageArry.h"

using namespace cv;   
using namespace std; 

CImageArry::CImageArry(void)
{
	//default
	mask_on=false;
	mWidth =540;//320;
	mHeigth = 360;//180;	
	mIndex=0;
	mAddImageCount=0;
}


CImageArry::~CImageArry(void)
{
}

int CImageArry::getHeight()
{
	return mHeigth;
}

int CImageArry::getWidth()
{
	return mWidth;
}

int CImageArry::newindex()
{
	mIndex=mIndex+1;
	if(mIndex>=ARRAYNUM)mIndex=0;

	return mIndex;
}

int CImageArry::preindex(int prenum)
{
	int i=mIndex-prenum;
	if(i<0)
		i=ARRAYNUM+i;
	return i;
}

bool CImageArry::SetMask(std::vector<cv::Point> maskpoints)
{
	//mask = cv::Mat::zeros(Size(mWidth,mHeigth),CV_8UC1);
	maskcolor = cv::Mat::zeros(Size(mWidth,mHeigth),CV_8UC3);

	int n = maskpoints.size();
	cv::Point *pts = (cv::Point*) alloca(sizeof(cv::Point) * n);
	for (int i = 0; i < n; i++) {
		pts[i] = maskpoints[i];
	}
	//cv::fillConvexPoly(mask, pts, n, cv::Scalar(255));
	cv::fillConvexPoly(maskcolor, pts, n, cv::Scalar(255,255,255));
	mask_on=true;
	return mask_on;
}

bool CImageArry::AddNewImage(cv::Mat icolorimage)
{
	int ind=newindex();
	imagecoo[ind] = icolorimage.clone();
	//cv::cvtColor(icolorimage, imagegraycoo[ind], CV_BGR2GRAY);
	//imagegraycoo[ind]
	resize(icolorimage, imagecolor[ind], Size(mWidth,mHeigth));

	if (mask_on)
	{
		imagecolor[ind] &=  maskcolor;
	}

	cv::cvtColor(imagecolor[ind], imagegray[ind], CV_BGR2GRAY);//

	mAddImageCount++;
	return true;
}