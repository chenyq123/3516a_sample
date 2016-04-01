#include "ImageArry.h"
#include <sys/time.h>
#include <cstdio>

using namespace cv;
using namespace std;

unsigned long TickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

CImageArry::CImageArry(void)
{
	//default
	mask_on=false;
	mWidth =480;//320;
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
    //printf("line:%d,time=%ld\n",__LINE__,TickCount());
	int ind=newindex();
    if(mAddImageCount >= ARRAYNUM)
    {
        icolorimage.copyTo(imagecoo[ind]);
    }
    else
	    imagecoo[ind] = icolorimage.clone();
	//cv::cvtColor(icolorimage, imagegraycoo[ind], CV_BGR2GRAY);
	//imagegraycoo[ind]
    //printf("line:%d,time=%ld\n",__LINE__,TickCount());
	//resize(icolorimage, imagecolor[ind], Size(mWidth,mHeigth));
    imagecolor[ind] = icolorimage.clone();
    //printf("line:%d,time=%ld\n",__LINE__,TickCount());

	if (mask_on)
	{
		imagecolor[ind] &=  maskcolor;
	}

    //printf("line:%d,time=%ld\n",__LINE__,TickCount());
	cv::cvtColor(imagecolor[ind], imagegray[ind], CV_BGR2GRAY);//
    //printf("line:%d,time=%ld\n",__LINE__,TickCount());
	mAddImageCount++;
	return true;
}
