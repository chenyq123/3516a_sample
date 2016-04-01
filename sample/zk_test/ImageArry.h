#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

#define ARRAYNUM 10
class CImageArry
{
private:
	bool mask_on;
	int mHeigth;
	int mWidth;
	int mIndex;
	long mAddImageCount;
public:
	CImageArry(void);
	~CImageArry(void);

	cv::Mat imagecoo[ARRAYNUM];//最原始图像
	cv::Mat imagegraycoo[ARRAYNUM];//原始图像灰度图

	cv::Mat imagecolor[ARRAYNUM];//缩小后的彩图
	cv::Mat imagegray[ARRAYNUM];//缩小后的灰度图

	cv::Mat imagefg[ARRAYNUM];//背景建模后获取的前景图像
	cv::Mat imageflowup[ARRAYNUM];//站起来的光流(目的地)
	cv::Mat imageflowdown[ARRAYNUM];//坐下来的光流
	cv::Mat imageflowupfrom[ARRAYNUM];//站起来的光流(源地)

	cv::Mat imageflowleft[ARRAYNUM];
	cv::Mat imageflowright[ARRAYNUM];
	
	cv::Mat imagecolorflow[ARRAYNUM];//缩小后的彩图

	cv::Mat imageflowupdown[ARRAYNUM];//累计幁的上下光流，单幁光流点 膨胀 加进来，减去最早那幁的光流，试一下 只记录上升  和 上升下降混合 的情况

	cv::Mat imageflowupdowncur[ARRAYNUM];

	cv::Mat maskcolor;

	int getHeight();
	int getWidth();

	bool AddNewImage(cv::Mat icolorimage);
	bool SetMask(std::vector<cv::Point> maskpoints);
	int newindex();

	int preindex(int prenum=0);//0 当前最新的，1 前一个，2 前2个，...
};

