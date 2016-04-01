
/************************************************************************
* File:	CompressiveTracker.h
* Brief: C++ demo for paper: Kaihua Zhang, Lei Zhang, Ming-Hsuan Yang,"Real-Time Compressive Tracking," ECCV 2012.
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2012/08/03
* History:
* Revised by Kaihua Zhang on 14/8/2012, 23/8/2012
* Email: zhkhua@gmail.com
* Homepage: http://www4.comp.polyu.edu.hk/~cskhzhang/
* Project Website: http://www4.comp.polyu.edu.hk/~cslzhang/CT/CT.htm
************************************************************************/
//这是一个比较常用的C/C++杂注，只要在头文件的最开始加入这条杂注，就能够保证头文件只被插入和编译一次
#pragma once

#include <opencv2/opencv1.hpp>
#include <vector>

using std::vector;
using namespace cv;
//---------------------------------------------------
class CompressiveTracker
{
public:
	CompressiveTracker(void);
	~CompressiveTracker(void);

private:
	int featureMinNumRect;
	int featureMaxNumRect;
	int featureNum;
	vector<vector<cv::Rect> > features;//每个box的harr特征个数（也就是弱分类器个数）
	vector<vector<float> > featuresWeight;
	int rOuterPositive;//在离上一帧跟踪到的目标位置的距离小于rOuterPositive的范围内采集 正样本
	vector<cv::Rect> samplePositiveBox;//采集的正样本box集
	vector<cv::Rect> sampleNegativeBox;//采集的负样本box集
	int rSearchWindow;//扫描窗口的大小，或者说检测box的大小
	Mat imageIntegral;//图像的积分图
	Mat samplePositiveFeatureValue;//采集的正样本的harr特征值
	Mat sampleNegativeFeatureValue;//采集的负样本的harr特征值
	//对每个样本z（m维向量），它的低维表示是v（n维向量，n远小于m）。假定v中的各元素是独立分布的。
	//假定在分类器H(v)中的条件概率p(vi|y=1)和p(vi|y=0)属于高斯分布，并且可以用以下四个参数来描述：
	//分别是描述正负样本的高斯分布的均值u和方差sigma
	vector<float> muPositive;
	vector<float> sigmaPositive;
	vector<float> muNegative;
	vector<float> sigmaNegative;
	float learnRate;//学习速率，控制分类器参数更新的步长
	vector<cv::Rect> detectBox;//需要检测的box
	Mat detectFeatureValue;
	RNG rng;//随机数

private:
	void HaarFeature(cv::Rect& _objectBox, int _numFeature);
	void sampleRect(Mat& _image, cv::Rect& _objectBox, float _rInner, float _rOuter, int _maxSampleNum, vector<cv::Rect>& _sampleBox);
	void sampleRect(Mat& _image, cv::Rect& _objectBox, float _srw, vector<cv::Rect>& _sampleBox);
	void getFeatureValue(Mat& _imageIntegral, vector<cv::Rect>& _sampleBox, Mat& _sampleFeatureValue);
	void classifierUpdate(Mat& _sampleFeatureValue, vector<float>& _mu, vector<float>& _sigma, float _learnRate);
	void radioClassifier(vector<float>& _muPos, vector<float>& _sigmaPos, vector<float>& _muNeg, vector<float>& _sigmaNeg,
		Mat& _sampleFeatureValue, float& _radioMax, int& _radioMaxIndex);
public:
	//void processFrame(Mat& _frame, cv::Rect& _objectBox);
	cv::Rect processFrame(Mat& _frame, cv::Rect objectBox);
	void init(Mat& _frame, cv::Rect& _objectBox);
};
