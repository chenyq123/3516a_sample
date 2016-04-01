#pragma once

#include "detect.h"

class DetectMotionTempl : public Detect
{
public:
	DetectMotionTempl(KVConfig *cfg);
	~DetectMotionTempl(void);

private:
	virtual std::vector<cv::Rect> detect0(cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);
};

