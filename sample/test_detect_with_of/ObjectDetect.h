#pragma once

#include <opencv2/opencv.hpp>
#include "../libkvconfig/KVConfig.h"

class ObjectDetect
{
	cv::CascadeClassifier cc_;
	bool loaded_;

public:
	explicit ObjectDetect(KVConfig *cfg);
	~ObjectDetect();

	bool enabled() const 
	{
		return loaded_;
	}

	std::vector<cv::Rect> detect(const cv::Mat &gray, const cv::Rect &roi);
};
