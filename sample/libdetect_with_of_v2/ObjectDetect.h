#pragma once

#include <opencv2/opencv1.hpp>
#include "KVConfig.h"

class ObjectDetect
{
	cv::CascadeClassifier cc_;
	bool loaded_;
	KVConfig *cfg_;

public:
	explicit ObjectDetect(KVConfig *cfg);
	~ObjectDetect();

	bool reload();

	bool enabled() const
	{
		return loaded_;
	}

	std::vector<cv::Rect> detect(const cv::Mat &gray, const cv::Rect &roi);
};
