#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include "../libkvconfig/KVConfig.h"

/** ¼ì²é upperbody ?
 */
class objdet
{
	KVConfig *cfg_;
	cv::CascadeClassifier det_faces_;
	bool loaded_;
	int debug2_;

public:
	objdet(KVConfig *cfg);
	virtual ~objdet(void);
	
	bool loaded() const { return loaded_; }
	bool has_faces(const cv::Mat &img, std::vector<cv::Rect> &faces);
};
