#pragma once

#include <opencv2/opencv1.hpp>
#include "../libkvconfig/KVConfig.h"

/** 尝试利用肤色阈值，将输入图像的非肤色部分进行屏蔽

		根据经验值：hsv 中，人的肤色 H = [7, 23]

 */
class SkinMask
{
	KVConfig *cfg_;
	cv::Mat ker_, ker2_;	// 腐蚀膨胀.
	int skin_thres_low_, skin_thres_high_;	// 肤色阈值上下限
	int hair_thres_high_;

public:
	SkinMask(KVConfig *cfg);
	~SkinMask(void);

	std::vector<std::vector<cv::Point> > find_skin_contours(const cv::Mat &origin);
	std::vector<std::vector<cv::Point> > find_hair_contours(const cv::Mat &origin);
};
