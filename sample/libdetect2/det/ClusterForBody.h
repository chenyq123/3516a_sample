#pragma once

#include "detect.h"

/** 希望将较近的轮廓合并
 */
class ClusterForBody
{
	KVConfig *cfg_;

public:
	ClusterForBody(KVConfig *cfg);
    std::vector<std::vector<cv::Point> > merge(const std::vector<std::vector<cv::Point> > &contours);
	~ClusterForBody(void);
};
