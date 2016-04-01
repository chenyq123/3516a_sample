#include "ClusterForBody.h"

ClusterForBody::ClusterForBody(KVConfig *cfg)
	: cfg_(cfg)
{
}

ClusterForBody::~ClusterForBody(void)
{
}

std::vector<std::vector<cv::Point> > ClusterForBody::merge(const std::vector<std::vector<cv::Point> > &contours)
{
	std::vector<std::vector<cv::Point> > merged;
	return merged;
}
