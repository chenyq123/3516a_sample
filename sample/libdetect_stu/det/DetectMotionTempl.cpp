#include "DetectMotionTempl.h"

DetectMotionTempl::DetectMotionTempl(KVConfig *cfg)
	: Detect(cfg)
{
}

DetectMotionTempl::~DetectMotionTempl(void)
{
}

std::vector<cv::Rect> DetectMotionTempl::detect0(cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs)
{
	dirs.clear();
	std::vector<cv::Rect> rcs;

	/** TODO: 通过帧差法得到变化位置，再跟踪这些位置 ?
	 */

	return rcs;
}
