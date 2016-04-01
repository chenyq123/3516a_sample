#pragma once
#include "detect.h"
#include "ct/CompressiveTracker.h"

/** 基于ct的跟踪

	1. 帧差，找到较大活动区域 MotionRect
	2. 在 MotionRect 内找所有“头肩”；
	3. 每个头肩作为目标，初始化，利用 CT 进行跟踪；
	4. 当 CT 目标很小活动时，则认为目标站住，此时根据目标运动历史，判断目标上下左右的活动;
 */
class DetectWithCT : public Detect
{
	/** 被 CT 跟踪的目标
	 */
	struct Target
	{
		static Target *new_target(const cv::Rect &header, double stamp)
		{
			Target *t = new Target;
			t->hist_rcs.push_back(header);
			t->stamp_begin = stamp;
		}

		double stamp_begin, stamp_last_updated;
		std::deque<cv::Rect> hist_rcs;	// 历史位置，最多保存10帧就足够了..
		
		cv::Rect rc() const { return hist_rcs.back(); }
	};

	typedef std::list<Target> TARGETS;
	TARGETS targets_;			// 当前正在跟踪的目标

	cv::Mat ker_erode_, ker_dilate_;

	cv::CascadeClassifier cc_;

public:
	DetectWithCT(KVConfig *cfg);
	~DetectWithCT(void);

private:
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);

	// 通过帧差，融合，得到运动区域
	RECTS get_motion_rects_using_diff(const cv::Mat &prev, const cv::Mat &curr);

	// 查找"头肩“
	RECTS get_headers(const cv::Mat &range);
};
