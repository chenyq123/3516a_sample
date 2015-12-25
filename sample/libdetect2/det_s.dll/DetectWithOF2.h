#pragma once
#include "detect.h"
#include <deque>
#include "objdet.h"

/** 首先帧差，然后对帧差后的所有点选择特征点，对所有特征点进行跟踪，然后进行聚合 ...
 */
class DetectWithOF2 : public Detect
{
	/** 跟踪目标，相当于一堆聚合的特征点，这些特征点距离比较近，运动方向一致
		当这些目标持续一段时间“不动”后，就认为目标停止了（消失），跟踪结束
	 */
	class Tracked
	{
		DetectWithOF2 *self_;
		cv::Rect brc_;
		std::vector<cv::Point2f> features_;	// 原始特征点 ...
		std::vector<double> pt_stamp_;	// 对应 features_ 最后更新的时间
		bool moving_;

		// 所有帧的跟踪点
		std::deque<std::vector<cv::Point2f>> frame_trackings_;	// 每层点数都相同 ...

	public:
		Tracked(DetectWithOF2 *self);

		// 在 position 位置选择特征点 ...
		bool init(const cv::Mat &gray, const cv::Rect &position);

		// 目标最新跟踪的位置 ...
		cv::Rect brc() const { return brc_; }

		void track(cv::Mat &g0, cv::Mat &g1);	// 跟踪

		// 
		bool stopped();

		// 返回跟踪方向
		Dir dir() const;

		// 返回运动距离，包括平均，最大，最小
		void distance(double &mean_dis, double &max_dis, double &min_dis) const;

		// 画出历史轨迹，每个点的 ...
		void draw_history(cv::Mat &origin);

	private:
		void remove_pt_from_history(int n);	// 删除所有帧的n点，这样保证了每帧保存的都是相同点数
	};
	friend class Tracked;

	typedef std::vector<Tracked> TRACKEDS;
	TRACKEDS trackings_;	// 正在被跟踪的对象

	bool od_;
	cv::CascadeClassifier cc_;

public:
	DetectWithOF2(KVConfig *cfg);
	~DetectWithOF2(void);

private:
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);

	// 从帧差返回的矩形中，删除正在跟踪的位置 ...
	std::vector<cv::Rect> get_new_position(const std::vector<cv::Rect> &all_diff_brcs);

	// 在 roi 内检测 body，返回的 bodies 转换为 origin 坐标
	bool detect_body(const cv::Mat &origin, const cv::Rect &roi, std::vector<cv::Rect> &bodies);

	// 根据所有 trackings_，再次合并，将相同方向运动的目标合并 ..
};
