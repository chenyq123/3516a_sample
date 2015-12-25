#pragma once
#include "detect.h"

/** 基于稠密光流，采集两帧之间距离大于 2 的点，根据方向，进行融合
 */
class DetectWithOF3 : public Detect
{
	cv::ocl::FarnebackOpticalFlow of_;	//
	double fb_pyrscale_, fb_polysigma_;
	int fb_levels_, fb_winsize_, fb_iters_, fb_polyn_;
	cv::ocl::oclMat d_gray_prev_, d_gray_curr_, d_distance_, d_angle_, d_dx_, d_dy_;
	bool d_first_;

	double threshold_optical_flow_;	// 两帧之间的光流阈值，缺省 3.0
	
	cv::Mat ker_erode_, ker_dilate_;

	int up_angle_; // 向上的角度范围，默认 110

	int merge_mode_;	// 合并模式，1: 与最后框合并，2: 与整个历史外接框合并

	double delay_;	// 判断目标活动的延迟，就是说，等 delay_ 秒后，开始分析 Target 的行为，缺省 0.3

	int debug_lmax_, debug_lmin_;
	double debug_max_dis_;

	double area_factor_ab_[2];	// 面积变化使用直线，求 y=ax+b
	double area_bottom_y_;	// 后排，中排，感觉目标面积变化比较均匀，但前排需要很大！！！该参数决定前排的分割位置，缺省 0.667
	double area_bottom_max_, area_max_, area_min_;	// 覆盖 Detect.h 中的 ...其中 area_max_, area_min_ 描述中后排的 Target 面积,
	double area_bottom_min_;						// area_bottom_max_ area_bottom_min_, 描述最前排 ...

	// 四个方向的统计
	struct DirCnt
	{
		int left, right, up, down;
		double dis_left, dis_right, dis_up, dis_down;	// 每个方向的距离累计和
	};

	// 记录一个位置，一段时间，
	struct Target
	{
		double stamp_first, stamp_last;	// 创建时间，最后更新时间

		cv::Rect rc;	// 最大外接矩形，合并 ...

		std::deque<cv::Rect> hist_rcs;	// 记录矩形
		std::deque<int> hist_dirs;	// 历史方向
		std::deque<cv::Mat> hist_of_dx;	// 矩形内的光流，dx,dy 格式
		std::deque<cv::Mat> hist_of_dy;
		std::deque<DirCnt> hist_dir_cnt;	// 每帧中的四个方向的统计
	};

	typedef std::vector<Target> TARGETS;
	TARGETS targets_;	//

public:
	DetectWithOF3(KVConfig *cfg);
	~DetectWithOF3(void);

private:
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);

private:
	void calc_optical_flow(const cv::Mat &p0, const cv::Mat &p1, cv::Mat &dis, cv::Mat &ang, cv::Mat &dx, cv::Mat &dy);
	void show_optical_flow(cv::Mat &origin, const cv::Mat &dis, const cv::Mat &ang);
	void get_motion_rects(const cv::Mat &dis, const cv::Mat &ang, std::vector<cv::Rect> &rcs, std::vector<int> &dirs,
		std::vector<DirCnt> &dircnts);
	Dir get_roi_property(const cv::Rect &roi, const cv::Mat &dis, const cv::Mat &dis_bin, const cv::Mat &ang, DirCnt &dc);
	inline Dir ang2dir(float ang) const;
	void merge_targets(const std::vector<cv::Rect> &motion_rcs, const std::vector<int> &dirs, 
		const std::vector<DirCnt> &dircnts, const cv::Mat &dx, const cv::Mat &dy);
	void merge_targets();	// 
	bool analyze_target_hist(const Target &target, Dir &dir, cv::Rect &rc) const;	// 如果明确知道是某个动作，则返回true，并返回dir和活动区域
	bool calc_area(int y, double &area_min, double &area_max) const;	// 根据y轴，计算面积系数 ...，返回是否 bottom 
};
