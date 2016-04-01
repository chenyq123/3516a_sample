#pragma once

#define _USE_MATH_DEFINES

#include "Detect.h"
#include "DiffMotion.h"
#include "Target.h"

/** 使用稀疏光流记录连续活动，当活动停止后，检查活动的轨迹，返回有效的活动 ...
 */
class DetectWithOf : public Detect
{
	DiffMotion dm_;
	std::vector<Target> targets_;	// 活动目标 ...
	std::vector<Target> long_timed_targets_;	// 测试：长时间跟踪目标，看看效果 ...

	// 学生区最后排，最前排的位置，在此之间，使用线性拟合 ...
	float far_y_, near_y_;	// 根据 y 轴 ...
	
	float far_width_, near_width_;	// 帧差矩形的中心y的宽度阈值，如果小于该值，则认为帧差矩形太小了 ...
	double a_width_, b_width_;		// 宽度的直线方程系数 ...
	
	float far_dis_[4], near_dis_[4];	// 四个方向的距离阈值 ...
	double a_dis_[4], b_dis_[4];	// 
	
	int up_angle_;		// 

	int next_tid_;		// Target ID

public:
	explicit DetectWithOf(KVConfig *cfg);
	~DetectWithOf();

	virtual void reset();

private:
	virtual void detect(RCS &motions, std::vector<Dir> &dirs);

	std::vector<Target>::iterator find_matched_target(const cv::Rect &rc)	// 找到与 rc 相交的 target ...
	{
		for (std::vector<Target>::iterator it = targets_.begin(); it != targets_.end(); ++it) {
			if (it->is_crossed(rc)) {
				return it;
			}
		}
		return targets_.end();
	}

	bool too_small(const cv::Rect &rc) const;

	enum {
		E_OK,
		E_NotEnoughLayers,
		E_NotEnoughPaths,
		E_NotEnoughDistance,
	};

	/// 评估target的活动，如果有效，则返回 OK
	int estimate(const Target &target, cv::Rect &pos, Dir &dir) const;

	/// 根据两点求直线的 a,b 系数
	inline void polyfit_line(const cv::Point2f &p1, const cv::Point2f &p2, double &a, double &b) const
	{
		cv::Point2f p = p2;
		if (p.x == p1.x) {	 // NOTE: 防止竖直直线 ...
			p.x += (float)0.01;
		}

		a = (p1.y - p.y) / (p1.x - p.x);
		b = p.y - a * p.x;
	}

	/// 根据两点，求角度，从 x 轴顺时针旋转一周，[0..360) 
	inline double calc_angle(const cv::Point2f &p1, const cv::Point2f &p2) const
	{
		double a = atan2l(-(p2.y - p1.y), p2.x - p1.x) * 180.0 / M_PI;
		if (a > 0) {  // 第一二象限 ..
			a = 360.0 - a;
		}
		else {	// 三四象限 ..
			a = -a;	
		}
		
		return a;
	}
};
