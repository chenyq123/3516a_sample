#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include "detect.h"
#include <deque>
#include <opencv2/ocl/ocl.hpp>

/** 利用稠密光流连续N帧累积得到“单向长距离移动区域”，然后检查该区域的面积
	这是基于人的形体来的：
		1. 人站立需要700ms,在这段时间内，应该是连续的单向运动；
		2. 胳膊等的挥动不会产生累积效果，因为速度和胳膊的运动方向的宽度;
		3. 人的快速晃动会导致累积抵消；

	探测头画面畸变比较严重，需要估算出前后排系数，目前采用2次多项式拟合，但感觉更像一个指数函数 ...

	进一步考虑：
		1. 貌似人 up 产生的光流累积相比其他方向比较小，可以考虑up使用更小的距离或面积阈值；
		2. 人站立时，似乎总是先有一个left或者right（与探测头的视角有关系）的光流，然后再有up，可以利用此特性

	借鉴谭工思路：
		保存数帧历史不同方向光流，总是与前几帧的历史进行合并评估

 */
class DetectWithOpticalFlow : public Detect
{
	int debug2_, debug_img3_, debug_img4_;
	bool gpu_, sum_;
	double up_rc_aspect_;	// 站立时，外接矩形的纵横比，照理说，应该 width < height * up_rc_aspect_
	cv::Mat ker_, ker2_;	// 腐蚀膨胀.
	size_t preload_frames_;	// 需要预先加载的帧数.
	typedef std::deque<cv::Mat> FIFO;
	cv::Mat sum_x_, sum_y_;	// x,y 矢量累积和.
	FIFO saved_x_, saved_y_;	// 缓冲 preload_frames 帧.
	FIFO saved_distance_;			// 缓冲的 preload_frames 的距离
	cv::Mat sum_dis_;
	double thres_dis_flipper_;	// flipper 模式下的累计距离阈值 ...

	double fb_pyrscale_, fb_polysigma_;
	int fb_levels_, fb_winsize_, fb_iters_, fb_polyn_;

	int up_angle_;	// 被认为向上的角度是多少，如 90°，60° ...
	double up_tune_factor_;		// 看起来向上的光流比其他方向小，所以向上面积阈值乘以该系数（<1），让向上更容易检测到.
	double lr_tune_factor_;		// 左右活动的阈值加大，可以防止站起来后，左右晃动的影响.
	double down_tune_factor_;

	std::vector<std::vector<cv::Point> > flipped_group_polys_;	// 用于保存翻转课堂的小组的位置

	cv::ocl::FarnebackOpticalFlow *flow_detector_fb_;

	cv::ocl::oclMat d_gray_prev_, d_gray_curr_, d_sum_x_, d_sum_y_, d_distance_, d_angle_;
	std::deque<cv::ocl::oclMat> d_saved_x_, d_saved_y_;
	bool d_first_;

	// 记录 preloaded 帧，上下左右四个方向的光流历史
	class History
	{
		int size_, pos_, width_, height_;
		cv::Mat *left_, *right_, *up_, *down_;
		cv::Mat sum_;	// 或结果 ..

		void init(cv::Mat *p)
		{
			for (int i = 0; i < size_; i++) {
				p[i] = cv::Mat::zeros(height_, width_, CV_8U);
			}
		}

		// id = 0 对应当前 pos_， id = 1 对应 pos_ 之前一个， ...
		cv::Mat &prev(cv::Mat *p, int id) const
		{
			id %= size_;
			
			if (pos_ - id < 0) {
				return p[size_ + pos_ - id];
			}
			else {
				return p[pos_ - id];
			}
		}

	public:
		History(KVConfig *cfg)
		{
			size_ = atoi(cfg->get_value("preload_frames", "5"));
			width_ = atoi(cfg->get_value("video_width", "480"));
			height_ = atoi(cfg->get_value("video_height", "270"));
			pos_ = 0;

			left_ = new cv::Mat[size_], init(left_);
			right_ = new cv::Mat[size_], init(right_);
			up_ = new cv::Mat[size_], init(up_);
			down_ = new cv::Mat[size_], init(down_);
			sum_ = cv::Mat::zeros(height_, width_, CV_8U);
		}

		~History()
		{
			delete []left_;
			delete []right_;
			delete []up_;
			delete []down_;
		}

		cv::Mat &left(int preid) const { return prev(left_, preid); }
		cv::Mat &right(int preid) const { return prev(right_, preid); }
		cv::Mat &up(int preid) const { return prev(up_, preid); }
		cv::Mat &down(int preid) const { return prev(down_, preid); }
		cv::Mat &sum() { return sum_; } // ???

		void clr()
		{
			// 删除所有历史 ...
		}
		
		void operator++()
		{
			pos_++;
			pos_ %= size_;
		}

		History &operator++(int)
		{
			pos_++;
			pos_ %= size_;
			return *this;
		}
	};

	History of_history;

public:
	DetectWithOpticalFlow(KVConfig *cfg);
	~DetectWithOpticalFlow(void);

	virtual std::vector<cv::Rect> detect0(size_t cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs)
	{
		return (this->*detect_00)(cnt, origin, gray_prev, gray_curr, dirs);
	}

	virtual int detect0(size_t cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr);

private:
	bool load_area_rect(const char *key, cv::Rect &rc);

	enum DIR
	{
		DIR_RIGHT,
		DIR_DOWN,
		DIR_LEFT,
		DIR_UP,
	};

	DIR get_dir(const cv::Mat &dirs_mat, const cv::Point &center, const cv::Rect &boundingRect, cv::Mat &origin, 
		const cv::Scalar &color = cv::Scalar(0, 255, 255));

	void draw_optical_flows(cv::Mat &origin, const cv::Mat &dis, const cv::Mat &angs);

	void (DetectWithOpticalFlow::*calc_flows)(cv::Mat &gray_prev, cv::Mat &gray_curr, cv::Mat &distance, cv::Mat &angles);
	void (DetectWithOpticalFlow::*calc_flow)(cv::Mat &gray_prev, cv::Mat &gray_curr, cv::Mat &distance, cv::Mat &angles);
	std::vector<cv::Rect> (DetectWithOpticalFlow::*detect_00)(size_t cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);

	inline bool get_flow(const cv::Mat &gray_curr, cv::ocl::oclMat &x, cv::ocl::oclMat &y);
	double get_roi_flow_sum(const cv::Mat &dis, const std::vector<cv::Point> &roi, cv::Mat &rgb);
	void calc_flows_ocl(cv::Mat &gray_prev, cv::Mat &gray_curr, cv::Mat &distance, cv::Mat &angles); // 计算累积光流..
	void calc_flows_cpu(cv::Mat &gray_prev, cv::Mat &gray_curr, cv::Mat &distance, cv::Mat &angles);
	void calc_flow_ocl(cv::Mat &gray_prev, cv::Mat &gray_curr, cv::Mat &distance, cv::Mat &angles);
	void calc_flow_cpu(cv::Mat &gray_prev, cv::Mat &gray_curr, cv::Mat &distance, cv::Mat &angles);
	void rgb_from_dis_ang(const cv::Mat &dis, const cv::Mat &angs, cv::Mat &rgb);	// 根据 dis, angs 生成rgb，便于显示..
	void load_flipped_polys(std::vector<std::vector<cv::Point> > &polys);
	inline void apply_threshold(const cv::Mat &dis, double factor, cv::Mat &bin);	// 对dis逐行用于factor，返回二值图，便于腐蚀膨胀 ...

	void update_history(const cv::Mat &dis, const cv::Mat &angles);	// 四个方向的光流，保存到 History 中..
	void update_history0(const cv::Mat &dis, const cv::Mat &dir, cv::Mat &target, int from, int to, int from2 = -1, int to2 = -2);

	std::vector<cv::Rect> detect_with_history(size_t cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);
	std::vector<cv::Rect> detect_with_sum(size_t cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);

	struct FindTargetResult
	{
		cv::Rect brc;
		double area, area_th;
		double fy;
	};

	std::vector<FindTargetResult> find_targets(cv::Mat &(History::*dir)(int) const, double area_factor, int frames, cv::Mat &debug_sum);
	void show_history(const std::vector<FindTargetResult> &rs, cv::Mat &(History::*dir)(int) const, cv::Mat &m0)
	{		
		m0 = (of_history.*dir)(0).clone();
		for (size_t n = 1; n <= preload_frames_-1; n++) {
			m0 |= (of_history.*dir)(n);
		}

		cv::blur(m0, m0, cv::Size(3, 3));
		cv::erode(m0, m0, ker_);
		cv::dilate(m0, m0, ker2_);

		for (size_t i = 0; i < rs.size(); i++) {
			cv::rectangle(m0, rs[i].brc, cv::Scalar(255));
			char buf[64];
			snprintf(buf, sizeof(buf), "%.2f|%.0f,%.0f", rs[i].fy, rs[i].area, rs[i].area_th);
			cv::putText(m0, buf, rs[i].brc.tl(), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255));
		}
	}
};
