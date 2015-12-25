#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include "../libkvconfig/KVConfig.h"
#include <time.h>
#include <assert.h>
#include "objdet.h"
#include <cc++/thread.h>
#include "polyfit.h"
#include "SkinMask.h"
#include <list>
#include "History.h"

#define CENTER_X(RECT) ((RECT).x+(RECT).width/2)
#define CENTER_Y(RECT) ((RECT).y+(RECT).height/2)

#define _USE_MATH_DEFINES
#include <math.h>

typedef std::vector<cv::Rect> RECTS;

static double _distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
	return sqrt(pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2));
}

// 方向
enum Dir
{
	RIGHT = 0,
	DOWN = 1,
	LEFT = 2,
	UP = 3,
};

static const char *DirDesc[] = {
	"right", "down", "left", "up", 
};

// 返回两个矩形是否有交集，或者包容
inline bool is_cross(const cv::Rect &rc1, const cv::Rect &rc2)
{
	return !(rc1.x+rc1.width <= rc2.x || rc2.x+rc2.width <= rc1.x || rc1.y + rc1.height <= rc2.y || rc2.y+rc2.height <= rc1.y);
}

class Detect
{
	class Target
	{
	public:
		Target()
		{
			updated_cnt = 0;
			detecte_state_ = 0;
		}

		cv::Rect pos;	// 位置
		double stamp;	// 创建 target 的时间 

		/* XXX: 照理说，每个目标的活动需要多帧来确定，就是说，一个目标位置，必须出现连
				续更新，才说明这个位置是有效的目标 ...
		 */
		int updated_cnt;	

		/** 目标识别:
				0: 还没有进行
				1: 已经确认
				<0: 识别识别次数 ...
		 */
		int detecte_state_;

		Dir dir;
	};

	// 记录最后一次活动的矩形，包含方向 
	class MotionRect : public Target
	{
		Dir dir_;

	public:
		MotionRect(const cv::Rect &rc, double stamp, Dir dir)
		{
			pos = rc, Target::stamp = stamp, dir_ = dir;
		}

		const cv::Rect &rc() const { return pos; }
		double stamp() const { return Target::stamp; }
		Dir dir() const { return dir_; }
	};

	std::list<MotionRect> down_motion_rects_;	// 保存两秒之前的活动区域，不包含向上的了 ...

	/** 对应有效活动 */
	struct Motions
	{
		cv::Rect rc;	// 活动位置
		Dir dir;		// 活动方向
		double stamp;	// 时间戳
	};
	std::vector<Motions> motion_hists_;

	/** 删除超时的活动历史 */
	struct TooooOldofMotionHist
	{
		double now_, delay_;

		TooooOldofMotionHist(double now, double delay)
		{
			now_ = now, delay_ = delay;
		}

		bool operator()(const Motions &m) const
		{
			return now_ - m.stamp > delay_;
		}
	};

	bool flipped_;
	ost::Mutex lock_;

	typedef std::vector<Target> TARGETS;
	TARGETS targets_;	// 目标 ..

	struct TooOld
	{
	private:
		double now_;
		double duration_;

	public:
		TooOld(double now, double duration = 30.0) 
		{ 
			now_ = now; 
			duration_ = (double)duration;
		}

		bool operator ()(const Target &t) const
		{
			return now_ - t.stamp > duration_;	// 大于三十秒 ..
		}
	};

	class ToooLarge
	{
	private:
		Detect *parent_;

	public:
		ToooLarge(Detect *det): parent_(det) {}
		bool operator()(const Target &t) const;
	};
	friend class ToooLarge;

	class ToooSmall
	{
	private:
		Detect *parent_;
		double now_;

	public:
		ToooSmall(Detect *det, double now): parent_(det), now_(now) {}
		bool operator()(const Target &t) const;
	};
	friend class ToooSmall;

	cv::Mat ker_;	// 对灰度图像进行增强 ...

	objdet *od_;	// 上半身检测.
	int od_max_times_;	// 对于每个 target 最多检测次数, 默认 5 次

	SkinMask *skin_;
	double skin_head_ratio_;	// 肤色至少在头肩框内的面积比例，缺省 >=20%

	bool first_;	// 是否第一帧 ...
	cv::Mat gray_prev_, gray_curr_;	// 连续两帧灰度

	size_t st_cnt_;	// 统计
	double st_begin_, st_seg_;
	double fps_;

	double factor_equation_x_[3], factor_equation_y_[3];	// 二次方程系数，分别用于微调
	double factor_equation_linear_y_[2];	// y轴线性方程 ...
	double factor_equation_area_y_[2];	// 直线方程， 

	bool wait_key_;	// 是否调用 waitkey

	double max_duration_;	// 最大持续时间，防止错误判断
	int min_updated_;	// 最小更新次数
	double min_updated_delay_; // 检查最小更新次数的时间间隔，大于两帧时间差即可 ..
	double matched_area_factor_;	// 非向上框面积的比较阈值，就是说，太小的框不应该影响很大的目标
	double up_area_tolerance_factor_; // target 面积阈值系数，与理论值的容忍范围，默认 1.3

	double motion_hist_delay_;	// 保留多长时间的活动历史，用于评估是否为有效的站立目标 ...

	bool save_history_;
	int target_x_, target_y_;	// 最近处，理想的人的大小，默认 130 x 170

public:
	Detect(KVConfig *cfg);
	virtual ~Detect();

	virtual void set_param(int thres_dis, int thres_area, double factor0, double factor05)
	{
		//thres_dis_ = thres_dis;
		thres_area_ = thres_area;
		
		factor_0_ = factor0;
		//factor_05_ = factor05;

		//polyfit(factor_0_, factor_05_, factor_1_, factor_equation_y_);
		polyfit_linear(factor_0_, factor_1_, factor_equation_linear_y_);
	}

	void set_flipped_mode(int enabled);
	std::vector<cv::Rect> current_targets();
	virtual void detect(cv::Mat &origin, std::vector<cv::Rect> &targets, int &flipped_index);

	void (*log)(const char *fmt, ...);

	/** 从 32FC1 中找出聚合点，返回外接矩形

		fy = factor_y();
		size *= (fy * fy);
		threshold *= fy
		
		当一个矩形中值的和大于 threshold 时，则认为这个矩形中又有效的目标 ...
	 */
	std::vector<cv::Rect> find_clusters(const cv::Mat &m, const cv::Size &size = cv::Size(100, 100), const double threshold = 2500,
		int stepx = 4, int stepy = 4) const;

protected:
	// 派生类去实现： assert(ret.size() == dirs.size())
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs) 
	{
		return std::vector<cv::Rect>();
	}
	virtual int detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr) { return -1; };	// 翻转模式探测

	void draw_area_thres_info(cv::Mat &origin);


	// 两点间距离.
	inline int distance(int x1, int y1, int x2, int y2)
	{
		return (int)sqrt((float)(x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
	}

	KVConfig *cfg_;	//
	int debug_, debug_log_, debug_img_, debug_img2_;
	int video_width_, video_height_;
	cv::Mat origin_;		// 为了方便画图 ...
	double factor_0_, factor_1_; //, factor_05_;	// 图像y，从上到下的阈值系数.
	double thres_dis_, thres_dis_far_, thres_area_, thres_flipped_dis_;	// 距离阈值，距离阈值（远处），面积阈值，翻转课堂距离阈值
	double far_ratio_;
	bool face_detect_far_;	// 后排是否启用检测..
	double *factor_y_tables_;	// 预先计算，优化，用线性方程得了 :)
	cv::Rect area_min_rect_, area_max_rect_;	// 这个目的是为了大致推导出面积阈值 ...
	int area_max_, area_min_;	// 最大，最小面积，根据 area_max_rect_, area_min_rect_ 计算..
	int max_rect_factor_;	// 目标位于最大框中的系数，默认1.1，这个位置的目标都变态的大 
	double curr_stamp_;
	double max_target_area_;	// 最大目标面积
	History<cv::Mat> frames_history_;	// 保留N帧历史的灰度图，当 frames_history_.full() 后，才能使用 ...

	/// 根据 factor_y_ 估算目标大小
	cv::Size est_target_size(int y)
	{
		return cv::Size((int)(target_x_ * factor_y(y)), (int)(target_y_ * factor_y(y)));
	}

	void (*log_init)(const char *fname);

	bool is_far(const cv::Rect &rc);	// 返回是否为后排

	double now() const
	{
		timeval tv;
		ost::gettimeofday(&tv, 0);
		return tv.tv_sec + tv.tv_usec/1000000.0;
	}

	//void polyfit(double f0, double f05, double f1, double *factors)
	//{
	//	double xx[3] = { 0, video_height_ / 2, video_height_ };
	//	double yy[3] = { f0, f05, f1 };
	//	::polyfit(3, xx, yy, 2, factors);

	//	fprintf(stderr, "calc factors of equation: %f %f %f\n",
	//		factor_equation_y_[2], factor_equation_y_[1], factor_equation_y_[0]);
	//}

	// f0 对应着0行的系数，缺省0.2 吧，f1 对应着最底行的，就是 1 吧
	void polyfit_linear(double f0, double f1, double *factors)
	{
		double xx[2] = { 0, video_height_ };
		double yy[2] = { f0, f1 };
		::polyfit(2, xx, yy, 1, factors);
	}

	double factor_y(int y) const
	{
		/** 因探测角度是俯拍，所以前后系数变化不是线性的，
			直觉想象，前排到后排的阈值系数应该类似一个抛物线，前排变化快，后排变化慢 
			需要估计出一个在 [0  video_height) 区间内的抛物线的系数

			f(y) = a * y^2 + b * y + c

			已知:
				f(0) = factor_0_;
				f(video_height_) = factor_1_;

			factor_0_ 与 factor_1_ 之间的关系就是在图像中看起来后排和前排“水平等长”的比例
			这样再估计一个中间位置的水平比例(factor_0.5_)，就能够得到 a,b,c 参数了

			FIXME: 真正的曲线肯定不是二次的，感觉....

			对于成像来说，线段长度比例基本是线性的，考虑到前排的仰角，似乎可以采用线性规律，这样中间系数和后排系数基本保持为线性的就行了

		 */
//		return factor_equation_y_[0] + factor_equation_y_[1] * y + factor_equation_y_[2] * y * y;
		return factor_equation_linear_y_[0] + factor_equation_linear_y_[1] * y;	//
	}

private:
	void try_object_detect();	// 尝试进行对象识别 ...
	void merge_rcs(const std::vector<cv::Rect> &rcs, const std::vector<int> &dirs, double stamp);
	void remove_bigger_smaller();
	void remove_up_of_last_motion(std::vector<cv::Rect> &rcs, std::vector<int> &dirs);
	std::vector<Target>::iterator find_matched(const cv::Rect &rc, int dir);
	bool is_target(const cv::Rect &rc, const Dir &dir);
	bool load_area_rect(const char *key, cv::Rect &rc)
	{
		const char *v = cfg_->get_value(key, 0);
		int left, top, right, bottom;
		if (v && 4 == sscanf(v, "(%d,%d),(%d,%d)", &left, &top, &right, &bottom)) {
			rc = cv::Rect(cv::Point(left, top), cv::Point(right, bottom));
			return true;
		}

		return false;
	}
	void update_motion_hist(const std::vector<cv::Rect> &rcs, const std::vector<int> &dirs);
	void remove_timeouted_motion_hist();
	std::vector<Motions> get_nearby_motion_hist(const cv::Rect &rc, const Dir &d);	// 返回与rc接近的位置的历史 ...

	// 日志实现.
	static std::string _log_fname;
	static void log_file(const char *fmt, ...)
	{
		va_list args;
		char buf[1024];

		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);

		FILE *fp = fopen(_log_fname.c_str(), "at");
		if (fp) {
			time_t now = time(0);
			struct tm *ptm = localtime(&now);
			fprintf(fp, "%02d:%02d:%02d.%03d: %s", 
				ptm->tm_hour, ptm->tm_min, ptm->tm_sec, GetTickCount() % 1000, buf);
			fclose(fp);
		}
	}

	static void log_init_file(const char *fname)
	{
		_log_fname = fname;
		FILE *fp = fopen(fname, "w");
		if (fp) {
			fprintf(fp, "------ log begin ---------\n");
			fclose(fp);
		}
	}

	static void log_dummy(const char *fmt, ...)
	{
		(void)fmt;
	}

	static void log_init_dummy(const char *notused)
	{
		(void)notused;
	}
};
