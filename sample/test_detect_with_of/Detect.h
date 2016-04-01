#pragma once

#include <opencv2/opencv.hpp>
#include <cc++/thread.h>
#include "../libkvconfig/KVConfig.h"
#include "ObjectDetect.h"

enum Dir
{
	RIGHT,
	DOWN,
	LEFT,
	UP,
	NONE,
};

static const char *DirDesc[] =
{
	"RIGHT",
	"DOWN",
	"LEFT",
	"UP",
	"NONE",
};

/** 维护站立目标 ...
 */
class Detect
{
	/// 对应一个“站立”目标 ...
	struct Standup
	{
		cv::Rect pos;		// 
		double stamp;		// 确认时间戳 ...
		bool waiting;		// 等待以避免后续的误判 ...

		bool enable_od;		// 是否启用对象识别 ...
		cv::Rect face;		// 如果启用对象识别，则为 ...
	};
	typedef std::vector<Standup> STANDUPS;
	STANDUPS standups_;

	/// 配置项 ...
	double max_duration_;	// “站立”目标最大持续时间，超时，则强制认为坐下 ...
	double waiting_;		// 目标坐下后，等待一段时间，再相应此处的活动 ...
	bool masked_;
	cv::Mat mask_;
	ObjectDetect *od_;

public:
	explicit Detect(KVConfig *cfg);
	virtual ~Detect();

public:
	typedef std::vector<cv::Rect> RCS;

	/// 输入 rgb 图像，输出返回“站立”人的位置 ...
	virtual void detect(const cv::Mat &origin, RCS &standups);

	/// 清空所有数据，重新从配置文件中加载 ...
	virtual void reset();

protected:
	KVConfig *cfg_;
	size_t cnt_;
	double stamp_;
	cv::Mat origin_;
	cv::Mat gray_prev_, gray_curr_;		// 两帧灰度图，便于使用帧差法 ...
	bool debug_;
	void(*log_init)(const char *fname);
	void(*log)(const char *fmt, ...);

	/// 派生类返回不同方向的活动 ...
	virtual void detect(RCS &rcs, std::vector<Dir> &dirs) = 0;

	// 找到相交面积最大的 standup
	STANDUPS::iterator find_crossed_target(const cv::Rect &motion)
	{
		int max_area = 0;
		STANDUPS::iterator found = standups_.end();

		for (STANDUPS::iterator it = standups_.begin(); it != standups_.end(); ++it) {
			int crossed = (it->pos & motion).area();
			if (crossed > max_area) {
				max_area = crossed;
				found = it;
			}
		}

		return found;
	}

	double now() const
	{
		struct timeval tv;
		ost::gettimeofday(&tv, 0);
		return tv.tv_sec + tv.tv_usec * 0.000001;
	}

	bool build_mask(cv::Mat &mask)
	{
		bool masked = false;

		const char *pts = cfg_->get_value("calibration_data", 0);
		std::vector<cv::Point> points;

		if (pts) {
			char *data = strdup(pts);
			char *p = strtok(data, ";");
			while (p) {
				// 每个Point 使"x,y" 格式
				int x, y;
				if (sscanf(p, "%d,%d", &x, &y) == 2) {
					cv::Point pt(x, y);
					points.push_back(pt);
				}

				p = strtok(0, ";");
			}
			free(data);
		}

		if (points.size() > 3) {
			int n = (int)points.size();
			cv::vector<cv::Point> pts;
			for (int i = 0; i < n; i++) {
				pts.push_back(points[i]);
			}

			mask = cv::Mat::zeros(cv::Size(atoi(cfg_->get_value("video_width", "960")), 
				atoi(cfg_->get_value("video_height", "540"))), CV_8UC3);

			std::vector<std::vector<cv::Point> > ptss;
			ptss.push_back(pts);
			cv::fillPoly(mask, ptss, cv::Scalar(255, 255, 255));

			masked = true;
		}

		return masked;
	}
};
