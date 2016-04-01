#pragma once

#include "detect.h"
#include <deque>

// 返回有效点的几何中心
static bool _center(const std::vector<cv::Point2f> &pts, cv::Point2f &center, const cv::Mat &valid = cv::Mat())
{
	const unsigned char *s = valid.ptr<unsigned char>(0);
	int n = 0;
	double x = 0.0, y = 0.0;

	if (valid.cols == pts.size()) {
		for (size_t i = 0; i < pts.size(); i++) {
			if (s[i]) {
				x += pts[i].x, y += pts[i].y;
				n++;
			}
		}
	}
	else {
		for (size_t i = 0; i < pts.size(); i++) {
			x += pts[i].x, y += pts[i].y;
			n++;
		}
	}

	if (n > 0) {
		center.x = x/n, center.y = y/n;
		return true;
	}
	else {
		return false;
	}
}

/** 基于稀疏光流的跟踪 ..

		1. 通过帧差，大致确定活动区域
		2. 在活动区域中，进行“头肩”识别，得到独立的人；
		3. 在“人”范围内找角点，
		4. 对“角点”进行跟踪
		5. 当被跟踪点不再活动时，本次跟踪结束
 */
class DetectWithOF : public Detect
{
	int debug_img_;	//
	cv::Mat ker_erode_, ker_dilate;

	/** 被跟踪目标，
			1. 当出现较大帧差结果时，开始跟踪， 创建跟踪目标后，不再关心此处帧差 ..
			2. 在目标范围内，goodFeaturesToTrack，如果小于阈值，则放弃该目标 ...
			3. calcOpticalFlowPyrLK，删除错误点，删除距离小于阈值的点，如果 features 数目小于阈值，则认为目标跟踪结束 ...
			4. 循环 3
	 */
	class TrackingRect
	{
		cv::Mat origin_;
		cv::Rect rc_start_;	// 起始位置
		cv::Rect boundingRect_;	// 当前位置
		cv::Rect last_boundingRect_;
		std::vector<cv::Point2f> features_;
		cv::Point2f center_start_;
		int cnt_min_, cnt_max_; // 最少关键点 ..
		bool moving_;

		typedef std::deque<std::vector<cv::Point2f> > ALL_POINTS;
		ALL_POINTS all_points_;	// 所有跟踪点 ...

	public:
		bool init(const cv::Mat &origin, const cv::Mat &gray, const cv::Rect &rc)
		{
			origin_ = origin, rc_start_ = rc;
			cnt_max_ = sqrt(rc.area()*1.0)+1, cnt_min_ = cnt_max_ / 2;	// 最多，最少

			cv::goodFeaturesToTrack(gray(rc), features_, cnt_max_, 0.1, 3);
			for (size_t i = 0; i < features_.size(); i++) {
				features_[i].x += rc.x;	// 转换为整个图像的坐标
				features_[i].y += rc.y;
			}

			// 保存历史
			all_points_.push_front(features_);	// 第一帧历史

			_center(features_, center_start_);

			boundingRect_ = rc;
			last_boundingRect_ = rc;
			moving_ = true;

			return features_.size() > cnt_min_;
		}

		cv::Rect boundingRect() const { return boundingRect_; }		// 完整的
		cv::Rect last_boundingRect() const { return last_boundingRect_; } // 最后的

		void remove_hist_points(int n)
		{
			// n 对应着错误点的位置
			for (size_t i = 0; i < all_points_.size(); i++) {
				std::vector<cv::Point2f> &ps = all_points_[i];
				int x = 0;
				for (std::vector<cv::Point2f>::iterator it = ps.begin(); it != ps.end(); it++, x++) {
					if (x == n) {
						ps.erase(it);
						break;
					}
				}
			}
		}

		void draw_hist(cv::Mat &origin)
		{
			for (int i = 1; i < all_points_.size(); i++) {
				std::vector<cv::Point2f> &l0 = all_points_[i-1], &l1 = all_points_[i];

				if (l0.size() == l1.size()) {
					for (int j = 0; j < l0.size(); j++) {
						cv::line(origin, l0[j], l1[j], cv::Scalar(0, 255, 0));
					}
				}
			}
		}

		// 返回第n个点的活动轨迹历史 ..
		bool get_pt_hist(int n, std::vector<cv::Point2f> &path)
		{
			if (all_points_.empty() || n >= all_points_[0].size()) {
				return false;
			}

			path.clear();
			ALL_POINTS::const_reverse_iterator it;
			for (it = all_points_.rbegin(); it != all_points_.rend(); it++) {
				path.push_back((*it)[n]);
			}

			return true;
		}

		/// 返回活动的点数目
		int get_moving_points() const
		{
			assert(!all_points_.empty());	// track() 返回后，才能调用
			return all_points_[0].size();
		}

		/** 输入两帧图像进行跟踪 ...
		 */
		bool track(const cv::Mat &prev, const cv::Mat &curr, cv::Rect &boundingRc, std::vector<cv::Point2f> &next_pts)
		{
			std::vector<cv::Point2f> next_features;
			cv::Mat status, err;

			cv::calcOpticalFlowPyrLK(prev, curr, features_, next_features, status, err);

			cv::Mat s = status.reshape(1, 1), e = status.reshape(1, 1);
			unsigned char *ps = s.ptr<unsigned char>(0), *pe = s.ptr<unsigned char>(0);

			std::vector<cv::Point2f> moved;

			bool moving = false;

			for (size_t i = 0; i < features_.size(); i++) {
				if (ps[i] == 1 && (next_features[i].x > 0 && next_features[i].y > 0 && next_features[i].x < origin_.cols-1 && next_features[i].y < origin_.rows-1)) {
					if (_distance(next_features[i], features_[i]) >= 3) {	// 必须有较大位移
						moving = true;	// 仍在活动中 ...
						moved.push_back(next_features[i]);
					}
				}
				else {
					// 错误点，删除历史 ...
					remove_hist_points(i);
				}
			}

			if (moving) {
				all_points_.push_front(moved);

				//next_pts = keeper;
				next_pts = moved;

				boundingRc = cv::boundingRect(moved);

				//boundingRc |= this->rc_start_;
				boundingRect_ = boundingRc;
				//features_ = keeper;	// 更新跟踪点
				features_ = moved;

				last_boundingRect_ = cv::boundingRect(moved);	// 活动的点子

				// 看看每个活动点的历史轨迹 ...
			}
			else {
				moving_ = false;	// 停止跟踪 ...
			}

			return moving;
		}

		bool moving() const { return moving_; }

		bool get_result(cv::Rect &rc, int &dir)
		{
			// 如果是稳定，则返回完整外接矩形，和运动方向
			if (get_moving_points() > 0) {
				rc = boundingRect_;
				dir = get_dir();
				return true;
			}
			else {
				return false;
			}
		}

		// 所有点的平均中心
		cv::Point2f mean_of_points(const std::vector<cv::Point2f>&pts) const
		{
			assert(!pts.empty());

			double sx = 0, sy = 0;
			for (size_t i = 0; i < pts.size(); i++) {
				sx += pts[i].x, sy += pts[i].y;
			}

			return cv::Point2f(sx/pts.size(), sy/pts.size());
		}

		// 0=right, 1=down, 2=left, 3=up
		int get_dir()
		{
			// 简单的计算起点的中心到结束点的中心的偏角 ...
			cv::Point2f start = mean_of_points(all_points_.back());
			cv::Point2f stop = mean_of_points(all_points_.front());

			double a = atan2(start.y-stop.y, stop.x-start.x) * 180 / M_PI;	// 注意：y 轴为负

			if (a < 0) {
				a = 0.0 - a;	// 右，下，左
			}
			else {
				a = 360 - a;	// 左，上，右
			}

			int dir_idx = 1;	// 默认 down
			int up_half = 110 / 2;
			int up_min = 270 - up_half, up_max = 270 + up_half;
			if (a >= up_min && a <= up_max) {
				dir_idx = 3;	// 上
			}
			else if (a >= 45 && a <= 135) {
				dir_idx = 1;	// 下
			}
			else if (a > 135 && a < up_min) {
				dir_idx = 2;	// 左
			}
			else {
				dir_idx = 0;	// 右
			}

			return dir_idx;
		}

		int hist_cnt() const
		{
			return all_points_.size();
		}

		static bool is_no_moving(const TrackingRect &tr)
		{
			return !tr.moving_;
		}

		cv::Point2f start_mean_pt() const
		{
			return mean_of_points(all_points_.back());
		}

		cv::Point2f end_mean_pt() const
		{
			return mean_of_points(all_points_.front());
		}
	};

	std::vector<TrackingRect> trackingRects_;	// 正在跟踪的对象

public:
	DetectWithOF(KVConfig *cfg);
	virtual ~DetectWithOF(void);

private:
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);

	// 返回每个轮廓的外接矩形 ...
	std::vector<cv::Rect> getBoundingRects(const std::vector<std::vector<cv::Point> > &contours);
};
