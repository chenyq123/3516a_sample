#pragma once
#include "detect.h"

/** 使用稀疏光流，持续跟踪目标，当连续 N 秒没有较大移动时，则认为本次跟踪结束

 */
class DetectWithOF4 : public Detect
{
	typedef std::vector<cv::Point> CONTOUR;	// 轮廓
	typedef std::vector<cv::Point2f> FEATURES;	// 特征点

	/** 描述一个对象的连续活动
		
			对象的创建：
				根据帧差，当出现较大帧差时，腐蚀膨胀后，得到轮廓，然后为每个轮廓创建一个 MotionObject，在MotionObject范围内找“特征点”开始跟踪；

			对象的跟踪：
				计算稀疏光流，跟踪，聚合主要方向的特征点，删除分离的特征点；及时补充特征点

			对象的销毁：
				当特征点数目少于阈值，或特征点的移动速度小于阈值后，认为目标已经停止，则销毁；
	 */
	class MotionObject
	{
		DetectWithOF4 *parent_;
		std::vector<CONTOUR> contours_hist_;	// 
		std::vector<cv::Point2f> mean_pt_hist_;	// 运动历史，就是中心点的位置？？
		int max_features_, min_features_;
		FEATURES curr_features_;
		bool moving_;
		cv::Mat prev_;
		double stamp_update_;

	public:
		MotionObject(DetectWithOF4 *parent, const cv::Rect &brc, const cv::Mat &curr_gray) : parent_(parent)
		{
			max_features_ = 300;
			min_features_ = 10;

			CONTOUR region;
			region.push_back(brc.tl());
			region.push_back(cv::Point(brc.x, brc.y+brc.height));
			region.push_back(brc.br());
			region.push_back(cv::Point(brc.x+brc.width, brc.y));

			curr_features_ = update_features(max_features_, curr_features_, region, curr_gray);
			moving_ = curr_features_.size() > min_features_;
			prev_ = curr_gray;

			if (moving_) {
				contours_hist_.push_back(region);
				mean_pt_hist_.push_back(mean_pt(curr_features_));
			}
		}

		bool is_moving() const { return moving_; }

		void track(const cv::Mat &gray)
		{
			FEATURES next_pts;
			cv::Mat status, err;
			cv::calcOpticalFlowPyrLK(prev_, gray, curr_features_, next_pts, status, err);
			cv::Mat s = status.reshape(1, 1), e = status.reshape(1, 1);
			unsigned char *ps = s.ptr<unsigned char>(0), *pe = s.ptr<unsigned char>(0);

			FEATURES features;
			for (size_t i = 0; i < status.cols; i++) {
				if (ps[i] == 1) {
					features.push_back(next_pts[i]);
				}
			}

			if (features.size() < min_features_) {
				moving_ = false;
				return;
			}

			remove_stray(features);	// 
			if (features.size() < min_features_) {
				curr_features_ = update_features(max_features_, features, get_contour(features), gray);
			}
			else {
				curr_features_ = features;
			}

			prev_ = gray;	// 
		}

		cv::Rect get_curr_brc() const
		{
			return cv::boundingRect(curr_features_);
		}

		void draw_hist(cv::Mat &img) const
		{
			if (mean_pt_hist_.size() > 0) {
				cv::circle(img, mean_pt_hist_[0], 2, cv::Scalar(0, 0, 255));
			}

			for (size_t i = 1; i < mean_pt_hist_.size(); i++) {
				cv::line(img, mean_pt_hist_[i-1], mean_pt_hist_[i], cv::Scalar(0, 0, 255));
			}

			for (size_t i = 0; i < curr_features_.size(); i++) {

			}
		}

	private:
		// 返回 features 的轮廓
		CONTOUR get_contour(const FEATURES &pts)
		{
			CONTOUR tmp, c;
			for (size_t i = 0; i < pts.size(); i++) {
				tmp.push_back(pts[i]);
			}

			cv::convexHull(tmp, c);
			return c;
		}

		// 平均点的位置
		cv::Point2f mean_pt(const FEATURES &pts) const
		{
			double sx = 0.0, sy = 0.0;
			for (size_t i = 0; i < pts.size(); i++) {
				sx += pts[i].x, sy += pts[i].y;
			}
			cv::Rect all(0, 0, parent_->video_width_, parent_->video_height_);
			cv::Point2f m = cv::Point2f(sx/pts.size(), sy/pts.size());
			if (m.x < 0) m.x = 0.1;
			if (m.y < 0) m.y = 0.1;
			if (m.x >= parent_->video_width_-0.1) m.x = parent_->video_width_-1;
			if (m.y >= parent_->video_height_-0.1) m.y = parent_->video_height_-1;
			return m;	// 平均位置
		}

		// 到平均点的距离
		double mean_dis(const cv::Point2f &mean_pt, const FEATURES &pts) const
		{
			double sd = 0.0;
			for (size_t i = 0; i < pts.size(); i++) {
				sd += _distance(mean_pt, pts[i]);
			}
			return sd / pts.size();
		}

		// 删除离群点
		void remove_stray(FEATURES &pts)
		{
			if (pts.size() < 3) {
				return;
			}

			/** 首先计算所有点的中心，计算所有点到中心的距离，删除超过平均距离的点
			 */
			cv::Point2f mean = mean_pt(pts);
			double mdis = mean_dis(mean, pts);

			for (int i = pts.size()-1; i >= 0; i--) { // 从后往前删除
				if (_distance(pts[i], mean) > 2 * mdis) {
					pts.erase(pts.begin() + i);
				}
			}
		}

		// 更新
		FEATURES update_features(int max, const FEATURES &old, const CONTOUR &region, const cv::Mat &curr_gray)
		{
			FEATURES features;
			cv::Rect brc = cv::boundingRect(region);
			cv::goodFeaturesToTrack(curr_gray(brc), features, max - old.size(), 0.05, 1.0);

			for (size_t i = 0; i < features.size(); i++) {
				features[i].x += brc.x;
				features[i].y += brc.y;
			}

			// 合并 old
			for (size_t i = 0; i < old.size(); i++) {
				if (!has_same_feature_pt(features, old[i])) {
					features.push_back(old[i]);
				}
			}

			return features;
		}

		bool has_same_feature_pt(const FEATURES &fs, const cv::Point2f &pt)
		{
			for (size_t i = 0; i < fs.size(); i++) {
				if (distance(pt, fs[i]) < 1.0) {
					return true;
				}
			}
			return false;
		}

		inline double distance(const cv::Point2f &p0, const cv::Point2f &p1) const
		{
			return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
		}
	};
	friend class MotionObject;
	std::vector<MotionObject> motion_objects_;

	/** 描述一个目标，目标由一个或者多个 MotionObject 构成，如一个人，可能有躯干和四肢的组合运动得到

			创建：
				???? 
	 */
	class Target
	{
	public:
		Target()
		{
		}
	};

	double threshold_diff_;	// 缺省 25
	cv::Mat ker_erode_, ker_dilate_;

public:
	DetectWithOF4(KVConfig *cfg);
	~DetectWithOF4(void);

private:
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);
	std::vector<cv::Rect> get_diff_rects(const cv::Mat &prev, const cv::Mat &curr)	// 根据帧差，腐蚀膨胀，然后找轮廓，返回轮廓外接矩形
	{
		cv::Mat diff;
		cv::absdiff(prev, curr, diff);
		cv::threshold(diff, diff, threshold_diff_, 255.0, cv::THRESH_BINARY);

		cv::erode(diff, diff, ker_erode_);
		cv::dilate(diff, diff, ker_dilate_);

		cv::imshow("ed", diff);

		std::vector<CONTOUR> contours;
		cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		std::vector<cv::Rect> rcs;
		for (size_t i = 0; i < contours.size(); i++) {
			rcs.push_back(cv::boundingRect(contours[i]));
		}

		return rcs;
	}
};
