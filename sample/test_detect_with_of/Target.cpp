#include "Target.h"
#include "Target.h"

Target::Target()
{
}

Target::~Target()
{
}

bool Target::init(KVConfig *cfg, int id, const cv::Rect &roi, const cv::Mat &curr_gray, double stamp)
{
	first_rc_ = roi;
	outer_.x = 0, outer_.y = 0, outer_.width = curr_gray.cols, outer_.height = curr_gray.rows;
	stamp_ = stamp;
	cfg_ = cfg;
	id_ = id;

	PTS pts;
	cv::goodFeaturesToTrack(curr_gray(roi), pts, 300, 0.05, 1.5);

	if (pts.size() < 15) {
		return false;
	}

	l2g(pts, roi.tl());
	
	layers_.push_back(pts);

	brc_ = roi;
	last_rc_ = cv::boundingRect(pts);

	return true;
}

bool Target::track(const cv::Mat &prev, const cv::Mat &curr, double stamp)
{
	// TODO: 应该根据轨迹的方向扩展搜索范围 ...
	// FIXME: 简单的四周扩展 ...
	// 第一次得到的特征点，总有部分不在活动目标上，所以应该在N帧之后，扔掉这些点，让“跟踪点”真正落在目标上 ...

	int exp = 60;	// 不知道这个距离是否合理 ...
	cv::Rect search_roi = last_rc_;
	search_roi.x -= exp;
	search_roi.y -= exp;
	search_roi.width += 2 * exp;
	search_roi.height += 2 * exp;
	search_roi &= outer_;

	PTS last_pts = layers_.back(), curr_pts;
	g2l(last_pts, search_roi.tl());

	cv::Mat status, err;
	cv::calcOpticalFlowPyrLK(prev(search_roi), curr(search_roi), last_pts, curr_pts, status, err);

	for (int r = 0; r < status.rows; r++) {
		// 标记找错的
		if (status.at<uchar>(r, 0) != 1) {
			curr_pts[r].x = -10000;
		}
	}

	/// 删除错误点对应的轨迹 ...
	PTS valid_pts;
	for (int i = (int)curr_pts.size() - 1; i >= 0; i--) {
		if (curr_pts[i].x < -5000) {
			remove_path(i);
		}
		else {
			valid_pts.push_back(curr_pts[i]);
		}
	}

	if (valid_pts.size() < 10) {
		return false;
	}

	std::reverse(valid_pts.begin(), valid_pts.end());	// 需要反序 ...

	l2g(valid_pts, search_roi.tl());

	layers_.push_back(valid_pts);
	last_rc_ = cv::boundingRect(valid_pts);
	brc_ |= last_rc_;

	check_paths(stamp);

	return true;
}

bool Target::is_crossed(const cv::Rect &rc) const
{
	return (rc & brc_).area() > 10;
}

bool Target::is_stopped() const
{
	/** 如果path的最后5个距离的平均值小于 2.0，则认为目标停止活动了 ...
	 */
	if (layers_.size() <= 5 || layers_[0].size() < 5) {
		return false;
	}

	std::vector<PATH> paths = get_sorted_paths((int)layers_.size() - 6, (int)layers_.size());
	
	// 统计距离最长的5条路径的最后5帧的距离 ...
	double sum = 0.0;
	for (size_t i = 0; i < 5; i++) {
		sum += distance(paths[i]);
	}
	sum /= 5.0 * 5.0;

	return sum < 2.0;
}

void Target::debug_draw_paths(cv::Mat &rgb, cv::Scalar &color) const
{
	for (int i = 0; i < (int)layers_[0].size(); i++) {
		PATH path = get_path(i);
		if (distance(path) > 5.0) {
			debug_draw_path(rgb, path, color);
		}
	}

	// 画第一层的外接，和最后一层的外接
	if (layers_.size() >= 2) {
		cv::rectangle(rgb, cv::boundingRect(layers_.front()), cv::Scalar(0, 0, 127));
		cv::rectangle(rgb, cv::boundingRect(layers_.back()), cv::Scalar(0, 128, 0));
	}

	char info[64];
	sprintf(info, "%d", id_);
	cv::putText(rgb, info, brc_.br(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(128, 255, 255));
	cv::rectangle(rgb, brc_, cv::Scalar(0, 255, 255));
}

void Target::debug_draw_path(cv::Mat &rgb, const Target::PATH &path, cv::Scalar &color) const
{
	if (path.size() >= 2) {
		cv::circle(rgb, path[0], 2, cv::Scalar(0, 0, 200));
	}

	for (size_t i = 1; i < path.size(); i++) {
		cv::line(rgb, path[i - 1], path[i], color);
	}
}

void Target::check_paths(double stamp)
{
	/** 如果持续时间很久，或者 layers_ 很大时，开始检查 
	 */

	if (layers_.size() > 30) {
		// XXX: 占用内存很小，也没有必要扔掉 ....
	}
}
