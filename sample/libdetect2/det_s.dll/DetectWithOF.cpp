#include "DetectWithOF.h"

DetectWithOF::DetectWithOF(KVConfig *cfg)
	: Detect(cfg)
{
	debug_img_ = atoi(cfg_->get_value("debug_img3", "0"));
	ker_erode_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	ker_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
}

DetectWithOF::~DetectWithOF(void)
{
}

std::vector<cv::Rect> DetectWithOF::detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs)
{
	std::vector<cv::Rect> rcs;
	dirs.clear();

	// diff
	cv::Mat diff;
	cv::absdiff(gray_prev, gray_curr, diff);
	cv::threshold(diff, diff, 30, 255, cv::THRESH_BINARY);

	cv::erode(diff, diff, ker_erode_);
	cv::dilate(diff, diff, ker_dilate);

	if (debug_img_) {
		cv::imshow("diff", diff);
	}

	cv::Mat tmp = diff.clone();
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(tmp, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

	if (debug_img_) {
		//cv::drawContours(origin, contours, -1, cv::Scalar(0, 0, 255), 1);
	}

	// 所有轮廓的外接矩形
	std::vector<cv::Rect> brcs = getBoundingRects(contours);

	// 返回每个矩形的下一帧跟踪
	for (size_t i = 0; i < brcs.size(); i++) {
		if (brcs[i].area() < 300) continue;

		bool crossed = false;

		// 如果与 trackingRects_ 有交叉，则继续跟踪，如果无交叉，则认为是新的跟踪目标 ...
		for (size_t j = 0; !crossed && j < trackingRects_.size(); j++) {
			if (is_cross(brcs[i], trackingRects_[j].boundingRect())) {
				crossed = true;
			}
		}

		if (!crossed) {
			if (debug_img_) {
				cv::rectangle(origin, brcs[i], cv::Scalar(0, 0, 255));
			}

			TrackingRect tr;
			cv::Rect brc;
			std::vector<cv::Point2f> next_pts;
			if (tr.init(origin, gray_prev, brcs[i])) {
				trackingRects_.push_back(tr);
			}
		}
	}

	// 跟踪 
	for (size_t i = 0; i < trackingRects_.size(); i++) {
		cv::Rect brc;
		std::vector<cv::Point2f> next_pts;
		bool moving = trackingRects_[i].track(gray_prev, gray_curr, brc, next_pts);

		if (debug_img_) {
			for (size_t x = 0; x < next_pts.size(); x++) {
				cv::circle(origin, next_pts[x], 2, cv::Scalar(0, 255, 255), 2);
			}
			cv::rectangle(origin, brc, cv::Scalar(0, 255, 255));
			cv::rectangle(origin, trackingRects_[i].last_boundingRect(), cv::Scalar(255, 0, 255), 3);

			trackingRects_[i].draw_hist(origin);
		}

		if (moving && trackingRects_[i].hist_cnt() > 0) {
			cv::Point2f sp = trackingRects_[i].start_mean_pt(), ep = trackingRects_[i].end_mean_pt();
			cv::line(origin, sp, ep, cv::Scalar(0, 0, 255), 5);
		}

		if (!moving) {
			// 说明目标稳定了，此时获取目标位置，运动方向
			cv::Rect rc;
			int dir;
			if (trackingRects_[i].get_result(rc, dir)) {
				rcs.push_back(rc);
				dirs.push_back(dir);

				const char* ss[] = { "right", "down", "left", "up" };
				cv::putText(origin, ss[dir], rc.tl(), cv::FONT_HERSHEY_PLAIN, 3.0, cv::Scalar(255, 255, 255));
			}
		}
	}

	// 删除不动的目标 ..
	trackingRects_.erase(std::remove_if(trackingRects_.begin(), trackingRects_.end(), TrackingRect::is_no_moving), trackingRects_.end());

	return rcs;
}

std::vector<cv::Rect> DetectWithOF::getBoundingRects(const std::vector<std::vector<cv::Point>> &contours)
{
	std::vector<cv::Rect> brcs;
	for (size_t i = 0; i < contours.size(); i++) {
		try {
			brcs.push_back(cv::boundingRect(contours[i]));
		}
		catch (...) {}
	}
	return brcs;
}
