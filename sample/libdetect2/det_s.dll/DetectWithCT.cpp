#include "DetectWithCT.h"

#define DIFF_THRESHOLD 25

DetectWithCT::DetectWithCT(KVConfig *cfg)
	: Detect(cfg)
{
	ker_erode_ = cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(1, 1));
	ker_dilate_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));

	cc_.load("data/header_18_18.xml");
}

DetectWithCT::~DetectWithCT(void)
{
}

std::vector<cv::Rect> DetectWithCT::detect0(size_t cnt, cv::Mat &origin, cv::Mat &prev, cv::Mat &curr, cv::vector<int> &dirs)
{
	RECTS motion_rects = get_motion_rects_using_diff(prev, curr);

	RECTS all_headers;
	for (RECTS::const_iterator it = motion_rects.begin(); it != motion_rects.end(); ++it) {
		RECTS headers = get_headers(origin(*it));
		for (size_t i = 0; i < headers.size(); i++) {
			headers[i].x += it->x;
			headers[i].y += it->y;

			all_headers.push_back(headers[i]);
		}
	}

	for (size_t i = 0; i < all_headers.size(); i++) {
		cv::rectangle(origin, all_headers[i], cv::Scalar(255, 255, 255), 2);
	}

	RECTS rcs;
	return rcs;
}

RECTS DetectWithCT::get_motion_rects_using_diff(const cv::Mat &prev, const cv::Mat &curr)
{
	cv::Mat diff;
	cv::absdiff(prev, curr, diff);

	cv::threshold(diff, diff, DIFF_THRESHOLD, 255.0, cv::THRESH_BINARY);

	cv::erode(diff, diff, ker_erode_);
	cv::dilate(diff, diff, ker_dilate_);
	cv::dilate(diff, diff, ker_dilate_);

	cv::imshow("ct diff", diff);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	cv::drawContours(origin_, contours, -1, cv::Scalar(0, 0, 255));

	RECTS rcs;
	for (size_t i = 0; i < contours.size(); i++) {
		rcs.push_back(cv::boundingRect(contours[i]));
	}

	return rcs;
}

// 在 rc 内找头肩，注意：返回的是基于 rc 的偏移
RECTS DetectWithCT::get_headers(const cv::Mat &img)
{
	RECTS rcs;
	cv::Mat gray;
	cv::cvtColor(img, gray, CV_RGB2GRAY);

	cc_.detectMultiScale(gray, rcs);
	return rcs;
}
