#include "objdet.h"
#include <cstdio>

objdet::objdet(KVConfig *cfg)
	: cfg_(cfg)
{
	loaded_ = false;

	const char *fname = cfg_->get_value("faces_meta_fname", "data/faces_18_18.xml");
	loaded_ = det_faces_.load(fname);

	debug2_ = 0;
	if (atoi(cfg_->get_value("debug", "0")) == 1) {
		debug2_ = atoi(cfg_->get_value("debug2", "0"));
	}
}

objdet::~objdet(void)
{
}

bool objdet::has_faces(const cv::Mat &origin, std::vector<cv::Rect> &faces)
{
	/** 检测 img 中，是否包含“人脸” */
	if (!loaded_) {
		return false;
	}

	const double dx = 2.0, dy = 2.0;

	cv::Mat gray;
	cv::cvtColor(origin, gray, cv::COLOR_BGR2GRAY);
	cv::resize(gray, gray, cv::Size(gray.cols * dx, gray.rows * dy));	//
    printf("detect begin\n");
	det_faces_.detectMultiScale(gray, faces); // , 1.1, 3, 0, cv::Size(18, 18));
    printf("detect over\n");

	for (std::vector<cv::Rect>::iterator it = faces.begin(); it != faces.end(); ++it) {
		if (debug2_) {
			cv::rectangle(gray, *it, cv::Scalar(255, 255, 255));
		}

		it->x /= dx, it->y /= dy;
		it->width /= dx, it->height /= dy;
	}

	if (debug2_) {
		cv::imshow("recognize", gray);
	}

	return !faces.empty();
}
