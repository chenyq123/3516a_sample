#include "ObjectDetect.h"
#include "utils.h"

ObjectDetect::ObjectDetect(KVConfig *cfg)
{
	loaded_ = false;
	int debug = atoi(cfg->get_value("debug", "0"));

	if (atoi(cfg->get_value("face_detect", "0")) == 1) {
		const char *fname = cfg->get_value("faces_meta_fname", "data/header_18_18_haar.xml");
		loaded_ = cc_.load(fname);

		log_file("ObjectDetect enabled: loaded '%s' %s\n", fname, loaded_ ? "OK" : "Fail!!!");
	}
	else {
		log_file("ObjectDetect disabled\n");
	}
}

ObjectDetect::~ObjectDetect()
{
}

std::vector<cv::Rect> ObjectDetect::detect(const cv::Mat &gray, const cv::Rect &roi)
{
	std::vector<cv::Rect> objs;
	if (!loaded_) {
		return objs;
	}
	else {
		cc_.detectMultiScale(gray(roi), objs, 1.2);
		for (std::vector<cv::Rect>::iterator it = objs.begin(); it != objs.end(); ++it) {
			it->x += roi.x, it->y += roi.y;
		}
		return objs;
	}
}
