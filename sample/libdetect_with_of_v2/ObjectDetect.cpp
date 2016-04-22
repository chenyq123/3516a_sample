#include "ObjectDetect.h"
#include "utils.h"

ObjectDetect::ObjectDetect(KVConfig *cfg)
{
	cfg_ = cfg;
	reload();
}

ObjectDetect::~ObjectDetect()
{
}

bool ObjectDetect::reload()
{
	loaded_ = false;
	int debug = atoi(cfg_->get_value("debug", "0"));

	if (atoi(cfg_->get_value("face_detect", "0")) == 1) {
		const char *fname = cfg_->get_value("faces_meta_fname", "data/header_18_18_haar.xml");
		loaded_ = cc_.load(fname);

		log_file("ObjectDetect enabled: loaded '%s' %s\n", fname, loaded_ ? "OK" : "Fail!!!");
	}
	else {
		log_file("ObjectDetect disabled\n");
	}

	return loaded_;
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
