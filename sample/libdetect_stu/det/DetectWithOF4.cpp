#include "DetectWithOF4.h"

DetectWithOF4::DetectWithOF4(KVConfig *cfg)
	: Detect(cfg)
{
	threshold_diff_ = atof(cfg_->get_value("of4_threshold_diff", "25.0"));

	int s = atoi(cfg_->get_value("of4_kernel_erode_size", "3"));
	ker_erode_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(s, s));
	s = atoi(cfg_->get_value("of4_kernel_dilate_size", "21"));
	ker_dilate_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(s, s));
}

DetectWithOF4::~DetectWithOF4(void)
{
}

std::vector<cv::Rect> DetectWithOF4::detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs)
{
	// 帧差位置
	std::vector<cv::Rect> diff_rcs = get_diff_rects(gray_prev, gray_curr);

	// 检查是否应该创建 motion object
	for (size_t i = 0; i < diff_rcs.size(); i++) {
		bool new_mt = true;
		for (size_t j = 0; j < motion_objects_.size(); j++) {
			if (is_cross(motion_objects_[j].get_curr_brc(), diff_rcs[i])) {
				new_mt = false;
				break;
			}
		}

		if (new_mt) {
			MotionObject mo(this, diff_rcs[i], gray_prev);
			if (mo.is_moving()) {
				motion_objects_.push_back(mo);
			}
		}
	}
    printf("size:%d\n",motion_objects_.size());

	for (int i = motion_objects_.size()-1; i >= 0; i--) {
		motion_objects_[i].draw_hist(origin);
		motion_objects_[i].track(gray_curr);
		if (!motion_objects_[i].is_moving()) {
			motion_objects_.erase(motion_objects_.begin()+i);
		}
	}

	std::vector<cv::Rect> rcs;
	dirs.clear();
	return rcs;
}

