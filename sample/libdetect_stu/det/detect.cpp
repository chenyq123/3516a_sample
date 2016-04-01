#include "detect.h"
//#include "direct.h"
#include <math.h>
#include <float.h>
#include <sys/stat.h>
#define MAX_PATH 256

std::string Detect::_log_fname = "det_s.log";

static const char *get_log_filename()
{
	static std::string fname;
	time_t t = time(0);
	struct tm *ptm = localtime(&t);

	int day = ptm->tm_wday; // 保存一个星期的 ..
	char buf[128];
	snprintf(buf, sizeof(buf), "detlog/det_s_%d.log", day);
	fname = buf;

	return fname.c_str();
}

Detect::Detect(KVConfig *cfg) : cfg_(cfg)
	, frames_history_(10)
{
	debug_ = atoi(cfg_->get_value("debug", "0"));
	debug_log_ = atoi(cfg_->get_value("debug_log", "0"));
	debug_img_ = atoi(cfg_->get_value("debug_img", "0"));
	debug_img2_ = atoi(cfg_->get_value("debug_img2", "0"));
	motion_hist_delay_ = atof(cfg_->get_value("motion_hist_delay", "1.5"));	// 默认的活动历史延迟
	save_history_ = atoi(cfg_->get_value("save_history", "0"));

    mkdir("detlog",666);
	//_mkdir("detlog");
	//system("del detlog\\*.jpg");
	//system("del detlog\\*.yaml");

	if (debug_) {
		//cv::namedWindow("student");
		//cv::moveWindow("student", 0, 540);	// 左下角
	}

	if (debug_log_) {
		log_init = log_init_file;
		log = log_file;
	}
	else {
		log_init = log_init_dummy;
		log = log_dummy;
	}

	log_init(get_log_filename());

	log("[VERSION Info]: build at %s\n", __TIMESTAMP__);
	log("%s: starting....\n", __FUNCTION__);

	video_width_ = atoi(cfg_->get_value("video_width", "480")), video_height_ = atoi(cfg_->get_value("video_height", "270"));

	max_duration_ = atof(cfg_->get_value("max_duration", "30.0"));
	//min_updated_ = atoi(cfg_->get_value("min_updated", "0"));
	//min_updated_delay_ = atof(cfg_->get_value("min_updated_delay", "0.4"));
	matched_area_factor_ = atof(cfg_->get_value("matched_area_factor", "5.0"));

	factor_0_ = atof(cfg_->get_value("factor_0", "0.2"));
	//factor_05_ = atof(cfg_->get_value("factor_05", "0.35"));
	factor_1_ = atof(cfg_->get_value("factor_1", "1.0"));
	//polyfit(factor_0_, factor_05_, factor_1_, factor_equation_y_);
	polyfit_linear(factor_0_, factor_1_, factor_equation_linear_y_);

	target_x_ = atoi(cfg_->get_value("of5_target_width", "130"));
	target_y_ = atoi(cfg_->get_value("of5_target_height", "170"));


	// XXX: 貌似采用较大的距离阈值能够更准确的框出活动?
	//thres_dis_ = atof(cfg_->get_value("thres_dis", "60.0"));	// 最前端距离阈值
	thres_dis_ = atof(cfg_->get_value("of3_threshold", "2.0")); // FIXME: 从32F转换到8U时，不做适应，而是直接使用原来的值，从观察看，没有必要使用前后系数
	thres_dis_far_ = atof(cfg_->get_value("of3_threshold_far", "1.4")); // 较远处，使用更小的阈值 ...
	far_ratio_ = atof(cfg_->get_value("of3_threshold_far_ratio", "0.3"));	// 何为较远处？
	thres_area_ = atof(cfg_->get_value("thres_area", "6400")); // 最前端面积阈值
	face_detect_far_ = atoi(cfg_->get_value("face_detect_far", "1")) == 1;

	if (far_ratio_ > 0.999) {
		far_ratio_ = 0.999;
	}
	if (far_ratio_ < 0.0001) {
		far_ratio_ = 0.0001;
	}

	if (atoi(cfg_->get_value("face_detect", "0")) == 1) {
		// 检查返回矩形中的人脸 ...
		od_ = new objdet(cfg_);
		if (!od_->loaded()) {
			log("WARNING: can't load meta file\n");
			delete od_;
			od_ = 0;
		}
	}
	else {
		od_ = 0;
	}

	if (atoi(cfg_->get_value("skin_detect", "0")) == 1) {
		skin_ = new SkinMask(cfg_);
		skin_head_ratio_ = atof(cfg_->get_value("skin_head_ratio", "0.3")); //
	}
	else {
		skin_ = 0;
	}

	ker_ = (cv::Mat_<char>(3, 3) << 0, -1, 0,
									-1, 4, -1,
									0, -1, 0); //

	if (od_) {
		od_max_times_ = atoi(cfg_->get_value("face_detect_max_times", "5"));
		log("\tenable od, meta fname=%s\n", cfg_->get_value("faces_meta_fname", 0));
	}
	else {
		log("\tdisable od\n");
	}

	if (skin_) {
		log("\tenable skin detect\n");
	}
	else {
		log("\tdisable skin detect\n");
	}

	flipped_ = false;
	if (atoi(cfg_->get_value("flipped_mode", "0")) == 1) {
		flipped_ = true;
	}

	log("\tface_detect_far: %d\n", face_detect_far_);
	log("\tfactor_0=%.3f\n", factor_0_);
	//log("\tfactor_05=%.3f\n", factor_05_);
	log("\tfactor_1=%.3f\n", factor_1_);
	log("\tmatched_area_factor: %.3f\n", matched_area_factor_);

	factor_y_tables_ = new double[video_height_];
	for (int i = 0; i < video_height_; i++) {
		factor_y_tables_[i] = factor_y(i);
	}

	max_rect_factor_ = atof(cfg_->get_value("max_rect_factor", "1.1"));
	if (!load_area_rect("area_max_rect", area_max_rect_)) {
		// 没有指定最大
		area_max_rect_.x = area_max_rect_.y = 0;
		area_max_rect_.width = area_max_rect_.height = 0;
	}

	area_min_rect_.x = area_min_rect_.y = area_min_rect_.width = area_min_rect_.height = 0;
	if (!load_area_rect("area_min_rect", area_min_rect_)) {
		area_min_rect_.x = area_min_rect_.y = 0;
		area_min_rect_.width = area_min_rect_.height = 0;
	}

	up_area_tolerance_factor_ = atof(cfg_->get_value("up_area_tolerance_factor", "1.8"));

	max_target_area_ = atof(cfg_->get_value("max_target_area", "10000"));

	area_min_ = area_min_rect_.area(), area_max_ = area_max_rect_.area();
	log("\tmax_rect: [%d,%d, %d,%d], area=%d\n", area_max_rect_.x, area_max_rect_.y, area_max_rect_.width, area_max_rect_.height, area_max_rect_.area());
	log("\tmin_rect: [%d,%d, %d,%d], area=%d\n", area_min_rect_.x, area_min_rect_.y, area_min_rect_.width, area_min_rect_.height, area_min_rect_.area());

	double xx[2] = { CENTER_Y(area_min_rect_), CENTER_Y(area_max_rect_) };
	double yy[2] = { sqrt(1.0*area_min_rect_.area()), sqrt(1.0*area_max_rect_.area()) };
	::polyfit(2, xx, yy, 1, factor_equation_area_y_);	// 面积的平方根按照线性变化吧 ...

	log("\tarea_min=%d, area_max=%d\n", area_min_, area_max_);
	log("\tof3_threshold=%.1f, thres_dis_tar=%.1f, far_ratio=%.3f\n", thres_dis_, thres_dis_far_, far_ratio_);
	log("\tthres_area=%.1f\n", thres_area_);
	log("\tvideo size=%d-%d\n", video_width_, video_height_);
	//log("\tthreshold factor: %f, %f, %f\n", factor_equation_y_[0], factor_equation_y_[1], factor_equation_y_[2]);
	log("\tarea factor: %f, %f\n", factor_equation_area_y_[0], factor_equation_area_y_[1]);
	log("\tup_area_tolerance_factor: %.3f\n", up_area_tolerance_factor_);
	log("\tmax_target_area: %.1f\n", max_target_area_);
	log("\tmotion_hist_delay: %.2f\n", motion_hist_delay_);
	log("\ttarget_width: %d, target_height: %d\n", target_x_, target_y_);

	first_ = true;

	st_cnt_ = 0;
	st_begin_ = now();
	st_seg_ = st_begin_;
	fps_ = -0.1;

	wait_key_ = true;
	char buf[MAX_PATH];
	//GetModuleFileNameA(0, buf, sizeof(buf));
	//if (strstr(buf, "test_imagesource.exe") || strstr(buf, "test_detect")) {
	//	wait_key_ = false;
	//}

	log("\n");
}

Detect::~Detect()
{
	delete od_;
	delete skin_;
	delete []factor_y_tables_;
	//cv::destroyAllWindows();
}

// 判断 pts 围成的轮廓是否更接近一个圆 ...
static bool like_circle(const std::vector<cv::Point> &pts)
{
	/** FIXME:
	 */
	return false;
}

void Detect::set_flipped_mode(int enabled)
{
	//ost::MutexLock al(lock_);
	flipped_ = !(enabled == 0);

	log("INFO: set_flipped_mode: enable=%d\n", flipped_);

	// 切换模式后，需要清空老的状态么?
	targets_.clear();
}

std::vector<cv::Rect> Detect::current_targets()
{
	//ost::MutexLock al(lock_);

	std::vector<cv::Rect> rets;
	for (size_t i = 0; i < targets_.size(); i++) {
		if (!od_ || targets_[i].detecte_state_ == 1) {
			rets.push_back(targets_[i].pos);
		}
	}

	return rets;
}

std::vector<Detect::Target>::iterator Detect::find_matched(const cv::Rect &rc, int dir)
{
	std::vector<Target>::iterator it = targets_.begin();
	for (; it != targets_.end(); ++it) {
		if (is_cross(it->pos, rc)) {
			// FIXME: 如果相对target的一个较小活动，则认为可能是无效的 ...
			if (dir != UP && it->pos.area() > matched_area_factor_ * rc.area()) {
				log(" XXX: %s: maybe a slice motion in target, just discard: dir=%s, old=%d, curr=%d\n",
					__FUNCTION__, DirDesc[dir], it->pos.area(), rc.area());
				continue;
			}
			return it;
		}
	}

	return it;
}

void Detect::merge_rcs(const std::vector<cv::Rect> &rcs, const std::vector<int> &dirs, double stamp)
{
	/** 从 targets_ 中找到距离接近 rcs 的框框，做出 dirs 的操作 */
	for (size_t i = 0; i < dirs.size(); i++) {
		std::vector<Target>::iterator matched = find_matched(rcs[i], dirs[i]);
		if (matched == targets_.end()) {
			// 没有找到匹配，如果是 up，则需要进一步判断是否为 Target，主要根据活动历史进行分析
			if (dirs[i] == UP) {
				if (is_target(rcs[i], UP)) {
					Target t;
					t.pos = rcs[i];
					t.stamp = stamp;
					t.updated_cnt = 0;
					t.dir = UP;
					targets_.push_back(t);
				}
			}
		}
		else {
			if (dirs[i] != UP) {
				targets_.erase(matched);	// 如果不是UP，则认为该目标消失！...
			}
			else {
				// 更新 matched
				matched->updated_cnt++;
				matched->pos |= rcs[i];

				if (matched->pos.area() > max_target_area_) {
					log("WARNING: TOOOO LARGE target area: [%d,%d, %d-%d], area=%d\n",
						rcs[i].x, rcs[i].y, rcs[i].width, rcs[i].height, matched->pos.area());
					targets_.erase(matched);
				}
			}
		}
	}
}

// 两个矩形是否顺向 ...
static bool rcs_with_dir(const cv::Rect &r0, const cv::Rect &r1, int dir)
{
	/** 返回 r1 相对 r0 是否符合 dir 方向 */
	bool rc = false;

	switch (dir) {
	case RIGHT:
		rc = CENTER_X(r1) > CENTER_X(r0);
		break;

	case LEFT:
		rc = CENTER_X(r0) > CENTER_X(r1);
		break;

	case DOWN:
		rc = CENTER_Y(r1) > CENTER_Y(r0);
		break;

	case UP:
		rc = CENTER_Y(r0) > CENTER_Y(r1);
		break;
	}

	return rc;
}

bool Detect::ToooLarge::operator()(const Detect::Target &t) const
{
	// 根据位置，检查是否太大了？？

	// 如果面积已经小于 area_max_ / 10，就算了 ....
	// 很小的矩形，已经严重偏离线性关系了
	if (t.pos.area() * 10 < parent_->area_max_) {
		return false;
	}

	int cy = CENTER_Y(t.pos);
	double exp_area = pow(parent_->factor_equation_area_y_[0] + parent_->factor_equation_area_y_[1] * cy, 2);	// 理论大小
	double area = t.pos.area();	// 实际大小
	double exp_area_2 = parent_->up_area_tolerance_factor_ * exp_area;

	if (area > parent_->area_max_ || area > exp_area_2) {
		parent_->log("WARNING: discard toooo LARGE target: (%d-%d), exp_area=%.1f, exp_area_tolerance=%.1f, area=%.1f, max=%d\n",
			t.pos.width, t.pos.height, exp_area, exp_area_2, area, parent_->area_max_);
		return true;
	}

	return false;
}

bool Detect::ToooSmall::operator()(const Detect::Target &t) const
{
	// FIXME: 即使前排也会产生不少小框，所以还是直接与最小框进行判断吧 ...
	return now_ - t.stamp > 0.3 && t.pos.area() < parent_->area_min_;

#if 0
	// 根据位置，时间，检查是否太小 ...
	if (now_ - t.stamp > 0.3) { // 防止第一帧很小的目标被删除 ...
		int cy = CENTER_Y(t.pos);
		double exp_area = parent_->factor_equation_area_y_[0] + parent_->factor_equation_area_y_[1] * cy;	// 理论大小
		double area = t.pos.area();	// 实际大小

		if (area < parent_->area_min_ || area < exp_area / 3) {
			parent_->log("WARNING: discard toooo SMALL target: (%d-%d), exp_area=%.1f, area=%.1f, min=%d\n",
				t.pos.width, t.pos.height, exp_area/3, area, parent_->area_min_);
			return true;
		}
	}

	return false;
#endif
}

void Detect::remove_bigger_smaller()
{
	// 检查 Target 中 ...
	ToooLarge tl(this);
	targets_.erase(std::remove_if(targets_.begin(), targets_.end(), tl), targets_.end());

//	ToooSmall ts(this, now());
//	targets_.erase(std::remove_if(targets_.begin(), targets_.end(), ts), targets_.end());
}

static bool n_in_vector(int n, const std::vector<int> &v)
{
	for (std::vector<int>::const_iterator it = v.begin(); it != v.end(); ++it) {
		if (n == *it) {
			return true;
		}
	}

	return false;
}

static inline bool same_rc(const cv::Rect &r0, const cv::Rect &r1)
{
	return r0.x == r1.x && r0.y == r1.y && r0.width == r1.width && r0.height == r1.height;
}

// 反序，
std::vector<Detect::Motions> Detect::get_nearby_motion_hist(const cv::Rect &rc, const Dir &dir)
{
	std::vector<Motions> hist;
	cv::Rect curr = rc;
	for (int i = motion_hists_.size()-1; i >= 0; i--) {
		// 删除自己 ..
		if (same_rc(rc, motion_hists_[i].rc) && dir == motion_hists_[i].dir) {
			continue;
		}

		if (is_cross(curr, motion_hists_[i].rc)) {
			hist.push_back(motion_hists_[i]);
		}

		curr = motion_hists_[i].rc;
	}
	return hist;
}

bool Detect::is_target(const cv::Rect &rc, const Dir &dir)
{
	if (motion_hist_delay_ < 0.0) {
		return true;
	}

	/** 根据活动历史，判断是否为有效的站立动作

			一般人站立时，需要首先有个俯身，再站立的动作:   安静-下-上上
			坐下时，会先下降，然后有个俯身，再直起身子： 下下-下-上
			有的学生在座位上，会先趴下，再抬起身子 ...: 下-上
			走动到座位坐下：应该先有左右，在俯身，最后有个上：左/右-下下-上
	 */

	std::vector<Motions> hist = get_nearby_motion_hist(rc, dir);

	if (dir == UP) {
		/** FIXME: 因为光流很不精确，根据外接矩形就差的更大了，

				这里做个很简单的判断，要求 rc 的 y 必须大于最后的“下”的矩形的 y
		 */
		int last_down_top = video_height_-1;
		for (size_t i = 0; i < hist.size(); i++) {
			if (hist[i].dir == DOWN) {
				if (hist[i].rc.y < last_down_top) {
					last_down_top = hist[i].rc.y;
				}
			}
		}

		if (debug_ && last_down_top < video_height_-1) {
			cv::line(origin_, cv::Point(rc.x, last_down_top), cv::Point(rc.x+rc.width, last_down_top), cv::Scalar(0, 0, 255), 3);
			cv::rectangle(origin_, rc, cv::Scalar(0, 255, 0), 3);
		}

		bool target = rc.y < last_down_top;

		if (debug_ && !target) {
			cv::rectangle(origin_, rc, cv::Scalar(255, 255, 255), 5);
		}

		if (!target) {
			log("\t%s: NOT a Target ??? last_down_top=%d, curr.y=%d\n", __FUNCTION__, last_down_top, rc.y);
		}

		return target;
	}

	return true;
}

void Detect::update_motion_hist(const std::vector<cv::Rect> &rcs, const std::vector<int> &dirs)
{
	/** 保留最后一段时间内，各种活动矩形 */
	for (size_t i = 0; i < rcs.size(); i++) {
		Motions m = { rcs[i], (Dir)dirs[i], curr_stamp_ };
		motion_hists_.push_back(m);
	}
}

void Detect::remove_timeouted_motion_hist()
{
	TooooOldofMotionHist t(curr_stamp_, motion_hist_delay_);
	motion_hists_.erase(std::remove_if(motion_hists_.begin(), motion_hists_.end(), t), motion_hists_.end());
}

// FIXME: 保存gray的hu moment，看看是否有明显的特征 ???
static void save_humoment(const char *fname, std::vector<cv::Mat> &grays)
{

	FILE *fp = fopen(fname, "at");
	if (fp) {
		fprintf(fp, "=============================\n");

		for (size_t i = 0; i < grays.size(); i++) {
			cv::Moments m = cv::moments(grays[i]);
			double hu[7];
			cv::HuMoments(m, hu);

			fprintf(fp,"[%d-%d]: %.7f, %.7f, %.7f, %.7f, %.7f, %.7f, %.7f\n",
				grays[i].cols, grays[i].rows, log(hu[0]), log(hu[1]), log(hu[2]), log(hu[3]), log(hu[4]), log(hu[5]), log(hu[6]));
		}
		fclose(fp);
	}
}

bool Detect::is_far(const cv::Rect &rc)
{
	return CENTER_Y(rc) / video_height_ < far_ratio_;
}

void Detect::try_object_detect()
{
	// 进行“头肩”识别 ...
	std::vector<Target>::iterator it;
	int n = 0;
	for (it = targets_.begin(); it != targets_.end(); n++) {
		// 如目标已经存在，并且没有成功识别过 ..，最多识别od_max_times次 ..
		if (curr_stamp_ - it->stamp > 0.1 && it->detecte_state_ <= 0 && it->detecte_state_ > 0 - od_max_times_) {
			std::vector<cv::Rect> faces;
			cv::Rect rc = it->pos;

			if (face_detect_far_ || !is_far(rc)) {
				/// FIXME: 实际发现，需要稍稍向上一点儿，否则头顶不完整 ...
				int top_del = rc.height / 5;
				if (rc.y >= top_del) {
					rc.y -= top_del;
					rc.height += top_del;
				}

				// 左右扩展 ??
				int lr_del = rc.width / 12;
				if (rc.x >= lr_del) {
					rc.x -= lr_del;
					rc.width += lr_del;
				}

				if (rc.x + rc.width + lr_del < origin_.cols) {
					rc.width += lr_del;
				}

				// 其实应该仅仅检测 rc 的上半部分，这里才包含头肩 ... 不过，对于很远的目标，就不好说了 :(
				// rc.height /= 2;

				cv::Mat roi = origin_(rc).clone();
				if (od_->has_faces(roi, faces)) {
					if (skin_) {
						/** 找人脸肤色，头发 ...

							累计 face 中的肤色面积，如果大于头肩框的 skin_head_ratio_ 就认为该头肩框有效
							*/
						for (std::vector<cv::Rect>::iterator itface = faces.begin(); itface != faces.end(); ) {
							cv::Rect face = *itface;
							std::vector<std::vector<cv::Point> > skins = skin_->find_skin_contours(roi(face));

							double areas = 0;	//

							for (size_t i = 0; i < skins.size(); i++) {
								areas += cv::contourArea(skins[i]);

								if (debug_img_) {
									// 加上偏移 face
									for (size_t x = 0; x < skins[i].size(); x++) {
										skins[i][x].x += face.x;
										skins[i][x].y += face.y;
									}
								}
							}

							if (debug_img_) {
								// 画到 roi 保存 ..
								cv::drawContours(roi, skins, -1, cv::Scalar(0, 0, 255));
							}

							// FIXME: 如果 skins area 大于头肩的 skin_head_ratio_ 就认为该头肩有效？？？
							if (areas >= face.area() * skin_head_ratio_) {
								++itface;
							}
							else {
								log("\t%s: skin NOT exist in [%d,%d, %d,%d]\n", __FUNCTION__,
									rc.x+face.x, rc.y+face.y, face.width, face.height);

								itface = faces.erase(itface);
							}
						}
					}

					if (faces.empty()) {
						it->detecte_state_--;	// 继续
					}
					else {
						it->detecte_state_ = 1;	// 已经找到目标 ...
						log("\t%s: found face in [%d,%d, %d,%d]\n", __FUNCTION__,
							it->pos.x, it->pos.y, it->pos.width, it->pos.height);

						if (debug_) {
							for (std::vector<cv::Rect>::const_iterator it = faces.begin(); it != faces.end(); ++it) {
								cv::rectangle(roi, *it, cv::Scalar(0, 255, 255));
							}
						}
					}
				}
				else {
					it->detecte_state_--;	// 继续 ...
				}

				if (debug_img_) {
					char fname[128];
					if (it->detecte_state_ == 1)	// 已经确认目标 ...
						snprintf(fname, sizeof(fname), "detlog/target_%u_%d_ok.jpg", st_cnt_, n);
					else
						snprintf(fname, sizeof(fname), "detlog/target_%u_%d_%d.jpg", st_cnt_, n, 0 - it->detecte_state_);

					//cv::imwrite(fname, roi);
				}
			}
			else {
				// 后排，不进行头肩检测
				it->detecte_state_ = 1;
			}
		}

		if (it->detecte_state_ <= 0 - od_max_times_) {
			// 需要删掉 ...
			it = targets_.erase(it);
			log("\tobject detection failure!!! just remove [%d,%d, %d-%d], chk cnt=%d\n",
				it->pos.x, it->pos.y, it->pos.width, it->pos.height, od_max_times_);
		}
		else {
			++it;
		}
	}
}

void Detect::detect(cv::Mat &origin, std::vector<cv::Rect> &targets, int &flipped_index)
{
	//ost::MutexLock al(lock_);
	curr_stamp_ = now();
	double curr = curr_stamp_;

	++st_cnt_;
	origin_ = origin;

	// 最后一段时间内的帧率统计
	if (st_cnt_ % 100 == 0) {
		fps_ = 100 / (curr - st_seg_);

		log("DEBUG: STAT: avg: %.3ffps, last: %.3ffps\n",
			st_cnt_ / (curr - st_begin_), fps_);

		st_seg_ = curr;
	}

	cv::cvtColor(origin, gray_curr_, cv::COLOR_BGR2GRAY);
	//cv::filter2D(gray_curr_, gray_curr_, gray_curr_.depth(), ker_);

	if (first_) {
		gray_prev_ = gray_curr_.clone();
		first_ = false;

		// 保存一帧探测图像为文件
		//cv::imwrite("detlog/det_s_origin.jpg", origin);
	}
	if (save_history_) {
		// 保存历史
		cv::Mat &mx = frames_history_.prev();
		gray_curr_.copyTo(mx);
		++frames_history_;
	}
    //printf("flipped:%d\n",flipped_);
	if (flipped_) {
		flipped_index = detect0(st_cnt_, origin, gray_prev_, gray_curr_);
	}
	else {
		std::vector<int> dirs;
		std::vector<cv::Rect> rcs = detect0(st_cnt_, origin, gray_prev_, gray_curr_, dirs);

		if (rcs.size() > 0 || targets_.size() > 0) {
			log("================== %u ===================\n", st_cnt_);
		}

		if (rcs.size() > 0) {
			log("M: detect FOUND %u motions\n", rcs.size());
			for (size_t i = 0; i < rcs.size(); i++) {
				log("\t#%u, MOTION: %s, [%d,%d,  %d-%d]\n",
					i, DirDesc[dirs[i]], rcs[i].x, rcs[i].y, rcs[i].width, rcs[i].height);
			}
			log("\n");
		}

		if (motion_hist_delay_ > 0) {
			// update motion hist
			update_motion_hist(rcs, dirs);
			remove_timeouted_motion_hist();
		}

		if (debug_img2_) {
			// 保存探测矩形对应的原始图像 ...
			char fname[128];
			const char *dir_str[4] = { "right", "down", "left", "up" };
			for (size_t i = 0; i < rcs.size(); i++) {
				snprintf(fname, sizeof(fname), "detlog/%s_%u_%d.jpg", dir_str[dirs[i]], st_cnt_, i);
				cv::Mat img = origin(rcs[i]);
				//cv::imwrite(fname, img);
			}
		}

		merge_rcs(rcs, dirs, curr);	// 合并 targets

		// 删除超时的目标 ..
		TooOld old(curr, max_duration_);
		targets_.erase(std::remove_if(targets_.begin(), targets_.end(), old), targets_.end());

		if (od_) {
            printf("object\n");
			try_object_detect();
		}

		targets = current_targets();
		if (targets_.size() > 0) {
			log("T: there are %u targets:\n", targets_.size());
			for (size_t i = 0; i < targets_.size(); i++) {
				cv::Rect &rc = targets_[i].pos;
				log("\t#%u: TARGET: [%d,%d,  %d-%d], duration=%.1f, odstate=%d, update_cnt=%d, area=%d\n",
					i, rc.x, rc.y, rc.width, rc.height,
					curr_stamp_ - targets_[i].stamp,
					targets_[i].detecte_state_,
					targets_[i].updated_cnt,
					rc.area());
			}
			log("\n");
		}

		if (debug_) {
			// 四个方向的活动 ...
			const cv::Scalar colors[4] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255)  };
			for (size_t i = 0; i < rcs.size(); i++) {
				cv::rectangle(origin, rcs[i], colors[dirs[i]], 1);
			}
		}
	}

	cv::swap(gray_curr_, gray_prev_);

	if (debug_) {
		if (flipped_) {
			char buf[64];
			snprintf(buf, sizeof(buf), "#%d", flipped_index);
			cv::putText(origin, buf, cv::Point(0, 30), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
		}
		else {
			draw_area_thres_info(origin);

			for (size_t i = 0; i < targets_.size(); i++) {
				// 确认的目标
				cv::rectangle(origin, targets_[i].pos, cv::Scalar(0, 0, 255), 2);

				// 如果启用od，并且确认，则画圆圈祝贺 ...
				if (targets_[i].detecte_state_ == 1) {
					cv::circle(origin, cv::Point(targets_[i].pos.x+targets_[i].pos.width/2, targets_[i].pos.y+targets_[i].pos.height/2),
						5, cv::Scalar(0, 0, 255), 2);
				}
			}
		}

		char buf[64];
		snprintf(buf, sizeof(buf), "%.03f", fps_);
	//	cv::putText(origin, buf, cv::Point(0, 60), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));

	//	cv::imshow("student", origin);

		if (wait_key_) {
			//cv::waitKey(1);
		}
	}
}

void Detect::draw_area_thres_info(cv::Mat &origin)
{
	int ystep = 20;
	for (int i = ystep; i < origin.rows; i+=ystep) {
		cv::line(origin, cv::Point(0, i), cv::Point(origin.cols, i), cv::Scalar(255, 255, 255));
		char info[64];
		snprintf(info, sizeof(info), "%.0f", factor_y(i)*factor_y(i)*thres_area_);
		cv::putText(origin, info, cv::Point(0, i), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
	}
}

/** m0 为 CV_8U, 数据或者是0，或者是1
	size 为小块的大小， 返回 Mat 的大小为 m0 / size [ +1 ]
	ratio：当小块中1个数目大于总数的 ratio 倍时，认为是 1
	返回的 mat
 */
static cv::Mat find_clusters(const cv::Mat &m0, const cv::Size &size, const double ratio = 0.5)
{
	cv::Mat integrated;
	cv::integral(m0, integrated, CV_32S);	// 积分图

	int dx = size.width, dy = size.height;

	int cx = (m0.cols + dx-1) / dx, cy = (m0.rows + dy-1) / dy;

	cv::Mat ret(cy, cx, CV_8U);

	for (int y = 0; y < m0.rows; y += dy) {
		if (y >= m0.rows) dy = m0.rows - (y-1) * dy;
		dx = size.width;

		unsigned char *p = ret.ptr<unsigned char>(y/dy);

		for (int x = 0; x < m0.cols; x += dx) {
			if (x >= m0.cols) dx = m0.cols - (x-1) * dx;

			cv::Rect mrc(x, y, dx, dy);
			int a = integrated.at<int>(y, x);
			int b = integrated.at<int>(y+dy, x);
			int c = integrated.at<int>(y+dy, x+dx);
			int d = integrated.at<int>(y, x+dx);

			int sum = a + c - b - d;
			if (sum > ratio * (dx * dy)) {
				p[x/dx] = 255;
			}
			else {
				p[x/dx] = 0;
			}
		}
	}

	return ret;
}

std::vector<cv::Rect> Detect::find_clusters(const cv::Mat &m0, const cv::Size &size0, const double threshold0, int stepx, int stepy) const
{
	/** m 为稠密光流做距离阈值后的结果，type() == 8UC1
		对 m 做积分图，然后使用滑动窗口移动监测
	 */

	std::vector<cv::Rect> rcs;

	cv::Mat integration, m;

#if 1
	cv::threshold(m0, m, 0.5, 1.0, cv::THRESH_BINARY);

#if 1
	cv::Mat m1 = ::find_clusters(m, cv::Size(1, 1));
	//cv::imshow("ii", m1);
#else
	cv::integral(m, integration, CV_32S);

	cv::imshow("integral image", integration);

#define DX 5
#define DY 5

	int dx = DX, dy = DY;
	for (int y = 0; y < m.rows; y += dy) {
		if (y + dy >= m.rows) dy = m.rows - y;
		dx = DX;

		for (int x = 0; x < m.cols; x += dx) {
			if (x + dx >= m.cols) dx = m.cols - x;
			int a = integration.at<int>(y, x);
			int b = integration.at<int>(y+dy, x);
			int c = integration.at<int>(y+dy, x+dx);
			int d = integration.at<int>(y, x+dx);

			int sum = a + c - b - d;

			if (sum > dx * dy / 2) {
				rcs.push_back(cv::Rect(x, y, dx, dy));
			}
		}
	}
#endif
#else
	m0.convertTo(m, CV_32F);
	cv::integral(m, integration, CV_32F);

	cv::imshow("original dis", m);
	cv::imshow("integral image", integration);

	double low, high;
	cv::minMaxIdx(m, &low, &high);

	for (int y = 0; y < integration.rows; y += stepy) {
		int sy = size0.height * factor_y(y);	// FIXME: 总是用正方形 ...
		int sx = sy;

		if (y + sy > integration.rows)
			sy = integration.rows - y - 1;

		double threshold = factor_y(y) * threshold0;

		for (int x = 0; x+sx <= integration.cols; x += stepx) {
			float a = integration.at<float>(cv::Point(x, y));
			float b = integration.at<float>(cv::Point(x, y+sy));
			float c = integration.at<float>(cv::Point(x+sx, y+sy));
			float d = integration.at<float>(cv::Point(x+sx, y));

			float sum = a + c - b - d;
			if (sum > 100) {
				fprintf(stderr, "sum: %.3f\n", sum);
			}

			if (sum > threshold) {
				rcs.push_back(cv::Rect(x, y, sx, sy));
			}
		}
	}
#endif
	return rcs;
}
