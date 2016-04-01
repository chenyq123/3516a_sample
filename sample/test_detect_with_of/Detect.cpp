#include "Detect.h"
#include "utils.h"

Detect::Detect(KVConfig *cfg)
    : cfg_(cfg)
{
    debug_ = atoi(cfg->get_value("debug", "0")) == 1;
    log_init = log_init_file;
    log = log_file;

    log_init(0);

    od_ = new ObjectDetect(cfg_);

    reset();
}

Detect::~Detect()
{
}

void Detect::reset()
{
    cnt_ = 0;
    max_duration_ = atof(cfg_->get_value("max_duration", "15.0"));
    waiting_ = atof(cfg_->get_value("max_wait", "2.0"));

    debug_ = atoi(cfg_->get_value("debug", "0")) == 1;
    log_init = log_init_file;
    log = log_file;

    log("Detect config:\n");
    log("\tmax_duration: %.2f\n", max_duration_);
    log("\tmax_wait: %.2f\n", waiting_);
    log("\n");

    masked_ = build_mask(mask_);

    standups_.clear();
    gray_prev_.release();
    gray_prev_.cols = 0;
    gray_prev_.rows = 0;
}

void Detect::detect(const cv::Mat &origin, Detect::RCS &standups)
{
    stamp_ = now();
    cnt_++;
    origin_ = origin;

    if (masked_) {
        cv::bitwise_and(origin, mask_, origin);
    }

    cv::cvtColor(origin, gray_curr_, cv::COLOR_BGR2GRAY);
    if (gray_prev_.cols == 0) {
        gray_prev_ = gray_curr_.clone();
    }

    RCS motions;
    std::vector<Dir> dirs;
    detect(motions, dirs);

    cv::swap(gray_prev_, gray_curr_);

    assert(dirs.size() == motions.size());

    if (dirs.size() > 0) {
        int a = 0;
    }

    // 处理  motions/dirs 对 standups_ 的影响 ...
    for (size_t i = 0; i < dirs.size(); i++) {
        STANDUPS::iterator it = find_crossed_target(motions[i]);
        if (it == standups_.end()) {
            // 新的活动区域 ...
            // 如果是上升，则创建一个新的 standup
            if (dirs[i] == UP) {
                Standup s;
                s.enable_od = od_->enabled();

                std::vector<cv::Rect> faces;
                if (s.enable_od) {
                    // 在 motions[i].pos 的上半部分进行头肩识别 ...
                    cv::Rect roi = motions[i];
                    roi.height /= 2;    // FIXME:
                    faces = od_->detect(gray_curr_, roi);
                    if (faces.empty()) {
                        log("WRN: %u: 新目标：%s, 但是找不到头肩，失败\n", cnt_, pos2str(motions[i]).c_str());
                        continue;
                    }
                    else {
                        s.face = faces[0];
                    }
                }

                s.pos = motions[i];
                s.stamp = stamp_;
                s.waiting = false;
                standups_.push_back(s);

                if (s.enable_od) {
                    log("INFO: %u: 新目标：%s, 头肩位置: %s, %lu\n", cnt_, pos2str(s.pos).c_str(), pos2str(s.face).c_str(), faces.size());
                }
                else {
                    log("INFO: %u: 新目标：%s\n", cnt_, pos2str(s.pos).c_str());
                }
            }
        }
        else if (it->waiting) {
            // 等待期，直接忽略新的活动 ...
            log("WRN: %u: 忽略 Waiting 活动, %s: pos=%s\n", cnt_, DirDesc[dirs[i]], pos2str(it->pos).c_str());
        }
        else {
            // 进入等待期 ...
            it->waiting = true;
            it->stamp = stamp_;
            log("INFO: %u: 目标消失，进入Waiting，%s: pos=%s\n", cnt_, DirDesc[dirs[i]], pos2str(it->pos).c_str());
        }
    }

    // 检查超时 ...
    for (STANDUPS::iterator it = standups_.begin(); it != standups_.end();) {
        if (stamp_ - it->stamp > max_duration_) {
            log("WRN: %u: 删除超时站立: %s, max_duration=%.2f\n",
                cnt_, pos2str(it->pos).c_str(), max_duration_);
            it = standups_.erase(it);   // 超时删除 ...
        }
        else if (it->waiting) {
            if (stamp_ - it->stamp > waiting_) {
                log("INFO: %u: 删除waiting区域: %s\n", cnt_, pos2str(it->pos).c_str());
                it = standups_.erase(it);  // 删除 waiting ...
            }
            else {
                ++it;
            }
        }
        else {
            standups.push_back(it->pos);  // 有效目标 ...
            ++it;
        }
    }
}
