#include "DetectWithOf.h"

DetectWithOf::DetectWithOf(KVConfig *cfg)
    : Detect(cfg)
    , dm_(cfg)
{
    cfg->set_value("video_width", "960");
    cfg->set_value("video_height", "540");

    reset();
}

DetectWithOf::~DetectWithOf()
{
}

void DetectWithOf::reset()
{
    Detect::reset();
    dm_.reset();

    next_tid_ = 1;

    if (debug_) {
        //cv::namedWindow("dwof");
    }
    else {
        //cv::destroyWindow("dwof");
    }

    far_y_ = (float)atof(cfg_->get_value("pd_far_y", "80"));
    near_y_ = (float)atof(cfg_->get_value("pd_near_y", "320"));

    far_width_ = (float)atof(cfg_->get_value("pd_far_width", "20")), near_width_ = (float)atof(cfg_->get_value("pd_near_width", "80"));
    polyfit_line(cv::Point2f(far_y_, far_width_), cv::Point2f(near_y_, near_width_), a_width_, b_width_);

    far_dis_[UP] = (float)atof(cfg_->get_value("pd_far_dis_up", "15")), near_dis_[UP] = (float)atof(cfg_->get_value("pd_near_dis_up", "30"));
    far_dis_[DOWN] = (float)atof(cfg_->get_value("pd_far_dis_down", "10.0")), near_dis_[DOWN] = (float)atof(cfg_->get_value("pd_near_dis_down", "40"));
    far_dis_[LEFT] = (float)atof(cfg_->get_value("pd_far_dis_left", "20.0")), near_dis_[LEFT] = (float)atof(cfg_->get_value("pd_near_dis_left", "50"));
    far_dis_[RIGHT] = (float)atof(cfg_->get_value("pd_far_dis_right", "20.0")), near_dis_[RIGHT] = (float)atof(cfg_->get_value("pd_near_dis_right", "50"));
    polyfit_line(cv::Point2f(far_y_, far_dis_[UP]), cv::Point2f(near_y_, near_dis_[UP]), a_dis_[UP], b_dis_[UP]);
    polyfit_line(cv::Point2f(far_y_, far_dis_[DOWN]), cv::Point2f(near_y_, near_dis_[DOWN]), a_dis_[DOWN], b_dis_[DOWN]);
    polyfit_line(cv::Point2f(far_y_, far_dis_[LEFT]), cv::Point2f(near_y_, near_dis_[LEFT]), a_dis_[LEFT], b_dis_[LEFT]);
    polyfit_line(cv::Point2f(far_y_, far_dis_[RIGHT]), cv::Point2f(near_y_, near_dis_[RIGHT]), a_dis_[RIGHT], b_dis_[RIGHT]);

    up_angle_ = atoi(cfg_->get_value("up_angle", "110"));

    log("DetectWithOf config:\n");
    log("\tpd_far_y, dof_near_y: %.0f, %.0f\n", far_y_, near_y_);
    log("\tpd_far_width, dof_near_width: %.0f, %.0f\n", far_width_, near_width_);
    log("\tpd_far_dis_DDD: %.2f, %.2f, %.2f, %.2f\n", far_dis_[RIGHT], far_dis_[DOWN], far_dis_[LEFT], far_dis_[RIGHT]);
    log("\tpd_near_dis_DDD: %.2f, %.2f, %.2f, %.2f\n", near_dis_[RIGHT], near_dis_[DOWN], near_dis_[LEFT], near_dis_[RIGHT]);
    log("\tup_angle: %d\n", up_angle_);
    log("\n");

    targets_.clear();
}

void DetectWithOf::detect(Detect::RCS &motions, std::vector<Dir> &dirs)
{
    /**
        跟踪所有 Targets
        当某个Targets is_stopped() 时，开始评估该 Target 的活动属性，决定是否为有效活动 ...
        如果是孤立的帧差矩形，则创建新的 Target 对象..
     */

    cv::Mat img;
    if (debug_) {
        img = cv::Mat::zeros(origin_.rows, origin_.cols, origin_.type());
    }
    //// 长时间跟踪对象 ...
    //for (std::vector<Target>::iterator it = long_timed_targets_.begin(); it != long_timed_targets_.end();) {
    //  if (!it->track(gray_prev_, gray_curr_, stamp_)) {
    //      it = long_timed_targets_.erase(it);
    //  }
    //  else {
    //      if (debug_) {
    //          it->debug_draw_paths(img, cv::Scalar(0, 64, 0));
    //      }
    //      ++it;
    //  }
    //}

    // 跟踪老的 ...
    std::vector<Target> stopped;
    for (std::vector<Target>::iterator it = targets_.begin(); it != targets_.end();) {
        if (!it->track(gray_prev_, gray_curr_, stamp_)) {
            log("WRN: %u: target 跟踪失败，放弃: %s\n", cnt_, it->descr().c_str());
            it = targets_.erase(it);
        }
        else {
            if (it->is_stopped()) {
                stopped.push_back(*it);
                long_timed_targets_.push_back(*it); // 长时间跟踪对象 ...
                it = targets_.erase(it);
            }
            else {
                if (debug_) {
                    cv::Scalar s = cv::Scalar(0, 64, 0);
                    //it->debug_draw_paths(img, cv::Scalar(0, 64, 0));    // 暗绿色 ..
                    it->debug_draw_paths(img, s);    // 暗绿色 ..
                }
                ++it;
            }
        }
    }

    // 处理停止活动的 ...
    for (std::vector<Target>::const_iterator it = stopped.begin(); it != stopped.end(); ++it) {
        cv::Rect pos;
        Dir dir;
        int rc = estimate(*it, pos, dir);
        if (E_OK == rc) {
            if (debug_) {
                cv::rectangle(img, it->pos(), cv::Scalar(0, 255, 0), 2);
                //imwrite("save.bmp", img);
                //printf("writed\n");
            }

            motions.push_back(pos);
            dirs.push_back(dir);
        }
        else {
            switch (rc) {
            case E_NotEnoughLayers:
                break;

            case E_NotEnoughPaths:
                break;

            case E_NotEnoughDistance:
                break;
            }
        }
    }

    // 新的来自不相交的帧差矩形可以创建新的跟着区域 ...
    RCS rcs = dm_.get_motions(gray_prev_, gray_curr_);
    for (RCS::const_iterator it = rcs.begin(); it != rcs.end(); ++it) {
        if (too_small(*it)) {
            continue;
        }

        std::vector<Target>::iterator x = find_matched_target(*it);
        if (x == targets_.end()) {
            Target t;
            if (t.init(cfg_, next_tid_++, *it, gray_curr_, stamp_)) {
                targets_.push_back(t);
            }
        }
    }

    if (debug_) {
        char info[64];

        snprintf(info, sizeof(info), "%lu", cnt_);
        cv::putText(img, info, cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 255, 255));

        cv::Point2f p1(20, far_y_), p2((float)origin_.cols, far_y_);
        cv::line(img, p1, p2, cv::Scalar(255, 0, 0));

        snprintf(info, sizeof(info), "%.0f,%.0f,%.0f", far_y_, far_width_, far_dis_[UP]);
        cv::putText(img, info, p1, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 255, 255));

        p1.x = 30, p1.y -= 30;
        p2.x = 30 + far_width_, p2.y = p1.y;
        cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 2);
        p1.x = 20, p1.y = near_y_, p2.x = (float)origin_.cols, p2.y = near_y_;
        cv::line(img, p1, p2, cv::Scalar(255, 0, 0));

        snprintf(info, sizeof(info), "%.0f,%.0f,%.0f", near_y_, near_width_, near_dis_[UP]);
        cv::putText(img, info, p1, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 255, 255));
        p1.x = 30, p1.y -= 30;
        p2.x = 30 + near_width_, p2.y = p1.y;
        cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 2);

        //cv::imshow("dwof", img);
    }
}

bool DetectWithOf::too_small(const cv::Rect &rc) const
{
    // 根据前后位置，去掉太小的活动 ...
    double want = a_width_ * (rc.y + rc.height / 2) + b_width_;
    if (want < far_width_) {
        want = far_width_;
    }

    return rc.width < want;
}

int DetectWithOf::estimate(const Target &target, cv::Rect &pos, Dir &dir) const
{
    /** 一个有效的活动，至少拥有10条轨迹，并且每条轨迹至少5层
     */

    if (target.layers() < 5) {
        return E_NotEnoughLayers;
    }

    if (target.get_path_cnt() < 10) {
        return E_NotEnoughPaths;
    }

    std::vector<Target::PATH> paths = target.get_sorted_paths();
    double angles[5];   // 统计前五条
    double diss[5];

    for (int i = 0; i < 5; i++) {
        cv::Point2f front = paths[i].front(), back = paths[i].back();
        angles[i] = calc_angle(front, back);
        diss[i] = distance(front, back);
    }

    double mean_angle = (angles[0] + angles[1] + angles[2] + angles[3] + angles[4]) / 5.0;
    double mean_dis = (diss[0] + diss[1] + diss[2] + diss[3] + diss[4]) / 5.0;

    int half_up = up_angle_ / 2;
    int up_left = 270 - half_up, up_right = 270 + half_up;
    int down_left = 90 + half_up, down_right = 90 - half_up;

    if (mean_angle >= up_left && mean_angle <= up_right) {
        dir = UP;
    }
    else if (mean_angle >= down_right && mean_angle <= down_left) {
        dir = DOWN;
    }
    else if (mean_angle > down_left && mean_angle < up_left) {
        dir = LEFT;
    }
    else {
        dir = RIGHT;
    }

    // 距离大于该位置阈值，则生效 ..
    pos = target.pos();
    double dis_want = a_dis_[dir] * (pos.y + pos.height / 2) + b_dis_[dir];
    if (mean_dis < dis_want) {
        if (mean_dis * 1.5 > dis_want) {
            // NOTE: 防止打印过多的 ...
            log("DBG: %u: 距离不够的活动：%s, %s, want=%.2f, got=%.2f\n",
                cnt_, DirDesc[dir], target.descr().c_str(), dis_want, mean_dis);
        }
        return E_NotEnoughDistance;
    }

    log("DBG: %u: 有效活动：%s, %s, want=%.2f, got=%.2f\n",
        cnt_, DirDesc[dir], target.descr().c_str(), dis_want, mean_dis);

    return E_OK;
}
