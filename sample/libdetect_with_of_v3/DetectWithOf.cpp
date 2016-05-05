#include "DetectWithOf.h"
#include <assert.h>

DetectWithOf::DetectWithOf(KVConfig *cfg)
    : Detect(cfg)
    , dm_(cfg)
{
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

    support_long_time_tracking_ = false;// 尚未实现 .. = atoi(cfg_->get_value("pd_support_long_time_tracking", "0")) == 1; // 是否采用 detect2 模式 ..
    target_min_dis_5frames_ = atof(cfg_->get_value("pd_target_min_dis_5frames", "5.0"));
    target_min_pts_ = atoi(cfg_->get_value("pd_target_min_pts", "10"));
    track_once_ = atoi(cfg_->get_value("pd_track_once", "1")) == 1;

    far_y_ = (float)atof(cfg_->get_value("pd_far_y", "80"));
    near_y_ = (float)atof(cfg_->get_value("pd_near_y", "320"));

    n_paths_for_stats_ = atoi(cfg_->get_value("pd_n_paths_for_stats", "8"));
    if (n_paths_for_stats_ < 2) n_paths_for_stats_ = 2;
    if (n_paths_for_stats_ > 10) n_paths_for_stats_ = 10;

    check_larger_ = atoi(cfg_->get_value("pd_check_larger", "1")) == 1;
    large_threshold_ = atoi(cfg_->get_value("pd_larger_threshold", "86400"));

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

    max_feature_pts_ = atoi(cfg_->get_value("pd_feature_max", "200"));
    feature_quality_level_ = atof(cfg_->get_value("pd_feature_quality_level", "0.02"));

    log("DetectWithOf config:\n");
    log("\tpd_track_once: %d\n", track_once_);
    log("\tpd_support_long_time_tracking: %d\n", support_long_time_tracking_);
    log("\tpd_target_min_pts: %d\n", target_min_pts_);
    log("\tpd_target_min_dis_5frames: %.2f\n", target_min_dis_5frames_);
    log("\tpd_far_y, pd_near_y: %.0f, %.0f\n", far_y_, near_y_);
    log("\tpd_far_width, pd_near_width: %.0f, %.0f\n", far_width_, near_width_);
    log("\tpd_far_dis_DDD: %.2f, %.2f, %.2f, %.2f\n", far_dis_[RIGHT], far_dis_[DOWN], far_dis_[LEFT], far_dis_[UP]);
    log("\tpd_near_dis_DDD: %.2f, %.2f, %.2f, %.2f\n", near_dis_[RIGHT], near_dis_[DOWN], near_dis_[LEFT], near_dis_[UP]);
    log("\tup_angle: %d\n", up_angle_);
    log("\tpd_check_larger: %d, pd_larger_threshold: %d\n", check_larger_, large_threshold_);
    log("\tpd_n_paths_for_stats: %d\n", n_paths_for_stats_);
    log("\tpd_feature_max: %d, pd_feature_quality_level: %.4f\n", max_feature_pts_, feature_quality_level_);
    log("\n");

    targets_.clear();
}

void DetectWithOf::detect(Detect::RCS &motions, std::vector<Dir> &dirs)
{
    /** 基本思路是：
            1. 帧差得到活动区域，扔掉太小的；
            2. 从活动区域中找“跟踪点”；
            3. 记录跟踪点轨迹，如果有重合帧差矩形，则扔掉；
            4. 当轨迹的最后几帧活动很小时，则认为目标停止活动了，开始判断轨迹的运动方向，以及是否达到触发阈值；
            5. 返回达到活动（距离）阈值的motions和direction
    */

    if (debug_) {
        debug_img_ = cv::Mat::zeros(origin_.rows, origin_.cols, origin_.type());
    }

    printf("DetectWithof line:%d, time:%d\n", __LINE__, GetTickCount());
    // 跟踪老的 ...
    std::vector<Target> stopped = track_targets();
    printf("DetectWithof line:%d, time:%d\n", __LINE__, GetTickCount());

    // 处理停止活动的 ...
    for (std::vector<Target>::const_iterator it = stopped.begin(); it != stopped.end(); ++it) {
        cv::Rect pos;
        Dir dir;
        int rc = estimate(*it, pos, dir);
        if (E_OK == rc) {
            if (debug_) {
                cv::rectangle(debug_img_, it->pos(), cv::Scalar(0, 255, 0), 2);
            }

            if (debug_) {
                debug_save_motion(*it, pos, dir);
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
    printf("DetectWithof line:%d, time:%d\n", __LINE__, GetTickCount());

    // 新的来自不相交的帧差矩形可以创建新的跟着区域 ...
    process_diff_motions();
    printf("DetectWithof line:%d, time:%d\n", __LINE__, GetTickCount());

    if (debug_) {
        char info[64];

        snprintf(info, sizeof(info), "%lu", cnt_);
        cv::putText(debug_img_, info, cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 255, 255));

        cv::Point2f p1(20, far_y_), p2((float)origin_.cols, far_y_);
        cv::line(debug_img_, p1, p2, cv::Scalar(255, 0, 0));

        snprintf(info, sizeof(info), "%.0f,%.0f,%.0f", far_y_, far_width_, far_dis_[UP]);
        cv::putText(debug_img_, info, p1, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 255, 255));

        p1.x = 30, p1.y -= 30;
        p2.x = 30 + far_width_, p2.y = p1.y;
        cv::line(debug_img_, p1, p2, cv::Scalar(255, 0, 0), 2);
        p1.x = 20, p1.y = near_y_, p2.x = (float)origin_.cols, p2.y = near_y_;
        cv::line(debug_img_, p1, p2, cv::Scalar(255, 0, 0));

        snprintf(info, sizeof(info), "%.0f,%.0f,%.0f", near_y_, near_width_, near_dis_[UP]);
        cv::putText(debug_img_, info, p1, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 255, 255));
        p1.x = 30, p1.y -= 30;
        p2.x = 30 + near_width_, p2.y = p1.y;
        cv::line(debug_img_, p1, p2, cv::Scalar(255, 0, 0), 2);

        //cv::imshow("dwof", debug_img_);
    }
}

void DetectWithOf::detect2(Detect::RCS &standups)
{
    //throw new std::exception("NOT IMPLE");
#if 0
    /** 基本思路是：
            1. 帧差得到活动区域，扔掉太小的；
            2. 从活动区域中找“跟踪点”；
            3. 记录跟踪点轨迹，如果有重合帧差矩形，则扔掉；
            4. 当轨迹的最后几帧活动很小时，则认为目标停止活动了，开始判断轨迹的运动方向，以及是否达到触发阈值；
            5. 保存“向上”并且超过阈值的轨迹的最后几帧，继续跟踪，直到这些点子又出现较大活动后，说明“站立”取消 ...
    */

    if (debug_) {
        debug_img_ = cv::Mat::zeros(origin_.rows, origin_.cols, origin_.type());
    }

    // 从 targets_ 中得到“停止活动”的，如果是向上，则保存到 standups_ 中 ..
    std::vector<Target> stopped = track_targets();
    for (std::vector<Target>::iterator it = targets_.begin(); it != targets_.end(); ++it) {
        cv::Rect pos;
        Dir dir;
        int rc = estimate(*it, pos, dir);
        if (E_OK == rc) {
            if (debug_) {
                cv::rectangle(debug_img_, it->pos(), cv::Scalar(0, 255, 0), 2);
            }

            if (dir == UP) {
                if (it->cut()) {
                    standups_.push_back(*it);
                }
            }
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
    process_diff_motions();

    if (debug_) {
        // TODO: draw debug info
    }
#endif //
}

void DetectWithOf::process_diff_motions()
{
    RCS rcs = dm_.get_motions(gray_prev_, gray_curr_);
    for (RCS::const_iterator it = rcs.begin(); it != rcs.end(); ++it) {
        if (too_small(*it)) {
            continue;
        }

        /// NOTE: 避开 targets_ 和 standups_
        std::vector<Target>::iterator x = find_matched_target(*it), y = find_matched_target_from_ltt(*it);
        if (x == targets_.end() && y == standups_.end()) {
            Target t;
            if (t.init(cfg_, next_tid_++, *it, gray_curr_, stamp_, target_min_dis_5frames_, target_min_pts_, max_feature_pts_, feature_quality_level_)) {
                targets_.push_back(t);
            }
        }
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

#if 1
    // 计算前 N 条轨迹的矢量，相当于求向量和 ...
    cv::Point2f from(0, 0), to(0, 0);
    int n = std::min<int>(n_paths_for_stats_, (int)paths.size());
    for (int i = 0; i < n; i++) {
        cv::Point2f &front = paths[i].front(), &back = paths[i].back();
        to.x += back.x - front.x;
        to.y += back.y - front.y;
    }

    double mean_angle = calc_angle(from, to);
    double mean_dis = distance(from, to) / n;

#else
    double angles[5];   // 统计前五条
    double diss[5];

    for (int i = 0; i < 5; i++) {
        cv::Point2f front = paths[i].front(), back = paths[i].back();
        angles[i] = calc_angle(front, back);
        diss[i] = distance(front, back);
    }

    double mean_angle = (angles[0] + angles[1] + angles[2] + angles[3] + angles[4]) / 5.0;
    double mean_dis = (diss[0] + diss[1] + diss[2] + diss[3] + diss[4]) / 5.0;

#endif
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

std::vector<Target> DetectWithOf::track_targets()
{
    if (track_once_) {
        return track_targets_once();
    }
    else {
        std::vector<Target> stopped;

        for (std::vector<Target>::iterator it = targets_.begin(); it != targets_.end();) {
            printf("track\n");
            if (!it->track(gray_prev_, gray_curr_, stamp_)) {
                log("WRN: %u: target 跟踪失败，放弃: %s\n", cnt_, it->descr().c_str());
                it = targets_.erase(it);
            }
            else {
                if (it->is_stopped()) {
                    stopped.push_back(*it);
                    it = targets_.erase(it);
                }
                else {
                    if (debug_) {
                        cv::Scalar s(0, 128, 0);
                        it->debug_draw_paths(debug_img_, s, 30);    // 暗绿色 ..
                    }

                    // 检查是否过大 ???
                    if (check_larger_ && it->pos().area() > large_threshold_) {
                        log("WRN: %u: target TOOOOOOOOO large %s, area=%d, threshold=%d\n",
                            cnt_, it->descr().c_str(), it->pos().area(), large_threshold_);
                        it = targets_.erase(it);
                    }
                    else {
                        ++it;
                    }
                }
            }
        }

        return stopped;
    }
}

std::vector<Target> DetectWithOf::track_targets_once()
{
    /** 同时跟踪所有 target ...
     */

    std::vector<Target> stopped;

    std::vector<cv::Point2f> from, to;
    std::vector<int> segs;  // 用于标记属于哪个 target
    int off = 0;
    for (std::vector<Target>::const_iterator it = targets_.begin(); it != targets_.end(); ++it) {
        const std::vector<cv::Point2f> &pts = it->last_pts();
        off += pts.size();
        segs.push_back(off);
        from.insert(from.end(), pts.begin(), pts.end());
    }

    //assert(targets_.size() == segs.size());

    if (from.empty()) {
        return stopped;
    }

    // 全局跟踪
    //cv::Mat err, status;
    std::vector<unsigned char>  err, status;
    //cv::calcOpticalFlowPyrLK(gray_prev_, gray_curr_, from, to, status, err);
    printf("calc begin, time:%d\n",GetTickCount());
    calcLKOpticalFlow(gray_prev_, gray_curr_, from, to, status);
    printf("calc end, time:%d\n",GetTickCount());

    //for (int r = 0; r < status.rows; r++) {
    //    // 标记找错的
    //    if (status.at<uchar>(r, 0) != 1) {
    //        from[r].x = -10000;
    //    }
    //}

    for (int r = 0; r < status.size(); r++) {
        // 标记找错的
        if (status[r] != 1) {
            from[r].x = -10000;
        }
    }

    // 恢复到每个 targets 中
    off = 0;
    cv::Rect roi(0, 0, gray_curr_.cols, gray_curr_.rows);
    size_t n = 0;
    for (std::vector<Target>::iterator it = targets_.begin(); it != targets_.end(); n++) {
        std::vector<cv::Point2f> pts;
        for (; off < segs[n]; off++) {
            pts.push_back(to[off]);
        }

        if (!it->update(pts, stamp_, roi)) {
            log("WRN: %u: target 跟踪失败，放弃: %s\n", cnt_, it->descr().c_str());
            it = targets_.erase(it);
        }
        else {
            if (it->is_stopped()) {
                stopped.push_back(*it);
                it = targets_.erase(it);
            }
            else {
                if (debug_) {
                    cv::Scalar s(0, 128, 0);
                    it->debug_draw_paths(debug_img_, s, 30);    // 暗绿色 ..
                }

                // 检查是否过大 ???
                if (check_larger_ && it->pos().area() > large_threshold_) {
                    log("WRN: %u: target TOOOOOOOOO large %s, area=%d, threshold=%d\n",
                        cnt_, it->descr().c_str(), it->pos().area(), large_threshold_);
                    it = targets_.erase(it);
                }
                else {
                    ++it;
                }
            }
        }
    }
    printf("line:%d, time:%d\n",__LINE__, GetTickCount());

    return stopped;
}

void DetectWithOf::debug_save_motion(const Target &target, const cv::Rect &pos, Dir &dir) const
{
    char fname[128];
    cv::Mat rgb = origin_.clone();
    cv::Scalar s(0, 128, 0);
    target.debug_draw_paths(rgb, s);
    snprintf(fname, sizeof(fname), "./detlog/%05d-%d_%d-%s.jpg", cnt_, pos.x, pos.y, DirDesc[dir]);
    cv::imwrite(fname, rgb(pos));
}
