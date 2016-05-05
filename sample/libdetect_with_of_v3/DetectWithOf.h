#pragma once

#define _USE_MATH_DEFINES

#include "Detect.h"
#include "DiffMotion.h"
#include "Target.h"

/** 使用稀疏光流记录连续活动，当活动停止后，检查活动的轨迹，返回有效的活动 ...

    @date: pc 上适合一帧跟踪多个小区域（多次局部的光流），但 arm 上可能适合做一次完整的跟踪 ...
 */
class DetectWithOf : public Detect
{
    cv::Mat debug_img_;     // 显示调试信息 ..

    bool track_once_;   // 是否跟踪一次 ...

    DiffMotion dm_;
    std::vector<Target> targets_;   // 活动目标 ...

    bool support_long_time_tracking_;   // 是否支持长时间跟踪，就是说，检测到站立后，继续跟踪，直到取消站立状态 ...
    std::vector<Target> standups_;  // 站立的目标 ...

    // 学生区最后排，最前排的位置，在此之间，使用线性拟合 ...
    float far_y_, near_y_;  // 根据 y 轴 ...

    float far_width_, near_width_;  // 帧差矩形的中心y的宽度阈值，如果小于该值，则认为帧差矩形太小了 ...
    double a_width_, b_width_;      // 宽度的直线方程系数 ...

    float far_dis_[4], near_dis_[4];    // 四个方向的距离阈值 ...
    double a_dis_[4], b_dis_[4];    //

    int up_angle_;      //

    int next_tid_;      // Target ID

    bool check_larger_; // 是否检查超大活动区域 ...
    int large_threshold_;   //

    // 仅仅计算长度最大的 N 条路径 ...
    int n_paths_for_stats_; // 默认 8，不能超过 10

    double target_min_dis_5frames_; // 一个目标的有效特征点，在前5帧内，必须至少移动的距离，如果前5帧几乎没有活动，可以认为这些特征点找错了 ...
    int target_min_pts_;            // 一个有效目标，至少拥有的特征点数目 ...
    int max_feature_pts_;           // 初始找特征点的数目 ...
    double feature_quality_level_;  // goodFeaturesToTrack() 的 quality level 参数 ..

public:
    explicit DetectWithOf(KVConfig *cfg);
    ~DetectWithOf();

    virtual void reset();

private:
    virtual void detect(RCS &motions, std::vector<Dir> &dirs);
    virtual void detect2(RCS &standups);

    virtual bool support_detect2() const
    {
        return support_long_time_tracking_;
    }

    inline std::vector<Target>::iterator find_matched_target_from(const cv::Rect &rc, std::vector<Target> &targets)
    {
        for (std::vector<Target>::iterator it = targets.begin(); it != targets.end(); ++it) {
            if (it->is_crossed(rc)) {
                return it;
            }
        }
        return targets.end();
    }

    /// 找到与 rc 相交的 target ...
    inline std::vector<Target>::iterator find_matched_target(const cv::Rect &rc)
    {
        return find_matched_target_from(rc, targets_);
    }

    /// 从 standups 中查找 ..
    inline std::vector<Target>::iterator find_matched_target_from_ltt(const cv::Rect &rc)
    {
        return find_matched_target_from(rc, standups_);
    }

    /// 根据远近，判断活动区域的宽度 ...
    bool too_small(const cv::Rect &rc) const;

    enum {
        E_OK,
        E_NotEnoughLayers,
        E_NotEnoughPaths,
        E_NotEnoughDistance,
    };

    /// 评估target的活动，如果有效，则返回 OK
    int estimate(const Target &target, cv::Rect &pos, Dir &dir) const;

    /// 根据两点求直线的 a,b 系数
    inline void polyfit_line(const cv::Point2f &p1, const cv::Point2f &p2, double &a, double &b) const
    {
        cv::Point2f p = p2;
        if (p.x == p1.x) {   // NOTE: 防止竖直直线 ...
            p.x += (float)0.01;
        }

        a = (p1.y - p.y) / (p1.x - p.x);
        b = p.y - a * p.x;
    }

    /// 根据两点，求角度，从 x 轴顺时针旋转一周，[0..360)
    inline double calc_angle(const cv::Point2f &p1, const cv::Point2f &p2) const
    {
        double a = atan2(-(p2.y - p1.y), p2.x - p1.x) * 180.0 / M_PI;
        if (a > 0) {  // 第一二象限 ..
            a = 360.0 - a;
        }
        else {  // 三四象限 ..
            a = -a;
        }

        return a;
    }

    /// 跟踪所有 targets_，返回停止活动的 targets
    std::vector<Target> track_targets();

    /// 处理新的活动区域 ...
    void process_diff_motions();

    /// 保存有效活动 ...
    void debug_save_motion(const Target &target, const cv::Rect &pos, Dir &dir) const;

    /// 单次跟踪 ...
    std::vector<Target> track_targets_once();
};
