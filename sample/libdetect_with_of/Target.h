#pragma once

#include <opencv2/opencv.hpp>
#include "utils.h"
#include "KVConfig.h"
//#include "hi_ive.h"
//#include "hi_comm_ive.h"
//#include "mpi_ive.h"
//#include "hi_comm_sys.h"
//#include "mpi_sys.h"
//#include <vector>

/** 对应一个活动目标，创建时，找到特征点，然后进行持续跟踪，如果发现“特征点”较长时间
    不动了，则认为目标活动结束，开始评估该目标的活动 ...
 */
class Target
{
    cv::Rect outer_;    // 全局外边框 ...
    cv::Rect brc_;  // 轨迹外框 ...
    cv::Rect first_rc_, last_rc_;   // 起始位置，最后一层的跟踪点的外接 ...
    double stamp_;
    int id_;
    KVConfig *cfg_;

    typedef std::vector<cv::Point2f> PTS;   // 相对一层的特征点 ...
    std::vector<PTS> layers_;               // 对应多层的特征点 ...


public:
    Target();
    ~Target();

public:
    typedef PTS PATH;

    bool init(KVConfig *cfg, int id, const cv::Rect &roi, const cv::Mat &curr, double stamp);
    bool track(const cv::Mat &prev, const cv::Mat &curr, double stamp);

    /// 检查轨迹的最后几帧，如果没有变化，则认为活动已经结束了 ...
    bool is_stopped() const;

    /// 返回 rc 是否属于 target ....
    bool is_crossed(const cv::Rect &rc) const;

    /// 返回pos
    inline cv::Rect pos() const
    {
        return brc_;
    }

    /// 返回层数 ...
    inline int layers() const
    {
        return (int)layers_.size();
    }

    /// 返回path数目
    inline int get_path_cnt() const
    {
        if (layers_.empty())
            return 0;
        else
            return (int)layers_[0].size();
    }

    /// 返回根据距离长短进行排序的所有 path
    std::vector<PATH> get_sorted_paths() const
    {
        return get_sorted_paths(0, (int)layers_.size());
    }

    /// 返回描述
    std::string descr() const
    {
        std::stringstream ss;
        ss << "ID:" << id_ << ", pos:" << pos2str(brc_) << ", layers:" << layers_.size() << ", paths:" << get_path_cnt();
        return ss.str();
    }

    ///
    void debug_draw_paths(cv::Mat &rgb, cv::Scalar &color) const;

private:

    /// 局部转到全局坐标 ..
    inline void l2g(PTS &pts, const cv::Point &tl) const
    {
        for (PTS::iterator it = pts.begin(); it != pts.end(); ++it) {
            it->x += tl.x;
            it->y += tl.y;
        }
    }

    /// 全局转换到局部 ...
    inline void g2l(PTS &pts, const cv::Point &tl) const
    {
        for (PTS::iterator it = pts.begin(); it != pts.end(); ++it) {
            it->x -= tl.x;
            it->y -= tl.y;
        }
    }

    /// 删除 idx 对应的路径 ...
    void remove_path(size_t idx)
    {
        assert(layers_.size() > 0);
        assert(idx < layers_[0].size());

        for (std::vector<PTS>::iterator it = layers_.begin(); it != layers_.end(); ++it) {
            it->erase(it->begin() + idx);
        }
    }

    /// 返回一条路径
    PATH get_path(int idx) const
    {
        return get_path(idx, 0, (int)layers_.size());
    }

    /// 返回一条路径的一部分　...
    PATH get_path(int idx, int from_layer, int to_layer) const
    {
        //printf("layers_.size=%d,to_layer=%d,from_layer:%d,layers[0]_.size:%d\n",layers_.size(), to_layer, from_layer, layers_[0].size());
        //assert(layers_.size() > to_layer && from_layer < to_layer && idx < layers_[0].size());

        PTS path;
        for (int i = from_layer; i < to_layer; i++) {
            path.push_back(layers_[i][idx]);
        }

        return path;
    }

    /// 根据path距离进行从大到小排序
    static bool op_bigger_dis(const PATH &p1, const PATH &p2)
    {
        return distance(p1) > distance(p2);
    }

    /// 返回两层之间的路径 ...
    std::vector<PATH> get_sorted_paths(int from, int to) const
    {
        assert(from < to);
        assert(layers_.size() >= to);
        assert(from >= 0);

        std::vector<PATH> paths;
        int cnt = get_path_cnt();
        for (int i = 0; i < cnt; i++) {
            paths.push_back(get_path(i, from, to));
        }

        std::sort(paths.begin(), paths.end(), op_bigger_dis);
        return paths;
    }

    /// 计算路径的累计长度
    static inline double distance(const PATH &path)
    {
        if (path.size() < 2)
            return 0.0;

        double dis = 0;
        for (size_t i = 1; i < path.size(); i++) {
            dis += ::distance(path[i - 1], path[i]);
        }

        return dis;
    }

    /// 检查长时间的活动 ...
    void check_paths(double stamp);

    void debug_draw_path(cv::Mat &rgb, const PATH &path, cv::Scalar &color) const;
};
