#pragma once

#include <opencv2/opencv1.hpp>

/// 从一大堆点中，找出N个聚类 ...
class cluster_builder
{
    int threshold_;
    int next_cid_;

public:
    explicit cluster_builder(int dis_threshold);
    ~cluster_builder();

    void calc(const std::vector<cv::Point2f> &pts, std::vector<std::vector<cv::Point2f> > &clusters);

private:
    struct Cluster
    {
        cv::Point2f mean_pt;
        std::vector<cv::Point2f> pts;
        int id; // 唯一 ... 方便合并 ...

        inline void calc_mean()
        {
            float x = 0, y = 0;
            for (std::vector<cv::Point2f>::const_iterator it = pts.begin(); it != pts.end(); ++it) {
                x += it->x, y += it->y;
            }

            mean_pt.x = x / pts.size(), mean_pt.y = y / pts.size();
        }
    };

    inline void once(std::vector<cv::Point2f> &pts, std::vector<Cluster> &clusters, double threshold);
    inline std::vector<Cluster>::iterator find_nearest_cluster(const cv::Point2f &pt, std::vector<Cluster> &clusters, double &dis);
    inline void merge_clusters(std::vector<Cluster> &clusters);

    struct ClusterPair
    {
        Cluster c0, c1;
        double dis;
    };

    inline static bool op_dis_small_cluster(const ClusterPair &cp0, const ClusterPair &cp1)
    {
        return cp0.dis < cp1.dis;
    }

    inline void build_each_clusters(const std::vector<Cluster> &clusters, std::vector<ClusterPair> &cluster_pairs);
    inline std::vector<Cluster>::iterator find_cluster(std::vector<Cluster> &clusters, int cid) const
    {
        for (std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
            if (it->id == cid) {
                return it;
            }
        }
        return clusters.end();
    }
    inline void remove_cluster(std::vector<Cluster> &clusters, int cid) const
    {
        for (std::vector<Cluster>::iterator it = clusters.begin(); it != clusters.end(); ++it) {
            if (it->id == cid) {
                clusters.erase(it);
                return;
            }
        }
    }
};
