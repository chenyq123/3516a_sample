#include "cluster.h"
#include "Target.h"

cluster_builder::cluster_builder(int dis)
	: threshold_(dis)
{
}

cluster_builder::~cluster_builder()
{
}

void cluster_builder::calc(const std::vector<cv::Point2f> &pts0, std::vector<std::vector<cv::Point2f> > &result)
{
	std::vector<cv::Point2f> pts = pts0;
	std::vector<Cluster> clusters;

	double threshold = threshold_;

	while (!pts.empty()) {
		once(pts, clusters, threshold);
		threshold /= 2;		// 增加创建新的 cluster 的机会 ....
	}

	merge_clusters(clusters);	// 合并 ..

	for (std::vector<Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
		result.push_back(it->pts);
	}
}

void cluster_builder::once(std::vector<cv::Point2f> &pts, std::vector<cluster_builder::Cluster> &clusters, double threshold)
{
	for (std::vector<cv::Point2f>::iterator it = pts.begin(); it != pts.end();) {
		double dis;
		std::vector<Cluster>::iterator it_cluster = find_nearest_cluster(*it, clusters, dis);
		if (it_cluster == clusters.end()) {
			Cluster c;
			c.id = next_cid_++;
			c.mean_pt = *it;
			c.pts.push_back(*it);
			clusters.push_back(c);

			it = pts.erase(it);
		}
		else {
			if (dis < threshold) {
				it_cluster->pts.push_back(*it);
				it_cluster->calc_mean();

				it = pts.erase(it);
			}
			else if (dis > 2 * threshold) {
				Cluster c;
				c.id = next_cid_++;
				c.mean_pt = *it;
				c.pts.push_back(*it);
				clusters.push_back(c);

				it = pts.erase(it);
			}
			else {
				++it;	// 保留，下次迭代 ...
			}
		}
	}
}

void cluster_builder::build_each_clusters(const std::vector<cluster_builder::Cluster> &clusters, std::vector<cluster_builder::ClusterPair> &cluster_pairs)
{
	for (size_t i = 1; i < clusters.size(); i++) {
		const Cluster &c0 = clusters[i - 1];
		for (size_t j = i; j < clusters.size(); j++) {
			const Cluster &c = clusters[j];
			struct ClusterPair cp;
			cp.c0 = c0, cp.c1 = c;
			cp.dis = distance(c0.mean_pt, c.mean_pt);

			cluster_pairs.push_back(cp);
		}
	}

	std::sort(cluster_pairs.begin(), cluster_pairs.end(), op_dis_small_cluster);
}

std::vector<cluster_builder::Cluster>::iterator cluster_builder::find_nearest_cluster(const cv::Point2f &pt,
	std::vector<cluster_builder::Cluster> &clusters, double &dis)
{
	dis = 10000000.0;
	std::vector<Cluster>::iterator it, it_nearest = clusters.end();
	for (it = clusters.begin(); it != clusters.end(); ++it) {
		double d = ::distance(pt, it->mean_pt);
		if (d < dis) {
			dis = d;
			it_nearest = it;
		}
	}

	return it_nearest;
}

void cluster_builder::merge_clusters(std::vector<cluster_builder::Cluster> &clusters)
{
	if (clusters.size() <= 1) {
		return;
	}

	std::vector<ClusterPair> cluster_pairs;
	build_each_clusters(clusters, cluster_pairs);

	for (std::vector<ClusterPair>::iterator it = cluster_pairs.begin(); it != cluster_pairs.end(); ++it) {
		if (it->dis < threshold_) {
			std::vector<Cluster>::iterator it0 = find_cluster(clusters, it->c0.id);
			std::vector<Cluster>::iterator it1 = find_cluster(clusters, it->c1.id);

			if (it0 != clusters.end() && it1 != clusters.end()) {
				Cluster c;
				c.id = next_cid_++;
				c.pts = it0->pts;
				c.pts.insert(c.pts.begin(), it1->pts.begin(), it1->pts.end());
				c.calc_mean();

				clusters.push_back(c);
				remove_cluster(clusters, it0->id);
				remove_cluster(clusters, it1->id);
			}
		}
		else {
			break;
		}
	}
}
