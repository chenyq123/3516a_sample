#pragma once
#include "detect.h"

/** 实现青大谭工模式

		0. 需要“活动区域”的概念，每个“活动区域”保留N帧活动历史，活动区域通过帧差法，聚合得到；
		1. 每个活动区域保留N帧历史
		2. 计算活动区域中的稠密光流，找出“向上”的活动;
		3. 在“向上”的活动区域中找“人脸”的活动 ..
 */

class DetectWithOF5: public Detect
{
	/** 活动区域 */
	struct Motion
	{
		DetectWithOF5 *parent;
		size_t M;	// 最多保存M个历史轮廓 ...
		int id;
		double stamp;	// 最后更新时间戳.
		size_t frame_idx_;		// 第一个历史对应的帧序号，便于从 hist_ 中查找对应的历史图像 ..
		std::deque<std::vector<cv::Point> > history;	// 历史轮廓
		std::vector<cv::Point> last_contour;	// 最后有效的轮廓
		cv::Rect brc;	// 包括历史的外接矩形，
		bool tracking_inited_;	// 是否已经开始跟踪 ..
		bool discard_;	// 仅仅 merge_overlapped_motions() 使用，标记为需要删除 ...

		cv::Scalar color;	// debug

		std::deque<std::vector<cv::Point2f> > tracking_pts;	// 跟踪的特征点，tracking_pts.size() == history.size()
		std::vector<cv::Rect> faces;	// FIXME: 人脸，如果多个怎么办？？

		// 保存历史稠密光流
		std::deque<cv::Rect> dense_of_poss;	// 计算光流的位置；
		std::deque<cv::Mat> dense_of_xs;	// 水平，竖直方向的光流 ...
		std::deque<cv::Mat> dense_of_ys;	// 
		cv::Mat dense_sum_dis;
		cv::Mat dense_sum_dir;	// 累计后的光流极坐标模式 ...

		std::vector<cv::Point> get_contours_center(); // 返回历史轮廓的中心，一定程度上，能反应motion的移动 ...
		std::vector<cv::Point> get_brcs_center();		// 返回历史轮廓外界矩形的中心，照理说比 get_contours_center() 更稳定？
		void update_bounding_rc();
		void init_tracking(cv::CascadeClassifier *cc); // 初始化跟踪 ...
		void track();		// 跟踪特征点 ...
		bool has_same_last_hist(const Motion &m);	// 与 m 比较，是否最后两张历史一样 ??
		bool last_hist(std::vector<cv::Point> &cont, int rid); // 倒数 rid 个有效历史轮廓，rid=0, 1, 2 ...
		void merge_dense_ofs(cv::Mat &xx, cv::Mat &yy);	// 合并历史光流，大小为 brc
		
	private:
		void hlp_make_of_brc(size_t idx, cv::Mat &x, cv::Mat &y);	// idx 为 dense_of_xxx 的序号，返回 brc 大小的，额外部分为 0
	};

	friend struct Motion;
	typedef std::vector<Motion> MOTIONS;
	MOTIONS motions_;	// 活动区域 ..

	struct History
	{
		std::deque<cv::Mat> frames;	// 历史图像 
		std::deque<cv::Mat> diff;	// 历史帧差 
		size_t frame_idx_;			// frames.front() 的帧序号 ...
		size_t N;					// 希望保留的历史 ..
		DetectWithOF5 *parent;

		void push(const cv::Mat &img);
		bool exist(size_t idx);
		cv::Mat get(size_t idx);  // assert(exist(idx))
		cv::Mat rget(size_t rid); // 从后往前找 ... rid=0 最后一帧，1 倒数第二帧 ...
								  // assert(rid < frames.size())

		cv::Mat get_diff(size_t idx);
		cv::Mat rget_diff(size_t rid);
	};
	friend struct History;
	History hist_;

	cv::Mat ker_erode_, ker_dilate_, ker_open_;
	cv::Mat origin_;
	double curr_;	// 当前时间 ..
	size_t frame_idx_;	// 当前帧序号 ..
	float threshold_diff_;	// 帧差阈值，默认 30

	double *factors_y_;	// 图像中，从上到下的系数，最下面（近）总为1.0，最上面（远）为参数配置值，如0.1，使用线性变换 ..

	int motion_M_;	// 每个活动最多保留历史数目，缺省 = hist_.N
	double motion_timeout_;	// 活动区域超时时间，默认 300ms

	cv::CascadeClassifier *cc_;

	int target_x_, target_y_; // 目标大体宽高 ...

public:
	DetectWithOF5(KVConfig *kv);
	~DetectWithOF5(void);

private:
	virtual void detect(cv::Mat &origin, std::vector<cv::Rect> &targets, int &flipped_index);

	/** 保存历史，历史包括原始图像，和帧差的灰度结果 */
	void save_hist(const cv::Mat &origin);

	/** 从最后的帧差找可能的目标 */
	void find_contours(std::vector<std::vector<cv::Point> > &regions);

	// 比较motion大小
	static bool op_larger_motion(const DetectWithOF5::Motion &m0, const DetectWithOF5::Motion &m1);

	/** 合并 find_motions() 得到的 motions */
	void merge_motions(const std::vector<std::vector<cv::Point> > &regions);

	/** 合并重叠的 motions
	 */
	void merge_overlapped_motions();

	/** 跟踪 motions */
	void tracking_motions();
	void draw_tracking();
	void draw_motion_tracking(Motion &m);

	/** 合并接近的轮廓 */
	void merge_contours(std::vector<std::vector<cv::Point> > &contours);

	/** draw motions */
	void draw_motions();

	/** 删除超时的 motions */
	void remove_timeout_motions();

	/// 面积从大到小排序
	static inline bool op_sort_motion_by_area(const Motion &m1, const Motion &m2)
	{
		return m1.brc.area() > m2.brc.area();
	}

	/** 根据 y 的位置，返回面积系数 */
	double factor_y(int y) const;

	/// 根据 factor_y_ 估算目标大小
	cv::Size est_target_size(int y);

	/** 从 m 中搜索邻近的聚类，使用估计的目标大小模板搜索 ...
			diff 要求是 0 或 1 的二值图
	 */
	std::vector<cv::Rect> find_diff_clusters(const cv::Mat &bin_diff);

	/** 为了调试方便，显示不同位置，目标的大小 */
	void show_targets_size();

	/** 计算最后历史之间的稠密光流，返回 ..*/
	bool calc_dense_of(const cv::Rect &roi, cv::Mat &x, cv::Mat &y);

	/** 统计稠密光流的累计和 */
	void sum_motions_dense_of();
	void draw_motions_dense_of();

	/** 根据 dis, ang 对应的 HSV，转换为 rgb 图 32FC3，方便显示 */
	void rgb_from_dis_ang(const cv::Mat &dis, const cv::Mat &ang, cv::Mat &rgb);

	/** 输入 32FC1 的 dir，四个方向选择最中间的值，就是说：右：0，上 270，下 90，左 180 */
	void normalize_dir(const cv::Mat &dir, cv::Mat &n);

	/** 输入 32FC1 的 dis，使用阈值 ... */
	void normalize_dis(const cv::Mat &dis, cv::Mat &n);
};
