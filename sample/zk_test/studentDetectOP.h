#include <opencv2/opencv.hpp>
#include <vector>
struct upStudent
{
	double stamp;
	cv::Rect position;
};
class studentDetectOP;

class studentReference 
{
	friend class studentDetectOP;
public:
	studentReference(cv::Rect position_c, double speed_c, double area_c, cv::Size size_c);
private:
	cv::Size size_;
	cv::Rect position;
	cv::Point base_point;
	double speed;
	double area;
	static cv::Point get_base_point(cv::Rect rc, int mid_x);
};
class studentDetectOP
{
public:
	studentDetectOP(cv::Size sz, double duration);
	~studentDetectOP();
	void process(const cv::Mat& img_color, double stamp);
	void clear(void);

	std::vector<upStudent> up_students;
	//调用process()前必须要设置;
	void add_student_ref(const studentReference & student_ref);
	void set_back_down_up(int start, double start_ratio, double end_ratio);
	void set_debug(bool on_off);
	void set_mask(const cv::Mat &mask_c);

	void motionToColor(cv::Mat flow, cv::Mat &color);
	void makecolorwheel(std::vector<cv::Scalar> &colorwheel);   
private:
	cv::Size img_size;
	double up_student_duration;
	double min_student_area;
	std::vector<studentReference> student_refs;

	int back_start;
	double start_back_down_up;
	double end_back_down_up;

	bool mask_on;
	cv::Mat mask;
	double down_up;
	double fb_speed_ratio;
	double up_speed_horizon_vertical ;

	bool is_debug;

	cv::Mat pre_gray;
	cv::Mat cur_gray;
	cv::Mat cur_color;
	double cur_stamp;

	cv::BackgroundSubtractorMOG2 *bg_model;
	std::vector<cv::Rect> foregrounds;

	std::vector<cv::Rect> up_student_candidates;
	
	cv::Ptr<cv::DenseOpticalFlow> tvl1;
private:
	void bg_filter();
	void student_down();
	void offb_filter();
	void oftvl1_filter();
	studentReference get_student_ref(cv::Rect position);
	
};