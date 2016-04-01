#pragma once
#ifndef _detect_t_h_
#define _detect_t_h_

//#include<opencv2/opencv.hpp>
//#include "opencv2/core/core_c.h"
//#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/ml/ml.hpp"
//#include "opencv2/highgui/highgui_c.h"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include<time.h>
#include "KVConfig.h"
#include <cstdio>
#include <sys/time.h>
//#include "ct.h"
using namespace cv;
using namespace std;

void print_time(char *str);

//计时结构体;
struct Static_s
{
	bool flag;
	Rect r;
	double pre_time;
	double cur_time;
	double continued_time;
	Static_s();

	//Static_s()
	//{
	//	flag = false;
	//}
};

//填充背景算法的结构体;
struct Fill_Bg
{
	//bool ud_static_begain;//是否刚更新完静止目标;
	bool body_move;

	double continued_time;//判断重合的时间（10）;
	double continued_time_long;//判断静止的第二种方法的更新时间;
	//double norect_update_time;//无目标多长时间就更新整个背景;

	bool isfillok;//蓝框区域更新的标志;
	bool isfillok_end;//蓝框区域是否更新正确的标志;
	int filltwice_num;//蓝框外连续单目标次数;

	int nframe;//贯穿主线;
	int num;//获取第10帧作为初始背景图;

	double mog2_interval1;//多目标融合间距（完全更新完毕之后）;
	double mog2_interval2;//大概一人的小的融合间距（在更新完毕之前）;
	double second_interval;//距离蓝框外多少时进行更新;

	double body_width;//人在图像中大概宽度(像素为单位);
	//double w_max;//bg2中第一个矩形需要扩展的距离;
	//double h_max;

	//std::vector<Rect> fill_nowrect;//当前刚更新的矩形框（？？？？）;
	std::vector<Rect> fist_fillrect;//扩展后蓝框;
	//std::vector<Rect> fist_fillrect_noclear;
	//std::vector<Rect> fist_fillrect_original;//第一个扩展后的矩形(未与整幅图像进行与的);
	//std::vector<Rect> fist_fillrect_t;//第一个扩展后的矩形;
	std::vector<Rect> rect_old;//背景减除获得的矩形;
	Mat bg;//背景图;
	//Static_s no_rect;
	Static_s mog2_s;//mog2算法的时间判断;

};

//struct CT_Face
//{
//	CvMemStorage* storage;
//    CvHaarClassifierCascade* cascade;
//	const char* cascade_name;
//	std::vector<Rect> face_rect;
//	std::vector<Rect> ct_rect;
//	bool is_face_ok;//是否检测到人脸;
//	bool is_ct_ok;//是否初始化ct框;
//};


//背景分区结构;
struct Region
{
	double cur_tbg;
	double pre_tbg;
	double continuetime_bg;
	bool flage_bg;//开始计时的标志;
	bool has_frame_rect;//是否有帧差框;
	bool has_old_rect;
	Rect region;//所在的分区;
	int num;
	double cur_static;
	double pre_static;
	double continuetime_static;
	bool flag_static;//开始计时的标志;
};

//利用帧差法更新背景图;
struct Update_Bg
{
    std::vector<Region> region;
	double time;//单目标持续无目标时间;
	double multiple_target_time;//多目标更新时间;
	int region_num;//分区数目;//从左向右依次1,2,3....
	double region_interval;//分区间隔;
	double slow_learn_rate;//无目标区域背景缓慢更新;
	double fast_learn_rate;//假目标区域快速更新;
};

//帧差法;
struct Frame_struct
{
	int N;
	Mat buffer[5];
	int threshold_three;//三帧差分法阈值;
	int threshold_two;//双帧差分法阈值;
	std::vector<Rect> frame_rect;
	std::vector<Rect> masked_frame_rect;
	int interval;
	int minarea;
	int minrect;
	bool is_body_down;
	int bottom_inter;//距离标定矩形框下方像素距离;
};

struct Upbody_Update
{
	Mat upbody_bg;
	Update_Bg bg_upbody;//&&&&&&&&&&;
	int Y_value;
	std::vector<cv::Rect> upbody_rect;
	int min_area;
	int min_rect_area;
	bool is_upbody;
	int upbody_u_max;
	int upbody_v_max;
	float region_interval;
	int frame_num;
};


class TeacherDetecting
{
	BackgroundSubtractorMOG2 bg_model;//可更改一些内部参数;
	float mog_learn_rate;

	int video_width_;
	int video_height_;
	KVConfig *cfg_;

	//CompressiveTracker ct;
	//CT_Face ct_face_s;
	Frame_struct frame_s;
	Update_Bg ud_bg_s;
	Fill_Bg  fillbg_struct;


	double min_area;
	double min_rect_area;
	double luv_u_max;
	double luv_v_max;
	double luv_L;

	bool ismask_;
	Mat img_mask_;//原来是原始大小掩码图像，后改成标定区掩码图像;

public:
	//bool ismask_;
	//Mat img_mask_;//原来是原始大小掩码图像，后改成标定区掩码图像;
	Upbody_Update up_update;
	TeacherDetecting(KVConfig *cfg);
	~TeacherDetecting(void);
	bool one_frame_luv(Mat raw_img,Mat img, vector<Rect> &r,vector<Rect> &first_rect);
	void upbody_bgupdate( Mat upbody_img);
    void get_upbody(Mat img_upbody );
	void upbody_luv_method(const Mat &img );
	void upbody_updatebg_slow(Mat img,Rect r,double learn_rate);
	void mog_method(Mat img);
	void do_mask(Mat &img);
	cv::Rect masked_rect;
	cv::Rect upbody_masked_rect;//&&&&&&&&&&&&;

protected:
	//人脸和ct;
	/*std::vector<Rect> face_detect_and_draw( IplImage* image);
	std::vector<Rect> face_detecting(Mat image,Rect rect);
	void init_ct(Mat raw_image,Rect rect);
	void is_right_ct( Mat raw_img,Mat raw_gray,std::vector<Rect> rect_old);
	void TeacherDetecting::facedetect_ctinit(Mat raw_image, Rect rect_bg);*/
	void init_fillbg_struct(Fill_Bg &fillbg_struct);

	//帧差法;
	void creat_buffer(IplImage *image);
	void two_frame_method(Mat img,Mat &silh);
	void three_frame_method(Mat img,Mat &silh);
	void frame_difference_method (Mat raw_image,std::vector<Rect> &full_rect,std::vector<Rect> &masked_rect);
	void init_frame_struct(Frame_struct &frame_s);

	//动态更新背景;
	void frame_updatebg(Mat image);
	void reset(Update_Bg &bg);
	void reset_upbody(Update_Bg &bg);//&&&&&&&&&&&&;
	void updatebg_slow(Mat img,Rect r,double learn_rate);
    void reset_static_region( Region &region );
	void reset_region( Region &region );
	void frame_updatebg(Mat raw_img,Mat image);

	//背景减除法;
	void luv_method(const Mat &img);
	void fillbg_LUV( Mat img);
	void is_teacher_down(Mat raw_img,Mat img2);
	void updatebg(Mat img,Rect r);
	void is_need_fillbg_twice(Mat img);
	void update_bg_again(Mat img);
	void norect_update_bg( Mat img);
	void eliminate_longrect(Mat img );

	//融合;
	static int cmp_area(const Rect&a,const Rect&b);
	Rect sort_rect(Rect a,Rect b);
	vector<Rect> refineSegments2(Mat img, Mat& mask,Mat &dst,double interval,double min_area,double rect_area);
	void rect_fusion2(vector<Rect> &seq,double interval);
	std::vector < Rect > upbody_refineSegments2(Mat img, Mat & mask,
						       Mat & dst,
						       double interval,double marea,double mrect_area);

	//掩码;
	bool build_mask_internal(const char *key, Mat& img);
	Mat build_mask(const char *key,const char *key2 = 0);
	std::vector<cv::Point> load_roi(const char *pts);
	static int cmp_min_x(const Point & a, const Point & b);
	static int cmp_min_y(const Point & a, const Point & b);
	cv::Rect get_point_rect(std::vector<cv::Point> pt);
	cv::Rect get_rect(const char* pt1,const char* pt2);
	void init_mask( );
};

#endif
