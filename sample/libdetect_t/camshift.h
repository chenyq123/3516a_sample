#pragma once

#include <opencv2/opencv1.hpp>
#include <cstdio>
#include <iostream>
//目前使用流程: ;
//1. 创建tracker_camshift对象后, 调用set_hist(const char *file_name)读取预存的直方图;
//2. 调用set_track_window()设置初始跟踪框, 可将hog确定的矩形框作为参数;
//3. 调用process(), is_erode设为true, 可确定人脸位置;
//4. 根据人脸框和头肩框的关系确定是否为人脸;
// 拉近镜头, 重复上面步骤;
//5. 把人脸框适当缩小,  调用set_hist(cv::Mat  &image_bgr, const cv::Rect &rect, int hsize, int ssize)重新设置直方图;
//6. 继续调用process(), is_erode如果设为true时可减小误检率, 但容易跟丢;
//7. 跟丢的时候process返回的Rect.area() < 1;

class tracker_camshift
{
public:
	tracker_camshift(const char* yaml_file);
	//用于显示背景状态,  一般用于测试;
	cv::Mat get_backproj(const cv::Mat &image_bgr);
	cv::Rect process(const cv::Mat &image, bool is_erode = true);
    void set_track_window(const cv::Rect &position);
	void set_hist(cv::Mat  &image_bgr, const cv::Rect &rect, int hsize = 60, int ssize = 128, int threshold_value = 50);
	void set_hist(const char *file_name);
	bool is_face(cv::Rect &face, cv::Rect upper_body);
	bool cam_shift_init_once(cv::Rect &face,cv::Mat image);
	~tracker_camshift();

private:
	cv::Rect track_window;
	cv::Mat hist;
	int vmin;
	int vmax;
	int vthreshold;
	int hist_size[2];
	float hranges[2];
	float sranges[2];
};


