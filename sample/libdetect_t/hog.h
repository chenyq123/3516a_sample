#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "KVConfig.h"
#include <sys/timeb.h>
#include "camshift.h"
//¶¨Ê±Í·¼ç¼ì²â;
struct Detecting
{
    bool start;
    cv::Rect up_body_pre;
    timeb pre_time;
    timeb cur_time;
    int short_inter_time;
    int long_inter_time;
    int inter_time;
    bool timeing_start;
    bool is_strict;
};

class zk_hog_detector
{
public:
    tracker_camshift *cf_;
    Detecting det_;
    bool is_debug;
    int UP_BODY_WIDTH ;
    int UP_BODY_HEIGHT;
    zk_hog_detector(const char *alt_file, KVConfig *cfg, cv::Size winsize);
    std::vector<cv::Rect> zk_hog_upperbody(const cv::Mat &gray, int group_threshold = 2, double scale = 1.05) ;
    //std::vector<cv::Rect> zk_hog_biggest_upperbody(const cv::Mat &gray, double scale = 1.05);
    void expend_target_rect(cv::Rect &target,cv::Size image);
    bool zk_hog_detect( cv::Mat img, const cv::Rect rc,cv::Rect &face);
    bool up_bd_detect(cv::Mat Img,std::vector <cv::Rect> r,cv::Rect &up_body);
    float set_zoom_ratio(int source_width);
    ~zk_hog_detector();
private:
    cv::HOGDescriptor hog;

};
