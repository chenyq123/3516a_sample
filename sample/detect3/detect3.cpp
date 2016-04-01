#include <iostream>
#include "detect_t.h"
#include "KVConfig.h"
#include "opencv2/opencv.hpp"

int main(void)
{
    bool isrect;
    KVConfig *cfg_ = new KVConfig("config");
    TeacherDetecting *detect_ = new TeacherDetecting(cfg_);
    Mat img = imread("img.jpg", 0);
    Mat masked_img_temp = Mat(Img, detect_->masked_rect);
    Mat masked_img;
    masked_img_temp.copyTo(masked_img);
    detect_->do_mask(masked_img);
    isrect = detect_->one_frame_luv(img, masked_img, r, first_r);

    for(int i = 0; i < r.size(); i++)
    {
        cv::Rect box = detect_->masked_rect;
        r[i].x = r[i].x + box.x;
        r[i].y = r[i].y + box.y;
        r[i] &= cv::Rect(0, 0, Img, cols, Img.rows);
    }

    cv::Rect upbody;
    bool is_up_body = false;
    if(atoi(cfg_->get_value("t_upbody_detect", "0")) > 0)
    {
        Mat masked_upbody = Mat(Img, detect_->upbody_masked_rect);
        detect_->get_upbody(masked_upbody);
        if(detect_->up_update.upbody_rect.size() > 0)
        {
            cv::Rect upbody_t = detect_->up_update.upbody_rect[0];
            if(!(upbody_t.x + upbody_t.width / 2 > detect_->upbody_masked_rect.width - 15 || upbody_t.x + upbody_t.width / 2 < 15))
            {
                is_up_body = true;
                cv::Rect box = detect_->upbody_masked_rect;
                upbody = upbody_t;
                upbody.x = upbody.x + box.x;
                upbody.y = upbody.y + box.y;
                upbody &= cv::Rect(0, 0, Img.cols, Img.rows);
            }
        }
    }
    return 0;
}
