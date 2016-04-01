#include "objdet.h"
#include <iostream>
#include <time.h>
#include <cstdio>
#include <opencv2/opencv.hpp>
int main()
{
    KVConfig *cfg_;
    objdet *ob_;
    std::vector<cv::Rect> faces;
    cfg_ = new KVConfig("student_detect_trace.config");
    ob_ = new objdet(cfg_);
    cv::Mat roi = cv::imread("face.jpg",1);
    while(1)
    {
        printf("\n==============================\n");
        printf("%d\n",time(NULL));
        ob_->has_faces(roi, faces);
        printf("size:%d\n",faces.size());
        //printf("%d\n",time(NULL));
        printf("\n==============================\n");
    }
    return 0;
}
