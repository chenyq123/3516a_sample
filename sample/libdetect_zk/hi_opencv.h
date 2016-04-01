#ifndef HI_OPENCV_H
#define HI_OPENCV_H

#include <opencv2/opencv.hpp>
#include "hi_comm_ive.h"
#include "hi_ive.h"
#include "mpi_ive.h"
#include "mpi_sys.h"
#include "hi_comm_sys.h"
using namespace cv;

void hi_dilate(Mat src, Mat &dst, Mat element, Point anchor = Point(-1, -1), int iterations = 1);
void hi_erode(Mat src, Mat &dst, Mat element, Point anchor = Point(-1, -1), int iterations = 1);
void hi_absdiff(Mat src1, Mat src2, Mat &dst);
//unsigned long GetTickCount1()
//{
//    struct timespec ts;
//    clock_gettime(CLOCK_MONOTONIC, &ts);
//    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
//}
#endif
