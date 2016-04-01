#ifndef DETECT_MAIN_H__
#define DETECT_MAIN_H__
#include <stdio.h>
#include <stdlib.h>
//#include "../libimagesource/image_source.h"
#include <opencv2/opencv.hpp>
//#include <opencv2/ocl/ocl.hpp>
//#include "DetectWithOpticalFlow.h"
//#include "DetectWithOF.h"
//#include "DetectWithOF2.h"
//#include "DetectWithOF3.h"
#include "DetectWithOF4.h"
//#include "DetectWithCT.h"
//#include "DetectWithOF5.h"
#include "detect_main.h"
#include <deque>
#include <time.h>
struct Ctx
{
	KVConfig *cfg_;
	Detect *detector_;
	std::vector<cv::Rect> students_rects;
	cv::Mat mask_;		// ÑÚÂë ...
	bool masked_;
	bool flipped_;

	Ctx(const char *cfg_name)
	{
		cfg_ = new KVConfig(cfg_name);
		int a_mode = atoi(cfg_->get_value("a_mode", "1"));
		if (a_mode == 1) {
			// ³íÃÜÀÛ¼Æ¹âÁ÷
//			detector_ = new DetectWithOpticalFlow(cfg_);
            printf("mode 1\n");
		}
		else if (a_mode == 0) {
			// ³íÃÜ¹âÁ÷
			//detector_ = new DetectWithOF3(cfg_);
            printf("mode 3\n");
		}
		else if (a_mode == -1) {
			// Ï¡Êè¹âÁ÷
			//detector_ = new DetectWithOF2(cfg_);
            printf("mode 2\n");
		}
		else if (a_mode == 2) {
			// DetectWithCT
			//detector_ = new DetectWithCT(cfg_);
            printf("mode CT\n");
		}
		else if (a_mode == 4) {
			detector_ = new DetectWithOF4(cfg_);
            printf("mode 4\n");
		}
		else if (a_mode == 5) {
			//detector_ = new DetectWithOF5(cfg_);
            printf("mode 5\n");
		}
		else {
			//MessageBoxA(0, "²»Ö§³ÖµÄ a_mode", "´íÎó", MB_OK);
            printf("wrong mode\n");
			exit(-1);
		}

		masked_ = build_mask(mask_);
		flipped_ = false;
	}

	~Ctx()
	{
		delete detector_;
	}

	const std::vector<cv::Rect> &detect(cv::Mat &frame, int &flipped_idx);

private:
	bool build_mask(cv::Mat &mask)
	{
		bool masked = false;

		const char *pts = cfg_->get_value("calibration_data", 0);
		std::vector<cv::Point> points;

		if (pts) {
			char *data = strdup(pts);
			char *p = strtok(data, ";");
			while (p) {
				// Ã¿¸öPoint Ê¹"x,y" ¸ñÊ½
				int x, y;
				if (sscanf(p, "%d,%d", &x, &y) == 2) {
					cv::Point pt(x, y);
					points.push_back(pt);
				}

				p = strtok(0, ";");
			}
			free(data);
		}

		if (points.size() > 3) {
			int n = points.size();
			cv::vector<cv::Point> pts;
			for (int i = 0; i < n; i++) {
				pts.push_back(points[i]);
			}

			mask = cv::Mat::zeros(cv::Size(atoi(cfg_->get_value("video_width", "480")), atoi(cfg_->get_value("video_height", "270"))), CV_8UC3);

			std::vector<std::vector<cv::Point> > ptss;
			ptss.push_back(pts);
			cv::fillPoly(mask, ptss, cv::Scalar(255, 255, 255));

			masked = true;
		}

		return masked;
	}

};

typedef struct zifImage
{
	//PixelFormat fmt_type;
	int width;
	int height;
	unsigned char *data[4];
	int stride[4];

	double stamp;

	void *internal_ptr;
} zifImage;

void det_stu_close(void *ins);
//const char *det_detect(void *det, zifImage *img);
const char *det_detect(void *det, cv::Mat &frame);
void *det_stu_open(const char *cfg_name);
void det_set_flipped_mod(void *det, int enabled);
void det_set_param(void *det, int thres_dis, int thres_area, double factor_0, double factor_05, double notused);
void det_source_stats(void *det, int total, int lost, int pending, int cached);

#endif
