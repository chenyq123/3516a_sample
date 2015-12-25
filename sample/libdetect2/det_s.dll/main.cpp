#include <stdio.h>
#include <stdlib.h>
#include "../libimagesource/image_source.h"
#include <opencv2/opencv.hpp>
#include <opencv2/ocl/ocl.hpp>
#include "DetectWithOpticalFlow.h"
#include "DetectWithOF.h"
#include "DetectWithOF2.h"
#include "DetectWithOF3.h"
#include "DetectWithOF4.h"
#include "DetectWithCT.h"
#include "DetectWithOF5.h"
#include <deque>
#include <Windows.h>

struct Ctx
{
	KVConfig *cfg_;
	Detect *detector_;
	std::vector<cv::Rect> students_rects;
	cv::Mat mask_;		// 掩码 ...
	bool masked_;
	bool flipped_;

	Ctx(const char *cfg_name)
	{
		cfg_ = new KVConfig(cfg_name);
		int a_mode = atoi(cfg_->get_value("a_mode", "1"));
		if (a_mode == 1) {
			// 稠密累计光流
			detector_ = new DetectWithOpticalFlow(cfg_);
		}
		else if (a_mode == 0) {
			// 稠密光流
			detector_ = new DetectWithOF3(cfg_);
		}
		else if (a_mode == -1) {
			// 稀疏光流
			detector_ = new DetectWithOF2(cfg_);
		}
		else if (a_mode == 2) {
			// DetectWithCT
			detector_ = new DetectWithCT(cfg_);
		}
		else if (a_mode == 4) {
			detector_ = new DetectWithOF4(cfg_);
		}
		else if (a_mode == 5) {
			detector_ = new DetectWithOF5(cfg_);
		}
		else {
			MessageBoxA(0, "不支持的 a_mode", "错误", MB_OK);
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
				// 每个Point 使"x,y" 格式
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

void *det_open(const char *cfg_name)
{
	Ctx *ctx = new Ctx(cfg_name);

#if 1
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#endif 

	return ctx;
}

void det_close(void *ins)
{
	Ctx *ctx = (Ctx*)ins;
	delete ctx;
}

const int _buf_size = 4096;
static char *_buf = (char*)malloc(_buf_size);
const char *_pre = "{ \"stamp\":12345, \"rect\":[";

const char *det_detect(void *det, zifImage *img)
{
	Ctx *ctx = (Ctx*)det;

	cv::Mat frame(img->height, img->width, CV_8UC3, img->data[0], img->stride[0]);
	int idx = -1;
	const std::vector<cv::Rect> &rcs = ctx->detect(frame, idx);

	strcpy(_buf, _pre);
	bool first = true;
	for (std::vector<cv::Rect>::const_iterator it = rcs.begin(); it != rcs.end(); ++it) {
		if (!first) {
			strcat(_buf, ",");
		}
		else {
			first = false;
		}

		char tmp[128];
		snprintf(tmp, sizeof(tmp), "{\"x\":%d, \"y\":%d, \"width\":%d, \"height\":%d}", it->x, it->y, it->width, it->height);
		strcat(_buf, tmp);
	}
	strcat(_buf, " ]");

	if (true) {
		char tmp[64];
		snprintf(tmp, sizeof(tmp), ", \"flipped_idx\": %d", idx);
		strcat(_buf, tmp);
	}

	strcat(_buf, "}");

	return _buf;
}

void det_set_param(void *det, int thres_dis, int thres_area, double factor_0, double factor_05, double notused)
{
	Ctx *ctx = (Ctx*)det;

	ctx->detector_->set_param(thres_dis, thres_area, factor_0, factor_05);
}

/** 这里从 frame 中提取站立学生区域 */
const std::vector<cv::Rect> &Ctx::detect(cv::Mat &frame, int &flipped_idx)
{
	if (masked_ && !flipped_) {
		cv::bitwise_and(frame, mask_, frame);
	}

	detector_->detect(frame, students_rects, flipped_idx);

	return students_rects;
}

void det_enable_gpu(void *det, int enabled)
{
}

void det_set_flipped_mode(void *det, int enabled)
{
	Ctx *ctx = (Ctx*)det;
	ctx->flipped_ = enabled == 1;
	ctx->detector_->set_flipped_mode(enabled);
}

void det_source_stats(void *det, int total, int lost, int pending, int cached)
{
	Ctx *ctx = (Ctx*)det;
	ctx->detector_->log("source stats: %d, %d, %d, %d\n", total, lost, pending, cached);
}
