#pragma once

#ifndef _libdetect_t_h_
#define _libdetect_t_h_
#include<opencv2/opencv1.hpp>
#include "detect_t.h"
#include "blackboard_detect.h"
#include "StudentTrack.h"
//#include "../libimagesource/image_source.h"
#include <string>
#include "hog.h"

#ifdef __cplusplus
extern "C" {
#endif

struct zifImage
{
    int fmt_type;
    int width;
    int height;
    unsigned char *data[4];
    int stride[4];
};

struct det_t
{
	KVConfig *cfg_;
	TeacherDetecting *detect_;
	BlackboardDetecting *bd_detect_;
    CStudentTrack *stu_detect_;
	zk_hog_detector *hog_det;
	IplImage *masked_;
	bool t_m;
	bool b_m;
    bool s_m;
	std::string result_str;	// FIXME: 希望足够了;
};

typedef struct det_t det_t;
typedef struct zifImage zifImage;
/** 返回实例
@param cfg_name: 配置文件名字
	@return det_t实例，0 失败
 */
det_t *det_open(const char *cfg_name);

void det_close(det_t *ctx);


/**
	@param img: 图像
	@return  ,json格式--   {"stamp":12345,"rect":[{"x":0,"y":0,"width":100,"height":100},{"x":0,"y":0,"width":100,"height":100}]}





	{
		"stamp":12345,
		"rect":
		[
			{"x":0,"y":0,"width":100,"height":100},
			{"x":0,"y":0,"width":100,"height":100}
		]
	}
 */
//char *det_detect(det_t *ctx, zifImage *img);
char *det_detect(det_t *ctx, cv::Mat &img);


#ifdef __cplusplus
};
#endif





#endif
