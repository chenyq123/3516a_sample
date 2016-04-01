#pragma once

#ifndef _libdetect_t_h_
#define _libdetect_t_h_
#include "opencv2/opencv1.hpp"
#include"blackboard_detect.h"
#include <string>
#include <cstdio>

#ifdef __cplusplus
extern "C" {
#endif
struct det_t
{
	KVConfig *cfg_;
	BlackboardDetecting *bd_detect_;
	IplImage *masked_;
	bool t_m;
	bool b_m;
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
char *det_detect(det_t * ctx, int height,int width,unsigned char *imgdata);//imgdata  YUV


#ifdef __cplusplus
};
#endif





#endif
