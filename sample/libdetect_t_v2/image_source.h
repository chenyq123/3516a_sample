/** 获取图像
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif // c++
#ifndef WIN32
#define __STDC_CONSTANT_MACROS
#define SUPPORT_YUANSDK
#endif
//#include <libswscale/swscale.h>

typedef struct imgsrc_t imgsrc_t;

typedef struct imgsrc_format
{
	PixelFormat fmt;
	int width;
	int height;
	double fps;
} imgsrc_format;


typedef struct zifImage
{
	PixelFormat fmt_type;
	int width;
	int height;
	unsigned char *data[4];
	int stride[4];

	double stamp;

	void *internal_ptr;
} zifImage;

/** url 支持两帧格式：
		tcp://xxx zqpkt的直播流 ..
		yuan://N 直接使用yuan sdk读取 N 通道数据 ..
		ipcam://.. 访问 ip 摄像机 ..
		rtsp://.. 访问 rtsp 源 ..
 */
imgsrc_t *imgsrc_open(const char *url, const imgsrc_format *fmt);

/** 支持 json 格式描述的 format，这样将方便 python 调用 ...
		{
			"pix_fmt": 3,
			"width": 960,
			"height": 540,
			"fps": 0.0
		}
 */
imgsrc_t *imgsrc_open_json(const char *url, const char *fmt_json_str);
void imgsrc_close(imgsrc_t *ctx);

/** 暂停/恢复:
		目前仅仅支持文件
 */
int imgsrc_pause(imgsrc_t *ctx);
int imgsrc_resume(imgsrc_t *ctx);

/** 获取下一帧图像，如内部缓冲超过500ms没有图像，则返回 0
	如果返回有效图像，使用完成后，必须调用 imgsrc_free() 释放！
 */
zifImage *imgsrc_next(imgsrc_t *ctx);
void imgsrc_free(imgsrc_t *ctx, zifImage *img);

#ifdef __cplusplus
}
#endif // c++
