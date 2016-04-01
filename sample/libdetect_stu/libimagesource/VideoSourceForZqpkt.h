#pragma once

#include <zonekey/zqpsource.h>
#include <cc++/thread.h>
#include <deque>
extern "C" {
#	include <libavformat/avformat.h>
#	include <libavcodec/avcodec.h>
#	include <libswscale/swscale.h>
}
#include "image_source.h"

/** 对应图像源 */
class VideoSource : ost::Thread
{
	// 工作线程从 zqpkt 接收数据，并保存到 images_ 中，next_img() 从中提取
	typedef std::deque<zifImage*> IMAGES;
	IMAGES fifo_, cached_;
	ost::Mutex cs_images_;
	void *src_;
	bool quit_;
	AVCodecContext *dec_;
	AVFrame *frame_;
	HANDLE evt_;
	int ow_, oh_;	// 要求的输出的大小
	unsigned int cnt_;
	bool opened_;
	std::string url_;
	imgsrc_format fmt_;

	enum Mode
	{
		M_TCP,		// 用于 tcp://... zqpkt 源
		M_LOCAL,	// 用于 jp100hd 的预览
	};

	Mode mode_;

public:
	VideoSource() : opened_(false)
	{
	}

	/** 打开 tcp://... 图像源，使用 zqpkt 格式 */
	int open(const imgsrc_format *fmt, const char *url);
	void close();

	/** 获取下一帧图像，应尽快，否则会溢出丢失 */
	zifImage *next_img();
	void free_img(zifImage *img);

	/** 清空缓冲 */
	void flush();

	/** 仅仅 M_LOCAL，
	 */
	int write_yuv420p(int width, int height, unsigned char *data[], int stride[]);

private:
	void run();
	void save_frame(SwsContext *sws, AVFrame *frame, double stamp);
	zifImage *get_cached(int width, int height, int fmt);
};
