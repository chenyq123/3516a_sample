#pragma once

#include "image_source.h"
#include <string>
#include <cc++/thread.h>
#include <deque>

extern "C" {
#	include <libavformat/avformat.h>
#	include <libavcodec/avcodec.h>
#	include <libswscale/swscale.h>
}

class VideoSourceForRTSP : ost::Thread
{
	std::string url_;
	imgsrc_format fmt_;
	bool quit_;
	ost::Semaphore sem_, sem_pics_;
	typedef std::deque<zifImage*> IMAGES;
	IMAGES fifo_, cached_;
	ost::Mutex cs_images_;

public:
	VideoSourceForRTSP(const imgsrc_format *fmt, const char *url);
	~VideoSourceForRTSP(void);

	/** 获取下一帧图像，应尽快，否则会溢出丢失 */
	zifImage *next_img();
	void free_img(zifImage *img);

private:
	void run();
	bool wait(int ms);

	/* 解码一帧，并拉伸到需要的格式，保存到pending列表中 */
	int one_video_frame(SwsContext *sws, AVCodecContext *ctx, AVPacket *pkg, AVFrame *frame, bool save);

	void save_frame(SwsContext *sws, AVFrame *frame, double stamp)
	{
		zifImage *img = get_cached(fmt_.width, fmt_.height, fmt_.fmt);

		sws_scale(sws, frame->data, frame->linesize, 0, frame->height, img->data, img->stride);
		img->width = fmt_.width;
		img->height = fmt_.height;
		img->fmt_type = fmt_.fmt;
		img->stamp = stamp;

		ost::MutexLock al(cs_images_);

		if (fifo_.size() >= 5) {
			av_log(0, AV_LOG_ERROR, "zqpkt buf overflowed, max=%d!!!!!!!\n", 5);
			zifImage *tmp = next_img();
			free_img(tmp);
			//sem_pics_.wait();	// 必须对上号 ...，但似乎 ost::Semaphore 的实现有问题 ....
		}

		fifo_.push_back(img);
		sem_pics_.post();
	}

	zifImage *get_cached(int width, int height, int fmt)
	{
		ost::MutexLock al(cs_images_);
		if (!cached_.empty()) {
			zifImage *img = cached_.front();
			cached_.pop_front();
			return img;
		}
		else {
			zifImage *img = new zifImage;
			img->internal_ptr = new AVPicture;
			avpicture_alloc((AVPicture*)img->internal_ptr, fmt_.fmt, width, height);
			img->width = width;
			img->height = height;
			img->fmt_type = (PixelFormat)fmt;

			for (int i = 0; i < 4; i++) {
				img->data[i] = ((AVPicture*)img->internal_ptr)->data[i];
				img->stride[i] = ((AVPicture*)img->internal_ptr)->linesize[i];
			}
			return img;
		}
	}
};
