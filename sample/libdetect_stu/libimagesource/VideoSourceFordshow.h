#pragma once

/** 利用 opencv VideoSource 类，从 dshow 设备获取图像 ....
 */

#include "image_source.h"

#include <string>
#include <cc++/thread.h>
#include <deque>

#include <DShow.h>

extern "C" {
#	include <libavformat/avformat.h>
#	include <libavcodec/avcodec.h>
#	include <libswscale/swscale.h>
}

class VideoSourceFordshow : ost::Thread
{
	std::string url_;
	imgsrc_format fmt_;
	bool quit_;
	ost::Semaphore sem_, sem_pics_;
	typedef std::deque<zifImage*> IMAGES;
	IMAGES fifo_, cached_;
	ost::Mutex cs_images_;

public:
	VideoSourceFordshow(const imgsrc_format *fmt, const char *url);
	~VideoSourceFordshow();

	zifImage *next_img();
	void free_img(zifImage *img);

private:
	void run();
	void try_list();

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

	void save_frame(zifImage *img)
	{
		ost::MutexLock al(cs_images_);

		fifo_.push_back(img);

		if (fifo_.size() >= 5) {
			fprintf(stderr, "zqpkt buf overflowed, max=%d!!!!!!!\n", 5);
			zifImage *tmp = next_img();
			free_img(tmp);
			//sem_pics_.wait();	// 必须对上号 ...，但似乎 ost::Semaphore 的实现有问题 ....
		}

		sem_pics_.post();
	}
};
