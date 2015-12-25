#include "VideoSourceForZqpkt.h"
#include <assert.h>

extern "C"  {
#	include <libavdevice/avdevice.h>
}

#if FFMPEG_DXVA2

typedef int (*PFN_DXVA2_INIT)(AVCodecContext *cc);

/** 从 libffmpeg.dxva2.dll 中加载 dxva2_init() */
static HMODULE _dll_dxva2;
PFN_DXVA2_INIT _func_dxva2_init = 0;

#endif 

static void log_init()
{
}

static void log(const char *fmt, ...)
{
}

class AVCodecInit
{
	ost::Mutex cs_;

public:
	AVCodecInit()
	{
		av_register_all();
		avdevice_register_all();	// dshow ...
		avcodec_register_all();
		avformat_network_init();
		log_init();

		av_log_set_level(AV_LOG_FATAL);

#if FFMPEG_DXVA2
		_dll_dxva2 = LoadLibrary("libffmpeg.dxva2.dll");
		if (_dll_dxva2) {
			fprintf(stderr, "INFO: en, load libffmpeg.dxva2.dll OK\n");

			_func_dxva2_init = (PFN_DXVA2_INIT)GetProcAddress(_dll_dxva2, "dxva2_init");
			if (!_func_dxva2_init) {
				fprintf(stderr, "ERR: oooh, can't get dxva2_init() func ptr ?????\n");
			}
		}
		else {
			fprintf(stderr, "WARNING: ooh, can't load libffmpeg.dxva2.dll\n");
		}
#endif
	}

	ost::Mutex &cs()
	{
		return cs_;
	}
};
static AVCodecInit _avcodec_init;

int VideoSource::open(const imgsrc_format *fmt, const char *url)
{
	fmt_ = *fmt;
	ow_ = fmt->width, oh_ = fmt->height;	// 要求输出的大小 
	src_ = 0;
	evt_ = CreateEvent(0, 0, 0, 0);

	url_ = url;
	{
		ost::MutexLock al(_avcodec_init.cs());

		AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
		dec_ = avcodec_alloc_context3(codec);

		avcodec_open2(dec_, codec, 0);
		frame_ = avcodec_alloc_frame();
	}

	quit_ = false;
	start();

	opened_ = true;
	
	return 0;
}

void VideoSource::close()
{
	if (!opened_) return;

	quit_ = true;
	join();

	avcodec_free_frame(&frame_);
	//zqpsrc_close(src_);

	CloseHandle(evt_);
	opened_ = false;
}

void VideoSource::flush()
{
	ost::MutexLock al(cs_images_);

	while (!fifo_.empty()) {
		zifImage *tmp = fifo_.front();
		fifo_.pop_front();
		free_img(tmp);
	}
}

zifImage *VideoSource::next_img()
{
	if (!opened_)
		return 0;

	WaitForSingleObject(evt_, 50);

	ost::MutexLock al(cs_images_);
	if (fifo_.empty()) {
		return 0;
	}

	zifImage *img = fifo_.front();
	fifo_.pop_front();

	return img;
}

void VideoSource::free_img(zifImage *img)
{
	if (!img) return;

	ost::MutexLock al(cs_images_);

	cached_.push_back(img);
	while (cached_.size() > 10) {
		zifImage *img = cached_.front();
		cached_.pop_front();

		avpicture_free((AVPicture*)img->internal_ptr);
		delete (AVPicture*)img->internal_ptr;
		delete img;
	}
}

void VideoSource::run()
{
	SwsContext *sws = 0;
	unsigned int cnt = 0;	// 用于方便丢帧

	while (!quit_) {
		if (zqpsrc_open(&src_, url_.c_str()) < 0) {
			log("ERR: open %s err\n", url_.c_str());

			fprintf(stderr, "ERR: %s: can't open url=%s\n", __FUNCTION__, url_.c_str());
			sleep(5*1000);
			continue;
		}

		while (!quit_) {
			zq_pkt *pkt = zqpsrc_getpkt(src_);
			if (!pkt) {
				log("ERR: %s: net broken!\n", url_.c_str());
				break;
			}

			if (pkt->type == 1) {
				// video
				AVPacket pkg;
				//memset(&pkg, 0, sizeof(pkg));
				av_init_packet(&pkg);

				pkg.data = (uint8_t*)pkt->ptr;
				pkg.size = pkt->len;

				int got;
				int rc = 0;
				__try {
					rc = avcodec_decode_video2(dec_, frame_, &got, &pkg);
				}
				__except(1) {
					got = 0;
					fprintf(stderr, "EXCEPT: avcodec_decode_video2 ....\n");
				}

				if (got) {
					if (!sws) {
						sws = sws_getContext(frame_->width, frame_->height, PIX_FMT_YUV420P, ow_, oh_, fmt_.fmt, SWS_FAST_BILINEAR, 0, 0, 0);
					}

					cnt++;
					if (cnt % 3 == 0) {
						save_frame(sws, frame_, pkt->pts/45000.0);
					}
				}
			}

			zqpsrc_freepkt(src_, pkt);
		}

		zqpsrc_close(src_);
	}
}

void VideoSource::save_frame(SwsContext *sws, AVFrame *frame, double stamp)
{
	zifImage *img = get_cached(ow_, oh_, fmt_.fmt);
	
	sws_scale(sws, frame->data, frame->linesize, 0, frame->height, img->data, img->stride);
	img->width = ow_;
	img->height = oh_;
	img->fmt_type = fmt_.fmt;
	img->stamp = stamp;

	ost::MutexLock al(cs_images_);

	if (fifo_.size() >= 5) {
		//fprintf(stderr, "ERROR: zqpkt buf overflowed, max=%d!!!!!!!\n", 5);
		zifImage *tmp = next_img();
		free_img(tmp);
	}

	fifo_.push_back(img);

	SetEvent(evt_);
}

zifImage *VideoSource::get_cached(int width, int height, int fmt)
{
	ost::MutexLock al(cs_images_);
	if (!cached_.empty()) {
		zifImage *img = cached_.front();
		cached_.pop_front();

		assert(width == img->width && height == img->height && fmt == img->fmt_type);

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
