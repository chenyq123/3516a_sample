#include <cc++/thread.h>
#include "image_source.h"

#include "VideoSourceForYuan.h"
#include "VideoSourceForZqpkt.h"
//#include "VideoSourceFromIPCam.h"
#include "VideoSourceForRTSP.h"
#include "VideoSourceFordshow.h"
#include "VideoSourceFFmpeg.h"

#if SUPPORT_MORN
#	include "VideoSourceForMorn.h"
#endif

#include "cJSON.h"

#ifdef WIN32
struct imgsrc_t
{
#if SUPPORT_YUANSDK
	VideoSourceForYuan *yuan_;
#endif
	VideoSource *zqpkt_;
#if 0
	VideoSourceFromIPCam *ip_;
#endif
	VideoSourceForRTSP *rtsp_;
	VideoSourceFordshow *dshow_;
	VideoSourceFFmpeg *ffmpeg_;
#if SUPPORT_MORN
	VideoSourceForMorn *vsm_;
#endif

public:
	imgsrc_t(const char *url, const imgsrc_format *fmt)
	{
#if SUPPORT_YUANSDK
		yuan_ = 0;
#endif
		zqpkt_ = 0;
		//ip_ = 0;
		rtsp_ = 0;
		dshow_ = 0;
		ffmpeg_ = 0;
#if SUPPORT_MORN
		vsm_ = 0;
#endif

		if (!strncmp(url, "tcp://", 6)) {
			zqpkt_ = new VideoSource;
			zqpkt_->open(fmt, url);
		}
#if SUPPORT_YUANSDK
		else if (!strncmp(url, "yuan://", 7)) {
			yuan_ = new VideoSourceForYuan(fmt, atoi(url+7));
		}
#endif
		else if (!strncmp(url, "rtsp://", 7)) {
			//rtsp_ = new VideoSourceForRTSP(fmt, url);
			ffmpeg_ = new VideoSourceFFmpeg(fmt, url);
			ffmpeg_->start();
		}
#if SUPPORT_MORN
		else if(!strncmp(url, "morn://", 7)) {
			char ip[64];
			sscanf(url, "morn://%s", ip);
			vsm_ = new VideoSourceForMorn(ip, fmt);
		}
#endif

#if 0
		else if (!strncmp(url, "ipcam://", 8)) {
			char ip[64];
			sscanf(url, "ipcam://%s", ip);

			ip_ = new VideoSourceFromIPCam(ip, fmt);
		}
#endif
		else if (!strncmp(url, "dshow://", 8)) {
			dshow_ = new VideoSourceFordshow(fmt, url);
		}
		else {
			ffmpeg_ = new VideoSourceFFmpeg(fmt, url);
			ffmpeg_->start();
		}
	}

	~imgsrc_t()
	{
#if SUPPORT_YUANSDK
		delete yuan_;
#endif
		if (zqpkt_) {
			zqpkt_->close();
			delete zqpkt_;
		}
		else if (ffmpeg_) {
			ffmpeg_->stop();
			delete ffmpeg_;
		}

#if 0
		else if (ip_)
			delete(ip_);
#endif
#if SUPPORT_MORN
		else if (vsm_)
			delete(vsm_);
#endif
		else if (dshow_)
			delete dshow_;
		else
			delete rtsp_;
	}

	zifImage *next()
	{
		if (zqpkt_)
			return zqpkt_->next_img();
		else if (ffmpeg_) {
			return ffmpeg_->next_img();
		}
#if SUPPORT_YUANSDK
		else if (yuan_)
			return yuan_->next_img();
#endif
#if 0
		else if (ip_)
			return ip_->next_img();
#endif
#if SUPPORT_MORN
		else if (vsm_)
			return vsm_->next_img();
#endif
		else if (dshow_)
			return dshow_->next_img();
		else if (rtsp_)
			return rtsp_->next_img();
		else {
			ost::Thread::sleep(500);
			return 0;
		}
	}

	int pause()
	{
		if (ffmpeg_) {
			ffmpeg_->pause();
		}

		return 0;
	}

	int resume()
	{
		if (ffmpeg_) {
			ffmpeg_->resume();
		}

		return 0;
	}

	void free(zifImage *img)
	{
		if (zqpkt_)
			zqpkt_->free_img(img);
#if SUPPORT_YUANSDK
		else if (yuan_)
			yuan_->free_img(img);
#endif
		else if (ffmpeg_) {
			ffmpeg_->free_img(img);
		}
		else if (rtsp_)
			rtsp_->free_img(img);
		else if (dshow_)
			dshow_->free_img(img);
#if SUPPORT_MORN
		else if (vsm_)
			vsm_->free_img(img);
#endif
#if 0
		else
			ip_->free_img(img);
#endif
	}
};
#else
struct imgsrc_t
{
	imgsrc_t(const char *url, const imgsrc_format *fmt)
	{}

	zifImage *next()
	{
		return 0;
	}
	void free(zifImage *img)
	{}
};
#endif
imgsrc_t *imgsrc_open(const char *url, const imgsrc_format *fmt)
{
	return new imgsrc_t(url, fmt);
}

imgsrc_t *imgsrc_open_json(const char *url, const char *fmt_json_str)
{
	imgsrc_format fmt;
	fmt.fmt = AV_PIX_FMT_BGR24;
	fmt.width = 960;
	fmt.height = 540;
	fmt.fps = 0.0;

	if (fmt_json_str) {
		cJSON *j = cJSON_Parse(fmt_json_str);
		if (j && j->child) {
			cJSON *j2 = j->child;
			while (j2) {
				if (!strcmp(j2->string, "pix_fmt")) {
					if (j2->type == cJSON_Number)
						fmt.fmt = (PixelFormat)j2->valueint;
				}
				if (!strcmp(j2->string, "width")) {
					if (j2->type == cJSON_Number)
						fmt.width = j2->valueint;
				}
				if (!strcmp(j2->string, "height")) {
					if (j2->type == cJSON_Number)
						fmt.height = j2->valueint;
				}
				if (!strcmp(j2->string, "fps")) {
					if (j2->type == cJSON_Number)
						fmt.fps = j2->valuedouble;
				}

				j2 = j2->next;
			}
		}

		if (j)
			cJSON_Delete(j);
	}

	fprintf(stderr, "DEBUG: %s: using { %d, %d, %d, %f }\n", __FUNCTION__, fmt.fmt, fmt.width, fmt.height, fmt.fps);

	return imgsrc_open(url, &fmt);
}

void imgsrc_close(imgsrc_t *ctx)
{
	delete ctx;
}

zifImage *imgsrc_next(imgsrc_t *ctx)
{
	return ctx->next();
}

void imgsrc_free(imgsrc_t *ctx, zifImage *img)
{
	ctx->free(img);
}

int imgsrc_pause(imgsrc_t *ctx)
{
	return ctx->pause();
}

int imgsrc_resume(imgsrc_t *ctx)
{
	return ctx->resume();
}
