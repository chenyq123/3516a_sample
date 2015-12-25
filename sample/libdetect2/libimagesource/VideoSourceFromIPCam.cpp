#pragma once
#include "VideoSourceFromIPCam.h"
#include <stdio.h>
#include <assert.h>
#include <utility>

#ifndef PIX_FMT
#	define PIX_FMT PIX_FMT_RGB32
#endif 

#define __func__ __FUNCTION__

class IPCam_Init
{
public:
	IPCam_Init()
	{
		IP_NET_DVR_Init();
	}
};

static IPCam_Init _ipcam_init;

int max1(int a, int b)
{
	return a >= b ? a : b;
}

VideoSourceFromIPCam::VideoSourceFromIPCam(const char *url, const imgsrc_format *fmt)
{
	url_.assign(url);
	ow_ = fmt->width, oh_ = fmt->height;

	evt_ = CreateEvent(0, 0, 0, 0);

	dec_ = 0;
	sws_ = 0;
	frame_ = avcodec_alloc_frame();

	start();
}


VideoSourceFromIPCam::~VideoSourceFromIPCam(void)
{
	Message quit;
	post_message(quit);
	join();

	if (dec_) { avcodec_close(dec_); av_free(dec_); }
	if (sws_) sws_freeContext(sws_);
	avcodec_free_frame(&frame_);
	CloseHandle(evt_);

	flush();
}

int VideoSourceFromIPCam::prepare_dec_sws(const VIDEO_PARAM &vp)
{
	ost::MutexLock al(cs_dec_);

	if (dec_) { avcodec_close(dec_); av_free(dec_); dec_ = 0; }
	if (sws_) { sws_freeContext(sws_); sws_ = 0; }

	AVCodec *codec = 0;
	if (!stricmp("h264", vp.codec))
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	else if (!stricmp("mpeg4", vp.codec))
		codec = avcodec_find_decoder(AV_CODEC_ID_MPEG4);
	else {
		fprintf(stderr, "ERR: %s: only support h264 or mpeg4, code='%s' NOT supported!\n", __func__, vp.codec);
		return -1;
	}

	dec_ = avcodec_alloc_context3(codec);
	avcodec_open2(dec_, codec, 0);

	dec_->extradata_size = vp.vol_length;

	dec_->extradata = (uint8_t *)av_malloc(max1(vp.vol_length, FF_INPUT_BUFFER_PADDING_SIZE));
	memcpy(dec_->extradata, vp.vol_data, vp.vol_length);

	sws_ = sws_getContext(vp.width, vp.height, PIX_FMT_YUV420P, ow_, oh_, PIX_FMT, SWS_FAST_BILINEAR, 0, 0, 0);

	return 0;
}

static void dump_stream_av_param(const STREAM_AV_PARAM &param)
{
	fprintf(stdout, "INFO: RECVVIDEOAUDIOPARAM:\n");
	fprintf(stdout, "\tprotocol: %s\n"
		"\taudio: %d, video: %d\n", 
		param.ProtocolName, param.bHaveAudio, param.bHaveVideo);
	if (param.bHaveVideo) {
		fprintf(stdout, "\tVideo:\n"
			"\t\tcodec: %s\n"
			"\t\twidth=%d, height=%d\n"
			"\t\tbitrate: %d, framerate: %d\n",
			param.videoParam.codec,
			param.videoParam.width, param.videoParam.height,
			param.videoParam.bitrate, param.videoParam.framerate);
	}
	if (param.bHaveAudio) {
		fprintf(stdout, "\tAudio:\n"
			"\t\tcodec: %s\n",
			"\t\tchannel: %d, bits: %d, samples: %d\n",
			param.audioParam.codec,
			param.audioParam.channels, param.audioParam.bitspersample, param.audioParam.samplerate);
	}
}

void VideoSourceFromIPCam::run()
{
	if (url_.empty()) {
		fprintf(stderr, "ERR: %s: NO 'ipcam_ip' setting\n", __func__);
		//::exit(-1);
		return;
	}

	IP_NET_DVR_SetStatusEventCallBack(_cb_status, this);
	IP_NET_DVR_SetAutoReconnect(0, 1);

	LONG userid = 0;
	userid = IP_NET_DVR_Login((char*)url_.c_str(), 8091, "admin", "123456", 0);
	if (userid == 0) {
		fprintf(stderr, "ERR: %s: IP_NET_DVR_Login failure, url_='%s', ptzport=8091, user=admin, passwd=123456\n",
			__func__, url_.c_str());
		return;
	}

	LONG handle = 0;

	Message msg;
	while (next_message(msg)) {
		assert(msg.id == 1 || msg.id == 2);

		if (msg.id == 1) {
			switch (msg.ev) {
			case EVENT_CONNECTOK:
				{
					IP_NET_DVR_SetAutoReconnect(0, 0);
					USRE_VIDEOINFO info;
					info.bIsTcp = 1;
					info.nVideoPort = 554;
					info.pUserData = this;
					info.nVideoChannle = 0;	// 0 主码流，1 子码流

					handle = IP_NET_DVR_RealPlayEx(userid, (char*)url_.c_str(), "admin", "123456", _cb_data, &info, 0);

					if (handle == 0) {
						fprintf(stderr, "ERR: %s: IP_NET_DVR_RealPlayEx ret handle = 0, ERR\n", __func__);
						post_message(Message());
					}
				}
				break;

			case EVENT_CONNECTFAILED:
				{
					fprintf(stderr, "ERR: %s: EVENT_CONNECTFAILED recved!\n", __func__);
					post_message(Message());
				}
				break;

			case EVENT_RECVVIDEOPARAM:
				{
					VIDEO_PARAM vp;
					memset(&vp, 0, sizeof(vp));
					int a = IP_NET_DVR_GetVideoParam(handle, &vp);

					if (prepare_dec_sws(vp) < 0) {
						post_message(Message());
					}
				}
				break;

			case EVENT_RECVVIDEOAUDIOPARAM:
				{
					STREAM_AV_PARAM &param = *(STREAM_AV_PARAM*)msg.data;
					dump_stream_av_param(param);
				}
				break;
			}
		}

		if (msg.id == 2) {
			// TODO: ???
		}
	}
	
	IP_NET_DVR_StopRealPlay(handle);
	IP_NET_DVR_Logout(userid);

	fprintf(stderr, "%s: thread terminated!\n", __func__);
}

void VideoSourceFromIPCam::cb_status(LONG lUser, LONG nStateCode, char *pReason)
{
	fprintf(stderr, "INFO:%s: lUser=%d, nStateCode=%d\n", __func__, lUser, nStateCode);
	if (nStateCode == EVENT_CONNECTOK) {
		Message msg(EVENT_CONNECTOK, 0);
		post_message(msg);
	}
	if (nStateCode == EVENT_CONNECTFAILED) {
		Message msg(EVENT_CONNECTFAILED, 0);
		post_message(msg);
	}
	if (nStateCode == EVENT_RECVVIDEOPARAM) {
		Message msg(EVENT_RECVVIDEOPARAM, 0);
		post_message(msg);
	}
	if (nStateCode == EVENT_RECVVIDEOAUDIOPARAM) {
		void *ptr = malloc(sizeof(STREAM_AV_PARAM));
		memcpy(ptr, pReason, sizeof(STREAM_AV_PARAM));

		Message msg(EVENT_RECVVIDEOAUDIOPARAM, ptr);
		post_message(msg);
	}
}

typedef struct
{
	unsigned long flag;			//标记，固定为0x1a2b3c4d
	unsigned long data;			//最后面字节，即data & 0xff如果值不为0，则为当前实际帧率
	unsigned long frame_index;		//当前帧ID
	unsigned long keyframe_index;	//当前帧依赖的关键帧ID
} VIDEO_FRAME_HEADER;

void VideoSourceFromIPCam::cb_data(LONG lRealHandle, DWORD dwDataType, BYTE *data, DWORD len, LPFRAME_EXTDATA pExtData)
{
	if (dwDataType == 0) {
		// video
		double stamp = pExtData->timestamp;
		VIDEO_FRAME_HEADER *pHeader = (VIDEO_FRAME_HEADER *)(data + len - sizeof(VIDEO_FRAME_HEADER));
		if (pHeader->flag == 0x1a2b3c4d) {
			len -= sizeof(VIDEO_FRAME_HEADER);
		}

		save_video(data, len, stamp);
	}
	else if (dwDataType == 1) {
		// audio
	}
	else if (dwDataType == 2) {
		STREAM_AV_PARAM &param = *(STREAM_AV_PARAM*)data;
		dump_stream_av_param(param);
	}
}

void VideoSourceFromIPCam::save_frame(SwsContext *sws, AVFrame *frame, double stamp)
{
	zifImage *img = get_cached(ow_, oh_, PIX_FMT);

	sws_scale(sws, frame->data, frame->linesize, 0, frame->height, img->data, img->stride);
	img->width = ow_;
	img->height = oh_;
	img->fmt_type = PIX_FMT;
	img->stamp = stamp;

	ost::MutexLock al(cs_images_);

	int fifo_max_size = 5;
	if (fifo_.size() >= fifo_max_size) {
		//av_log(0, AV_LOG_ERROR, "zqpkt buf overflowed, max=%d!!!!!!!\n", atoi(cfg_->get_value("image_fifo_max_size", "5")));
		zifImage *tmp = next_img();
		free_img(tmp);
	}

	fifo_.push_back(img);

	SetEvent(evt_);
}

zifImage *VideoSourceFromIPCam::get_cached(int width, int height, int fmt)
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
		avpicture_alloc((AVPicture*)img->internal_ptr, PIX_FMT, width, height);
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

void VideoSourceFromIPCam::save_video(uint8_t *data, int len, double stamp)
{
	AVPacket pkg;
	av_init_packet(&pkg);

	pkg.data = data;
	pkg.size = len;

	int got;
	int rc = avcodec_decode_video2(dec_, frame_, &got, &pkg);
	if (got) {
		save_frame(sws_, frame_, stamp);
	}
}

void VideoSourceFromIPCam::flush()
{
	ost::MutexLock al(cs_images_);

	while (!fifo_.empty()) {
		zifImage *tmp = fifo_.front();
		fifo_.pop_front();
		free_img(tmp);
	}
}

zifImage *VideoSourceFromIPCam::next_img()
{
	WaitForSingleObject(evt_, 500);

	ost::MutexLock al(cs_images_);
	if (fifo_.empty()) {
		return 0;
	}

	zifImage *img = fifo_.front();
	fifo_.pop_front();

	return img;
}

void VideoSourceFromIPCam::free_img(zifImage *img)
{
	if (!img) return;

	ost::MutexLock al(cs_images_);

	cached_.push_back(img);

	int fifo_max_cached = 20;
	while (cached_.size() > fifo_max_cached) {
		zifImage *img = cached_.front();
		cached_.pop_front();

		avpicture_free((AVPicture*)img->internal_ptr);
		delete (AVPicture*)img->internal_ptr;
		delete img;
	}
}
