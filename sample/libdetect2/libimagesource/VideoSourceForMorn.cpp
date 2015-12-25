#if SUPPORT_MORN
#include "VideoSourceForMorn.h"
#include <assert.h>

#pragma comment(lib, "HHNetClient.lib")
#pragma comment(lib, "HH5KDecoder.lib")

#ifndef PIX_FMT
# define PIX_FMT PIX_FMT_BGR24
#endif
VideoSourceForMorn::VideoSourceForMorn(const char *url, const imgsrc_format *fmt)
{
	url_.assign(url);
	width_ = fmt->width, height_ = fmt->height;

	evt_ =  CreateEvent(0, 0, 0, 0);

	dec_ = 0;
	sws_ = 0;
	frame_ = avcodec_alloc_frame();

	quit_ = false;
	start();
}

VideoSourceForMorn::~VideoSourceForMorn()
{
	quit_ = true;
	join();

	if (dec_) { avcodec_close(dec_); av_free(dec_); }
	if (sws_) sws_freeContext(sws_);
	avcodec_free_frame(&frame_);
	CloseHandle(evt_);

	HHNET_CloseChannel(hOpenChannel_);
}



void VideoSourceForMorn::run()
{
	HHERR_CODE errCode;
	HHOPEN_CHANNEL_INFO_EX openInfo_ex;

	openInfo_ex.nSubChannel = 0;
	openInfo_ex.dwClientID = 0;
	openInfo_ex.nOpenChannel = 0;
	openInfo_ex.res = 100;
	openInfo_ex.protocolType = NET_PROTOCOL_TCP;
	openInfo_ex.funcStreamCallback = (ChannelStreamCallback)_ChannelStreamCallback;
	openInfo_ex.pCallbackContext= this;


	errCode = HHNET_OpenChannel(
					(char*)url_.c_str(),
					5000,
					"IPC50525",
					"admin",
					"admin",
					(HHOPEN_CHANNEL_INFO*)&openInfo_ex,
					hOpenChannel_
					);
	fprintf(stderr, "%s hOpen = %d\n", __FUNCTION__, hOpenChannel_);
	fprintf(stderr, "%s errCode = %d\n", __FUNCTION__, errCode);
	
	while (!quit_){ 
		;
	}
}

void VideoSourceForMorn::save_frame(SwsContext *sws, AVFrame *frame, double stamp)
{
		zifImage *img = get_cached(width_, height_, PIX_FMT);
		sws_scale(sws, frame->data, frame->linesize, 0, frame->height, img->data, img->stride);
		img->width = width_;
		img->height = height_;
		img->fmt_type = PIX_FMT;
		img->stamp = stamp;

		ost::MutexLock al(cs_images_);

		if(fifo_.size() >= 5) {
				zifImage *tmp = next_img();
				free_img(tmp);
		}

		fifo_.push_back(img);

		SetEvent(evt_);
}

zifImage *VideoSourceForMorn::get_cached(int width, int height, int fmt)
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

void VideoSourceForMorn::save_video(uint8_t *data, int len, double stamp)
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

void VideoSourceForMorn::flush()
{
		ost::MutexLock al(cs_images_);

		while (!fifo_.empty()) {
				zifImage *tmp = fifo_.front();
				free_img(tmp);
		}
}

zifImage *VideoSourceForMorn::next_img()
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

void VideoSourceForMorn::free_img(zifImage *img)
{
	if (!img) return;

	ost::MutexLock al(cs_images_);
	
	cached_.push_back(img);
	while (cached_.size() > 20) {
			zifImage *img = cached_.front();
			cached_.pop_front();

			avpicture_free((AVPicture*)img->internal_ptr);
			delete (AVPicture*)img->internal_ptr;
			delete img;
	} }


int VideoSourceForMorn::prepare_dec_sws(const EXT_FRAME_VIDEO &efv)
{
	ost::MutexLock al(cs_dec_);

	if (dec_) { avcodec_close(dec_); av_free(dec_); dec_ = 0; }
	if (sws_) { sws_freeContext(sws_); sws_ = 0; }

	AVCodec *codec = 0;
	
	if (efv.nVideoEncodeType == 0x01) 
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (efv.nVideoEncodeType == 0x03)
		codec = avcodec_find_decoder(AV_CODEC_ID_MPEG4);
	else {
			fprintf(stderr, "ERR: %s: only support h264 or mpeg4, code = '%s' NoT supported!\n",__FUNCTION__, efv.nVideoEncodeType);
			return -1;
	}
	
	dec_ = avcodec_alloc_context3(codec);
	avcodec_open2(dec_, codec, 0);

	dec_->extradata_size = 0;
	dec_->extradata = 0;

	sws_ = sws_getContext(efv.nVideoWidth, efv.nVideoHeight, PIX_FMT_YUV420P, width_, height_, PIX_FMT, SWS_FAST_BILINEAR, 0, 0, 0);

	return 0;
}

int VideoSourceForMorn::ChannelStreamCallback_(HANDLE hOpenChannel,
	void *pStreamData,
	DWORD dwClientID,
	void *pContext,
	ENCODE_VIDEO_TYPE encodeVideoType,
	HHAV_INFO *pAVInfo)
{

	EXT_FRAME_HEAD *efh = (EXT_FRAME_HEAD*)((BYTE*)pStreamData + sizeof(HV_FRAME_HEAD));
	assert(efh->CheckExtFlag() == true);

	HV_FRAME_HEAD *hfh = (HV_FRAME_HEAD*)(pStreamData);
	if (hfh->streamFlag == MY_FRAME_TYPE_I || hfh->streamFlag == MY_FRAME_TYPE_P) {
		BYTE *data = (BYTE*)pStreamData + sizeof(HV_FRAME_HEAD) + sizeof(EXT_FRAME_HEAD);
		int len = hfh->nByteNum - sizeof(EXT_FRAME_HEAD);

		save_video((uint8_t*)data, len, hfh->nTimestamp);
	}
	
	return 0;
}

static void *DecodeMorn(void *data)
{
	HV_FRAME_HEAD *hfh = (HV_FRAME_HEAD*)data;
	EXT_FRAME_HEAD *efh = (EXT_FRAME_HEAD*)((BYTE*)data + sizeof(HV_FRAME_HEAD));
	    
	data = (BYTE*)data + sizeof(EXT_FRAME_HEAD);
	return data; 	   
}
#endif //
