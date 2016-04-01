#include "VideoSourceFordshow.h"
#include <assert.h>

extern "C" {
#	include <libavformat/avformat.h>
#	include <libavdevice/avdevice.h>
#	include <libavcodec/avcodec.h>
}

#pragma comment(lib, "strmiids")

static IEnumMoniker * _getEnumerator()
{
	ICreateDevEnum *en;
	HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, 0, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&en));
	if (SUCCEEDED(hr)) {
		IEnumMoniker *emoniker;
		hr = en->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &emoniker, 0);
		en->Release();
		
		if (hr != S_FALSE)
			return emoniker;
		else {
			fprintf(stderr, "ERR: %s: CreateClassEnumerator for VideoInputDeviceCategory err, code=%08x\n", __FUNCTION__, hr);
		}
	}
	else {
		fprintf(stderr, "ERR: %s: CoCreateInstance for CLSID_SystemDeviceEnum ERR???, code=%08x\n", __FUNCTION__, hr);
	}
	return 0;
}

static void _displayInfo(IEnumMoniker *emoniker)
{
	IMoniker *moniker;
	int index = 0;

	while (emoniker->Next(1, &moniker, 0) == S_OK) {
		IPropertyBag *pPropBag;
		HRESULT hr = moniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
		if (FAILED(hr))
		{
			moniker->Release();
			continue;  
		} 

		fprintf(stderr, "index: %d\t", index);
		index++;

		VARIANT var;
		VariantInit(&var);

		// Get description or friendly name.
		hr = pPropBag->Read(L"Description", &var, 0);
		if (FAILED(hr))
		{
			hr = pPropBag->Read(L"FriendlyName", &var, 0);
		}
		if (SUCCEEDED(hr))
		{
			fprintf(stderr, "%S\n", var.bstrVal);
			VariantClear(&var); 
		}

		hr = pPropBag->Write(L"FriendlyName", &var);

		// WaveInID applies only to audio capture devices.
		hr = pPropBag->Read(L"WaveInID", &var, 0);
		if (SUCCEEDED(hr))
		{
			//printf("WaveIn ID: %d\n", var.lVal);
			VariantClear(&var); 
		}

		hr = pPropBag->Read(L"DevicePath", &var, 0);
		if (SUCCEEDED(hr))
		{
			// The device path is not intended for display.
			//printf("Device path: %S\n", var.bstrVal);
			VariantClear(&var); 
		}

		pPropBag->Release();
		moniker->Release();
	}
}

VideoSourceFordshow::VideoSourceFordshow(const imgsrc_format *fmt, const char *url)
	: url_(url)
	, fmt_(*fmt)
{
	quit_ = false;
	start();
}

VideoSourceFordshow::~VideoSourceFordshow()
{
	sem_.post();
	quit_ = true;
	join();
}

static double time_now()
{
	struct timeval tv;
	ost::gettimeofday(&tv, 0);
	return tv.tv_sec + tv.tv_usec / 1000000.0;
}

void VideoSourceFordshow::run()
{
	std::string dev_name = url_.substr(8);

	if (dev_name == "list") {
		try_list();
		::exit(1);
	}

	dev_name = "video=" + dev_name;

	/// 使用 ffmpeg 获取 ...
	AVFormatContext *fmtctx = 0;
	AVInputFormat *iformat = av_find_input_format("dshow");
	AVCodecContext *decoder = 0;
	int video_index = -1;
	AVFrame *frame = avcodec_alloc_frame();
	SwsContext *sws = 0;

	bool opened = false;
	size_t cnt = 0;		// 为了扔帧 .....

	while (!quit_) {
		if (!opened) {
			int rc = avformat_open_input(&fmtctx, dev_name.c_str(), iformat, 0);
			if (rc < 0) {
				fprintf(stderr, "ERR: %s: can't open dshow named '%s'\n", __FUNCTION__, dev_name.c_str());
				sem_.wait(5000);
				continue;
			}

			fprintf(stderr, "DEBUG: %s: dshow named '%s' opened ok!\n", __FUNCTION__, dev_name.c_str());
			if (avformat_find_stream_info(fmtctx, 0) < 0) {
				fprintf(stderr, "ERR: %s: av_find_stream_info err!!!\n", __FUNCTION__);
				avformat_close_input(&fmtctx);
				sem_.wait(5000);
				continue;
			}

			fprintf(stderr, "DEBUG: %s: dshow named '%s' query ok!, maybe run\n", __FUNCTION__, dev_name.c_str());
			av_dump_format(fmtctx, 0, dev_name.c_str(), 0);

			for (int i = 0; i < fmtctx->nb_streams; i++) {
				if (fmtctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
					video_index = i;
					break;
				}
			}

			if (video_index < 0) {
				fprintf(stderr, "DEBUG: %s: dshow named '%s' is NOT support VIDEO!!!\n", __FUNCTION__, dev_name.c_str());
				avformat_close_input(&fmtctx);
				sem_.wait(5000);
				continue;
			}

			decoder = fmtctx->streams[video_index]->codec;

			fprintf(stderr, "DEBUG: dshow named '%s' video index=%d\n", dev_name.c_str(), video_index);

			AVCodec *codec = avcodec_find_decoder(decoder->codec_id);
			if (!codec) {
				fprintf(stderr, "ERR: %s: dshow named '%s' can't find decoder ??? codec_id=%d\n", __FUNCTION__, dev_name.c_str(), decoder->codec_id);
				avformat_close_input(&fmtctx);
				sem_.wait(5000);
				continue;;
			}

			avcodec_open2(decoder, codec, 0);

			fprintf(stderr, "DEBUG: dshow named '%s' video decoder opened!\n", dev_name.c_str());

			opened = true;
		}

		if (opened) {
			AVPacket pkt;
			if (av_read_frame(fmtctx, &pkt) < 0) {
				fprintf(stderr, "ERR: dshow named '%s' av_read_frame err!\n", __FUNCTION__, dev_name.c_str());
				sleep(5000);
				opened = false;
				avformat_close_input(&fmtctx);
				if (sws) {
					sws_freeContext(sws);
					sws = 0;
				}
				continue;
			}

			if (pkt.stream_index != video_index) {
				av_free_packet(&pkt);
				continue;
			}

			int got;
			avcodec_decode_video2(decoder, frame, &got, &pkt);
			av_free_packet(&pkt);

			if (got) {
				if (decoder->width != fmt_.width || decoder->height != fmt_.height || decoder->pix_fmt != fmt_.fmt || !sws) {
					if (sws) sws_freeContext(sws);
					sws = sws_getContext(decoder->width, decoder->height, decoder->pix_fmt,
						fmt_.width, fmt_.height, fmt_.fmt,
						SWS_FAST_BILINEAR, 0, 0, 0);
					if (!sws) {
						fprintf(stderr, "ERR: dshow named '%s' create sws failure: [%dx%d, %d], [%dx%d, %d]\n", dev_name.c_str(),
							decoder->width, decoder->height, decoder->pix_fmt,
							fmt_.width, fmt_.height, fmt_.fmt);
						sleep(5000);

						opened = false;
						avformat_close_input(&fmtctx);
						if (sws) {
							sws_freeContext(sws);
							sws = 0;
						}
						continue;
					}
				}

				if (++cnt % 2)
					continue;	// 扔掉一般吧 ..

				assert(sws);

				zifImage *zif = get_cached(fmt_.width, fmt_.height, fmt_.fmt);
				assert(zif);

				AVPicture *pic = (AVPicture*)zif->internal_ptr;

				sws_scale(sws, frame->data, frame->linesize, 0, decoder->height, pic->data, pic->linesize);
				
				zif->stamp = time_now();

				save_frame(zif);
			}
		}
	}

	avcodec_free_frame(&frame);
	if (fmtctx) {
		avformat_close_input(&fmtctx);
	}

	if (sws)
		sws_freeContext(sws);
}

zifImage *VideoSourceFordshow::next_img()
{
	if (sem_pics_.wait(50)) {
		ost::MutexLock al(cs_images_);
		if (fifo_.empty())
			return 0;

		zifImage *img = fifo_.front();
		fifo_.pop_front();
		return img;
	}
	else {
		return 0;
	}
}

void VideoSourceFordshow::free_img(zifImage *img)
{
	if (img) {
		ost::MutexLock al(cs_images_);

		cached_.push_back(img);
		while (cached_.size() > 5) {
			zifImage *img = cached_.front();
			cached_.pop_front();

			avpicture_free((AVPicture*)img->internal_ptr);
			delete (AVPicture*)img->internal_ptr;
			delete img;
		}
	}
}

void VideoSourceFordshow::try_list()
{
	CoInitialize(0);
	fprintf(stderr, "%s: ...\n", __FUNCTION__);
	IEnumMoniker *em = _getEnumerator();
	if (em) {
		_displayInfo(em);
		em->Release();
	}
}
