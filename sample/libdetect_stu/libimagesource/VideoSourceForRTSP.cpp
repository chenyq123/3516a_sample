#include "VideoSourceForRTSP.h"

#if FFMPEG_DXVA2
typedef int (*PFN_DXVA2_INIT)(AVCodecContext *cc);
extern PFN_DXVA2_INIT _func_dxva2_init;
#endif // 

VideoSourceForRTSP::VideoSourceForRTSP(const imgsrc_format *fmt, const char *url)
	: url_(url)
	, fmt_(*fmt)
{
	quit_ = false;
	start();
}

VideoSourceForRTSP::~VideoSourceForRTSP(void)
{
	quit_ = true;
	sem_.post();
	join();
}

bool VideoSourceForRTSP::wait(int ms)
{
	sem_.wait(ms);
	return quit_;
}

void VideoSourceForRTSP::run()
{
	AVFormatContext *ctx = 0;
	SwsContext *sws = 0;
	int last_w = -1, last_h = -1;
	AVFrame *frame = avcodec_alloc_frame();
	size_t cnt = 0;

	while (!quit_) {
		if (!ctx) {
			AVDictionary *dic = 0;
//			av_dict_set(&dic, "timeout", "3", 0);
			av_dict_set(&dic, "stimeout", "2000000", 0);	// stimeout 为 tcp io 超时，微秒 
			av_dict_set(&dic, "rtsp_transport", "tcp", 1);	// using tcp transport ...
			int rc = avformat_open_input(&ctx, url_.c_str(), 0, &dic);
			av_dict_free(&dic);

			if (rc != 0) {
				ctx = 0;
				fprintf(stderr, "%s: open %s err\n", __FUNCTION__, url_.c_str());
			}
			else {
				fprintf(stderr, "%s: open %s ok\n", __FUNCTION__, url_.c_str());
				avformat_find_stream_info(ctx, 0);
				for (int i = 0; i < ctx->nb_streams; i++) {
					AVCodecContext *codec = ctx->streams[i]->codec;
					if (codec->codec_type == AVMEDIA_TYPE_VIDEO) {
						if (codec->codec_id == CODEC_ID_H264) {
#if FFMPEG_DXVA2
							AVHWAccel *hw_h264 = 0;
							AVHWAccel *hw = av_hwaccel_next(0);
							while (hw) {
								if (hw->pix_fmt == AV_PIX_FMT_DXVA2_VLD && hw->id == CODEC_ID_H264) {
									hw_h264 = hw;
									av_register_hwaccel(hw_h264);
									break;
								}

								hw = av_hwaccel_next(hw);
							}

							if (hw_h264 && _func_dxva2_init) {
								fprintf(stderr, "DEBUG: %s: en, h264_dxva2 supported!\n", __FUNCTION__);
								// codec->hwaccel_context = hw_h264;

								av_log_set_level(AV_LOG_DEBUG);
								_func_dxva2_init(codec);
							}
#endif 
						}

						AVCodec *cc = avcodec_find_decoder(codec->codec_id);
						avcodec_open2(codec,  cc, 0);
					}
				}
			}
		}

		if (!ctx) {
			wait(5000);
			continue;
		}

		AVPacket pkg;
		if (av_read_frame(ctx, &pkg) == 0) {
			AVCodecContext *codec = ctx->streams[pkg.stream_index]->codec;
			if (codec->coder_type == AVMEDIA_TYPE_VIDEO) {
				// 检查是否需要重置 sws
				if (last_w != codec->width && last_h != codec->height) {
					if (codec->width > 0 && codec->height > 0) {
						if (sws) sws_freeContext(sws);

						last_w = codec->width, last_h = codec->height;
						sws = sws_getContext(last_w, last_h, codec->pix_fmt, fmt_.width, fmt_.height, fmt_.fmt, SWS_FAST_BILINEAR, 0, 0, 0);
					}
				}

				bool save = false;
				if (cnt % 2)
					save = true;
				cnt++;
				one_video_frame(sws, codec, &pkg, frame, save);
			}

			av_free_packet(&pkg);
		}
		else {
			fprintf(stderr, "%s: read %s err\n", __FUNCTION__, url_.c_str());
			avformat_close_input(&ctx);
			ctx = 0;
			wait(5000);
			continue;
		}
	}

	if (sws) {
		sws_freeContext(sws);
	}

	if (frame) {
		avcodec_free_frame(&frame);
	}

	if (ctx) {
		avformat_close_input(&ctx);
		ctx = 0;
	}
}

int VideoSourceForRTSP::one_video_frame(SwsContext *sws, AVCodecContext *codec, AVPacket *pkg, AVFrame *frame, bool save)
{
	int got;
	
	int rc = avcodec_decode_video2(codec, frame, &got, pkg);
	if (rc >= 0 && got) {
		if (!sws) return -1;
		if (save) {
			save_frame(sws, frame, pkg->pts/90000.0);	 // 90000Hz ?? 
		}

		return 0;
	}
	else {
		fprintf(stderr, "E");
		return -1;
	}
}

zifImage *VideoSourceForRTSP::next_img()
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

void VideoSourceForRTSP::free_img(zifImage *img)
{
	if (img) {
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
}
