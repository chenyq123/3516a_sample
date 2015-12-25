#include "VideoSourceFFmpeg.h"


VideoSourceFFmpeg::VideoSourceFFmpeg(const imgsrc_format *fmt, const char *url)
	: VideoSourceBase(fmt, url)
{
	fc_ = 0;
	sw_ = 0;
	frame_ = 0;
	last_h_ = -1;
	last_w_ = -1;
}

VideoSourceFFmpeg::~VideoSourceFFmpeg(void)
{
}

int VideoSourceFFmpeg::open_src()
{
	int rc = avformat_open_input(&fc_, url_.c_str(), 0, 0);
	if (rc != 0) {
		fprintf(stderr, "ERR: [imgsrc]: %s: can't open %s\n", __FUNCTION__, url_.c_str());
		return -1;
	}

	if (avformat_find_stream_info(fc_, 0) < 0) {
		fprintf(stderr, "ERR: [imgsrc]: %s: can't fine stream info for %s\n", __FUNCTION__, url_.c_str());
		avformat_close_input(&fc_);
		fc_ = 0;
		return -2;
	}
	
	bool found_video = false;

	for (int i = 0; i < fc_->nb_streams; i++) {
		AVCodecContext *codec = fc_->streams[i]->codec;
		if (codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			AVCodec *cc = avcodec_find_decoder(codec->codec_id);
			if (cc) {
				avcodec_open2(codec, cc, 0);
				found_video = true;
				break;
			}
		}
	}

	if (!found_video) {
		fprintf(stderr, "ERR: [imgsrc]: %s: can't find video stream in %s\n", __FUNCTION__, url_.c_str());
		avformat_close_input(&fc_);
		fc_ = 0;
		return -3;
	}

	frame_ = av_frame_alloc();

	return 0;
}

void VideoSourceFFmpeg::close_src()
{
	if (fc_) {
		avformat_close_input(&fc_);
		fc_ = 0;
	}

	if (sw_) {
		sws_freeContext(sw_);
		sw_ = 0;
	}

	if (frame_) {
		av_frame_free(&frame_);
		frame_ = 0;
	}

	last_w_ = last_h_ = -1;
}

int VideoSourceFFmpeg::get_src(zifImage *img)
{
	assert(fc_);

	AVPacket pkg;
	bool got_video = false;

	while (!got_video) {
		if (av_read_frame(fc_, &pkg) < 0) {
			fprintf(stderr, "ERR: [imgsrc] %s: read frame err for %s\n", __FUNCTION__, url_.c_str());
			return -1;
		}

		AVCodecContext *cc = fc_->streams[pkg.stream_index]->codec;
		if (cc->codec_type == AVMEDIA_TYPE_VIDEO) {
			int got;
			if (avcodec_decode_video2(cc, frame_, &got, &pkg) < 0) {
				fprintf(stderr, "WARNING: [imgsrc] %s: decode video frame err for %s\n", __FUNCTION__, url_.c_str());
			}

			if (got) {
				if (chk_sws(cc)) {
					got_video = save(img);
					img->stamp = pkg.pts * 1.0 / cc->ticks_per_frame * cc->time_base.num / cc->time_base.den;
				}
			}
		}

		av_free_packet(&pkg);
	}

	return 1;
}

bool VideoSourceFFmpeg::chk_sws(AVCodecContext *cc)
{
	if (cc->width != last_w_ || cc->height != last_h_ || !sw_) {
		if (sw_) {
			sws_freeContext(sw_);
		}

		sw_ = sws_getContext(cc->width, cc->height, cc->pix_fmt, fmt_.width, fmt_.height, fmt_.fmt, SWS_FAST_BILINEAR, 0, 0, 0);
		if (!sw_) {
			fprintf(stderr, "ERR: [imgsrc]: %s: can't create sws: src=[%d-%d, %d], dst=[%d-%d, %d] for %s\n", __FUNCTION__,
				cc->width, cc->height, cc->pix_fmt, fmt_.width, fmt_.height, fmt_.fmt, url_.c_str());
		}
		else {
			last_w_ = cc->width, last_h_ = cc->height;
		}
	}

	return sw_ != 0;
}

bool VideoSourceFFmpeg::save(zifImage *img)
{
	assert(sw_);
	assert(frame_);
	assert(img);

	int rc = sws_scale(sw_, frame_->data, frame_->linesize, 0, last_h_, img->data, img->stride);
	return true;
}

void VideoSourceFFmpeg::pause_src()
{
	if (fc_) {
		av_read_pause(fc_);
	}
}

void VideoSourceFFmpeg::resume_src()
{
	if (fc_) {
		av_read_play(fc_);
	}
}
