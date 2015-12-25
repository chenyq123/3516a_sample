#if SUPPORT_YUANSDK

#include "VideoSourceForYuan.h"
#include <Objbase.h>
#include <assert.h>

#pragma comment(lib, "qcap.lib")

VideoSourceForYuan::VideoSourceForYuan(const imgsrc_format *fmt, int channel)
	: fmt_(fmt)
	, channel_(channel)
{
	ow_ = fmt->width;
	oh_ = fmt->height;

	last_width_ = -1;
	sws_ = 0;

	evt_ = CreateEvent(0, 0, 0, 0);

	quit_ = false;
	start();
}

VideoSourceForYuan::~VideoSourceForYuan(void)
{
	quit_ = true;
	join();
	CloseHandle(evt_);
}

void VideoSourceForYuan::flush()
{
	ost::MutexLock al(cs_images_);

	while (!fifo_.empty()) {
		zifImage *tmp = fifo_.front();
		fifo_.pop_front();
		free_img(tmp);
	}
}

zifImage *VideoSourceForYuan::get_cached(int width, int height, int fmt)
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
		avpicture_alloc((AVPicture*)img->internal_ptr, fmt_->fmt, width, height);
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

zifImage *VideoSourceForYuan::next_img()
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

void VideoSourceForYuan::free_img(zifImage *img)
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

void VideoSourceForYuan::run()
{
	CoInitialize(0);

	cnt_ = 0;
	lost_ = 2;
	if (lost_ < 0) lost_ = 0;
	
	PVOID dev;
	QRESULT rc = QCAP_CREATE("FH8735 PCI", channel_, 0, &dev);
	if (rc != QCAP_RT_OK) {
		fprintf(stderr, "ERR: %s: QCAP_CREATE for %d err\n", __FUNCTION__, channel_);
		::exit(-1);
	}

	if (channel_ > 8)
		rc = QCAP_SET_VIDEO_INPUT(dev, QCAP_INPUT_TYPE_AUTO);
	else
		rc = QCAP_SET_VIDEO_INPUT(dev, QCAP_INPUT_TYPE_AUTO);

	rc = QCAP_REGISTER_VIDEO_PREVIEW_CALLBACK(dev, _pic_callback, this);

	rc = QCAP_RUN(dev);

	if (rc != QCAP_RT_OK) {
		fprintf(stderr, "ERR: ??? QCAP_RUN err\n");
	}
	else {
		while (!quit_) {
			sleep(100);
		}
	}
	rc = QCAP_STOP(dev);
	rc = QCAP_DESTROY(dev);
}

void VideoSourceForYuan::pic_callback(PVOID dev, double stamp, uint8_t *data, int len)
{
	if (channel_ >= 8) {
		fprintf(stderr, "[%d]: stamp=%.3f, data=%p, len=%d\n", channel_, stamp, data, len);
	}
	
	if (!data) {
		return;
	}
	if (len == 0) {
		return;
	}

	ULONG width, height, framerate;
	BOOL interleaved;
	QRESULT rc = QCAP_GET_VIDEO_CURRENT_INPUT_FORMAT(dev, &width, &height, &interleaved, &framerate);

	cnt_++;
	if ((cnt_ % (lost_+1)) != 0)
		return;

//	if (interleaved) {
//		rc = QCAP_SET_VIDEO_DEINTERLACE(dev, 1);
//	}

	//fprintf(stderr, "[%d]: stamp=%.3f, width=%d, height=%d, framerate=%d, len=%d\n", channel_, stamp, width, height, framerate, len);
	if (width != last_width_ || height != last_height_) {
		last_width_ = width, last_height_ = height;

		if (sws_) sws_freeContext(sws_);
		sws_ = sws_getContext(width, height, PIX_FMT_YUV420P, ow_, oh_, fmt_->fmt, SWS_FAST_BILINEAR, 0, 0, 0);
	}

	// FIXME: 总是使用 NV12，与 YUV420P 的 UV 颠倒
	uint8_t *imgdata[4] = {
		data,
		data + width * height + width * height / 4,
		data + width * height,
		0 };
	int stride[4] = {
		width,
		width / 2,
		width / 2,
		0 };


	assert(sws_);
	zifImage *img = get_cached(ow_, oh_, fmt_->fmt);
	sws_scale(sws_, imgdata, stride, 0, height, img->data, img->stride);

	img->stamp = stamp;
	img->width = ow_;
	img->height = oh_;
	img->fmt_type = fmt_->fmt;

	ost::MutexLock al(cs_images_);

	if (fifo_.size() >= 5) {
		zifImage *tmp = next_img();
		free_img(tmp);
	}

	fifo_.push_back(img);
	SetEvent(evt_);
}

#endif // 