#pragma once

#include <opencv2/opencv.hpp>
#include "../libimagesource/image_source.h"
#include "../libkvconfig/KVConfig.h"

class Source
{
	bool opened_;
	imgsrc_t *src_;
	cv::VideoCapture vc_;
	int width, height;

public:
	explicit Source(KVConfig *cfg, const char *url0)
	{
		opened_ = false;
		src_ = 0;

		const char *url = url0 ? url0 : cfg->get_value("video_source", 0);
		if (url) {
			width = atoi(cfg->get_value("video_width", "960"));
			height = atoi(cfg->get_value("video_height", "540"));

			if (!strncmp("rtsp:", url, 5)) {
				imgsrc_format fmt;
				fmt.width = width, fmt.height = height, fmt.fmt = AV_PIX_FMT_BGR24, fmt.fps = 0;
				src_ = imgsrc_open(url, &fmt);
				opened_ = true;
			}
			else {
				opened_ = vc_.open(url);
			}
		}
	}

	~Source()
	{
		if (src_) {
			imgsrc_close(src_);
		}
		else if (vc_.isOpened()) {
			vc_.release();
		}
	}

	bool is_opened() const
	{
		return opened_;
	}

	int wait() const
	{
		if (src_)
			return 1;
		else
			return 100;
	}

	cv::Mat next()
	{
		if (src_) {
			zifImage *img = imgsrc_next(src_);
			if (img) {
				IplImage *image = zifImage2IplImage(img);
				cv::Mat frame = cv::Mat(image).clone();
				free(image);
				imgsrc_free(src_, img);

				return frame;
			}
			else {
				return cv::Mat();
			}
		}
		else {
			cv::Mat frame;
			vc_ >> frame;
			if (frame.rows > 0) {
				cv::resize(frame, frame, cv::Size(width, height));
			}

			return frame;
		}
	}

	static IplImage *zifImage2IplImage(zifImage *img)
	{
		IplImage *image = (IplImage*)malloc(sizeof(IplImage));
		image->nSize = sizeof(IplImage);
		image->ID = 0;
		image->nChannels = 3;
		image->alphaChannel = 0;
		image->depth = 8;
		image->origin = 0;
		image->dataOrder = 0;
		image->width = img->width;
		image->height = img->height;

		image->roi = 0;
		image->imageId = 0;
		image->tileInfo = 0;

		image->imageSize = img->width * img->height * 3;
		image->widthStep = img->stride[0];
		image->imageData = (char*)img->data[0];

		image->imageDataOrigin = 0;

		return image;
	}
};
