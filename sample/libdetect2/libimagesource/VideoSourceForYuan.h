#pragma once

#ifdef SUPPORT_YUANSDK

#include "image_source.h"
#include <cc++/thread.h>
#include <yuan/QCAP.H>
#include "../libkvconfig/KVConfig.h"
#include <deque>

extern "C" {
#	include <libavcodec/avcodec.h>
#	include <libswscale/swscale.h>
}

class VideoSourceForYuan : ost::Thread
{
	const imgsrc_format *fmt_;
	int channel_;
	bool quit_;

	SwsContext *sws_;
	int last_width_, last_height_;

	typedef std::deque<zifImage*> IMAGES;
	IMAGES fifo_, cached_;
	ost::Mutex cs_images_;

	HANDLE evt_;
	int ow_, oh_;	// 要求的输出的大小

	size_t cnt_;	// 用于支持主动丢帧
	int lost_;

public:
	VideoSourceForYuan(const imgsrc_format *fmt, int channel);
	~VideoSourceForYuan(void);

	/** 获取下一帧图像，应尽快，否则会溢出丢失 */
	zifImage *next_img();
	void free_img(zifImage *img);
	void flush();

private:
	void run();
	static QRETURN _pic_callback(PVOID pDevice, double dSampleTime, BYTE* pFrameBuffer, ULONG nFrameBufferLen, PVOID pUserData)
	{
		((VideoSourceForYuan*)pUserData)->pic_callback(pDevice, dSampleTime, pFrameBuffer, nFrameBufferLen);
		return QRETURN::QCAP_RT_OK;
	}
	void pic_callback(PVOID dev, double stamp, uint8_t *data, int len);
	zifImage *get_cached(int width, int height, int fmt);
};
#endif 
