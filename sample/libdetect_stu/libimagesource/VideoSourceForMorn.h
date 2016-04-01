#if SUPPORT_MORN

#include <cc++/thread.h>
#include "../libkvconfig/KVConfig.h"
#include "image_source.h"
#include <deque>

extern "C" {
#	include <libavcodec/avcodec.h>
#	include <libswscale/swscale.h>
}

#include "winsock2.h"
#include <HHAVDefine.h>
#include <HHNetInterface.h>
#include <HH5KDecoder.h>

class VideoSourceForMorn : ost::Thread
{
	HANDLE hOpenChannel_;
	std::string url_;
	bool quit_;

	SwsContext *sws_;
	int width_, height_;
	AVCodecContext *dec_;
	AVFrame *frame_;

	typedef std::deque<zifImage*> IMAGES;
	IMAGES fifo_, cached_;
	ost::Mutex cs_images_;
	ost::Mutex cs_dec_;

	HANDLE evt_;
public:
	VideoSourceForMorn::VideoSourceForMorn(const char *url, const imgsrc_format *fmt);
	~VideoSourceForMorn(void);

	zifImage *next_img();
	void free_img(zifImage *img);
	void flush();

private:
	void run();

	int ChannelStreamCallback_(HANDLE hOpenChannel, void *pSteamData, DWORD dwClientID, void *pContext, ENCODE_VIDEO_TYPE encodeVideoType, HHAV_INFO *pAVInfo);
	static int  _ChannelStreamCallback(HANDLE hOpenChannel, void *pStreamData, DWORD dwClientID, void *pContext, ENCODE_VIDEO_TYPE encodeVideoType, HHAV_INFO *pAVInfo)
	{
		return ((VideoSourceForMorn*)pContext)->ChannelStreamCallback_(hOpenChannel,
			pStreamData,
			dwClientID,
			pContext,
			encodeVideoType,
			pAVInfo
			);
	}

	void save_video(uint8_t *data, int len, double stamp);
	int prepare_dec_sws(const EXT_FRAME_VIDEO &vp);
	void save_frame(SwsContext *sws, AVFrame *frame, double stamp);
	zifImage *get_cached(int width, int height, int fmt);

};

#endif //
