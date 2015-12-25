#ifdef SUPPORT_IPCAM
#include <cc++/thread.h>
#include <NetSDKDLL.h>
#include <DllPlayer.h>
#include <deque>
#include "image_source.h"

extern "C" {
#	include <libavcodec/avcodec.h>
#	include <libswscale/swscale.h>
}

class VideoSourceFromIPCam : ost::Thread
{
	std::string url_;

	SwsContext *sws_;
	AVCodecContext *dec_;
	AVFrame *frame_;

	struct Message
	{
		int id;
		void *data;
		enumNetSatateEvent ev;
		int media_type;

		Message () : id(0) {} // 用于结束 
		Message (enumNetSatateEvent ev, void *ptr) : id(1), ev(ev), data(ptr) {}
		Message (int media_type, void *ptr) : id(2), media_type(media_type), data(ptr) {}
	};

	typedef std::deque<Message> MESSAGES;
	MESSAGES msgs_;
	ost::Mutex cs_msg_;
	ost::Semaphore sem_msg_;
	ost::Mutex cs_dec_;

	typedef std::deque<zifImage*> IMAGES;
	IMAGES fifo_, cached_;
	ost::Mutex cs_images_;

	HANDLE evt_;
	int ow_, oh_;	// 要求的输出的大小

public:
	VideoSourceFromIPCam(const char *url, const imgsrc_format *fmt);
	~VideoSourceFromIPCam(void);

	/** 获取下一帧图像，应尽快，否则会溢出丢失 */
	zifImage *next_img();
	void free_img(zifImage *img);
	void flush();

private:
	void run();

	static LONG CALLBACK _cb_data(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, LPFRAME_EXTDATA  pExtData)
	{
		if (pExtData && pExtData->pUserData) {
			((VideoSourceFromIPCam*)pExtData->pUserData)->cb_data(lRealHandle, dwDataType, pBuffer, dwBufSize, pExtData);
		}
		return 0;
	}
	void cb_data(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, LPFRAME_EXTDATA  pExtData);

	static LONG CALLBACK _cb_status(LONG lUser, LONG nStateCode, char *pResponse, void *pUser)
	{
		((VideoSourceFromIPCam*)pUser)->cb_status(lUser, nStateCode, pResponse);
		return 0;
	}
	void cb_status(LONG lUser, LONG nStateCode, char *pReason);

	void save_video(uint8_t *data, int len, double stamp);	// 写入流
	int prepare_dec_sws(const VIDEO_PARAM &vp);

	bool next_message(Message &msg)
	{
		bool got = false;

retry:
		sem_msg_.wait();
		cs_msg_.enter();
		if (!msgs_.empty()) {
			msg = msgs_.front();
			msgs_.pop_front();
			got = true;
		}
		cs_msg_.leave();

		if (!got) goto retry;

		if (msg.id == 0)
			return false;
		else
			return true;
	}

	void post_message(const Message &msg)
	{
		ost::MutexLock al(cs_msg_);
		msgs_.push_back(msg);
		sem_msg_.post();
	}

	void save_frame(SwsContext *sws, AVFrame *frame, double stamp);
	zifImage *get_cached(int width, int height, int fmt);
};

#endif // ipcam