#include "camshift.h"
#include <assert.h>
// 矩形 r1 是否在 f2 范围中
#define IN_RECT(r1, r2) ((r1).x >= (r2).x && (r1).y >= (r2).y && (r1).x+(r1).width <= (r2).x+(r2).width && (r1).y+(r1).height <= (r2).y+(r2).height)
tracker_camshift::tracker_camshift(const char* yaml_file)  
{
	vmax = 140;
	//vmax = 255;
	vmin = 60;
	//vmin = 0;
	hranges[0] = 0;
	hranges[1] = 180;
	sranges[0] = 0;
	sranges[1] = 255;
	const float* ranges[] = {hranges, sranges};
	hist_size[0] = 30;//值越大越容易跟丢
	hist_size[1] = 32;//值越大越容易跟丢
	set_hist(yaml_file);
}

cv::Mat tracker_camshift::get_backproj(const cv::Mat &image)
{
	cv::Mat hsv;
	cvtColor(image, hsv, CV_BGR2HSV);
	cv::Mat mask;
	inRange(hsv, cv::Scalar(0, 0, vmin), cv::Scalar(180, 255, vmax), mask);
	int ch[] = {0, 1};
	const float* ranges[] = {hranges, sranges};
	cv::Mat backproj;
	calcBackProject(&hsv, 1, ch, hist, backproj, ranges);
	return backproj&mask;
}

cv::Rect tracker_camshift::process(const cv::Mat &image, bool is_erode)
{

	cv::Mat backproj = get_backproj(image);
	if (is_erode)
	{
		cv::Mat kernal(3, 3, CV_8UC1, cv::Scalar(1));
		cv::Mat dest;
		cv::erode(backproj, dest, kernal);	
		backproj = dest;
	}
	//threshold(backproj, backproj,vthreshold, 255, cv::THRESH_BINARY);
	if (track_window.height <= 0 || track_window.width <= 0) {
		fprintf(stderr, "???? %s:\n", __FUNCTION__);
		//__asm int 3;
	}
	CamShift(backproj, track_window, cv::TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

	if (track_window.width > image.cols || track_window.height > image.rows || track_window.width <= 0 || track_window.height <= 0) {
		track_window = cv::Rect();
	}
	return track_window;
}

void tracker_camshift::set_track_window(const cv::Rect &position)
{
	track_window = position;
}

void tracker_camshift::set_hist(cv::Mat  &image, const cv::Rect &rect, int hsize, int ssize, int threshold_value)
{
	cv::Mat hsv;
	cvtColor(image, hsv, CV_BGR2HSV);
	cv::Mat mask;
	inRange(hsv, cv::Scalar(0, 0, vmin), cv::Scalar(180, 255, vmax), mask);
	hist_size[0] = hsize;
	hist_size[1] = ssize;
	vthreshold = threshold_value;
    int ch[] = {0, 1};
	const float* ranges[] = {hranges, sranges};
	cv::Rect rec = rect & cv::Rect(0, 0, hsv.cols, hsv.rows);
	cv::Mat roi(hsv, rec), roimask(mask, rec);
	cv::calcHist(&roi, 1, ch, roimask, hist, 2, hist_size, ranges);
	normalize(hist, hist, 0, 255, CV_MINMAX);
}

void tracker_camshift::set_hist(const char *file_name)
{
	cv::FileStorage fs(file_name, cv::FileStorage::READ);
	if (!fs.isOpened())
		throw std::runtime_error("Can't find the hist file\n");
	fs["hist"] >> hist;
 	hist_size[0] = (int) fs["hsize"];
 	hist_size[1] = (int) fs["ssize"];
	vthreshold = (int)fs["threshold"];
}

// 判断是否为人脸
bool tracker_camshift::is_face(cv::Rect &face, cv::Rect upper_body)
{
	/** FIXME:
			face 为 cam shift 处理得到的人脸肤色匹配位置，upper_body 为头肩识别到的位置，
			这里只要简单的判断，如果 face 在 upper_body 的“上中”部分即可

		*/
	bool found = false;
	// 在 upperbody 中画一个可能的人脸位置框，要求 face 必须完整在这个框内，即可
	cv::Rect face_want(upper_body.x + upper_body.width / 5, upper_body.y + 2, upper_body.width * 3 / 5, upper_body.height * 4 / 5);
	found = IN_RECT(face, face_want);
	return found;
}


/** 第一次根据人脸设置直方图，应该更严格！！！！
	face 为人类，首先缩小一点，如果还足够大，则修正直方图
	*/
bool tracker_camshift::cam_shift_init_once(cv::Rect &face,cv::Mat image)
{
#define SCALE_X 0.8
#define SCALE_Y 0.6
		int left = face.x + (1.0 - SCALE_X)/2.0 * face.width;
		int top = face.y + (1.0 - SCALE_Y)/2.0 * face.height;
		int width = 1.0 * face.width * SCALE_X;
		int height = 1.0 * face.height * SCALE_Y;
		if (width * height < 36) {
			//ctx->debug("%s: rc.width=%d, rc.height=%d\n", __func__, face.width, face.height);
			return false;
		}
		face.x = left, face.y = top, face.width = width, face.height = height;
		
		cv::Mat bkimg(image);
		//set_hist(bkimg, face, 60, 128, 36);
		set_hist(bkimg, face, 30, 100, 36);

		//draw_rect(ctx, face, CV_RGB(255, 255, 255));

		//ctx->debug("%s: en, init face hist: size=%d-%d\n", __func__, face.width, face.height);

		return true;
}


tracker_camshift::~tracker_camshift()
{
}




