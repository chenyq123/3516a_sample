#include "studentDetectOP.h"
#include "opencv2/opencv.hpp"

#define SHOW

using namespace cv;
using namespace std;

studentDetectOP::studentDetectOP(cv::Size sz, double duration)
	:img_size(sz), up_student_duration(duration)
{
	min_student_area = 100;
	down_up = 0.8;
	fb_speed_ratio = 1/*0.6*/;
	up_speed_horizon_vertical = 0.3;
	bg_model = new cv::BackgroundSubtractorMOG2();
	tvl1 =  cv::createOptFlow_DualTVL1();
	back_start = 0;
	is_debug = false;
	mask_on = false;
}

studentDetectOP::~studentDetectOP()
{
	delete bg_model;
}
void studentDetectOP::bg_filter()
{
	cv::Mat fgmask;
	std::vector<std::vector<cv::Point> > contours;
	(*bg_model)(cur_color, fgmask);
	if (mask_on)
	{
		fgmask &=mask;
	}
	cv::Mat temp;
	if (!pre_gray.empty())
	{
		//imshow("imgT_pre_gray", pre_gray);
		//waitKey(1);

		imshow("imgT_fgmask", fgmask);
		waitKey(1);

		threshold(fgmask, temp, 20, 255, cv::THRESH_BINARY);

		int niters = 3;
		dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);
		erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters*2);
		dilate(temp, temp,cv::Mat(), cv::Point(-1,-1), niters*2);

		//¶àÅªÒ»´Î
		dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);
		erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);

		imshow("imgT_fgmask2", temp);
		waitKey(1);

		//findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		int i=0;
		for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ++it)
		{
			i++;
			if (contourArea(*it) > min_student_area)
			{
				foregrounds.push_back(boundingRect(*it));
				if (is_debug)
				{
					rectangle(cur_color, boundingRect(*it), cv::Scalar(255, 0, 0), 2);
				}
			}
			if(i>1){
				i=0;
			}
		}
	}
}

void studentDetectOP::student_down()
{

	std::vector<upStudent>::iterator it = up_students.begin();
	while( it != up_students.end())
	{
		if (cur_stamp -  it->stamp > up_student_duration)
		{
			it = up_students.erase(it);
		}
		else
		{
			bool student_up = true;

			cv::Mat flow, yflow;
			cv::Rect rc = cv::Rect(it->position.x - it->position.width * 0.2, it->position.y - it->position.height * 0.2,
				it->position.width * 1.4, it->position.height * 1.4) & cv::Rect(0, 0, img_size.width, img_size.height);
			cv::calcOpticalFlowFarneback(pre_gray(rc), cur_gray(rc), flow, 0.5, 3, 15, 3, 5, 1.2, 0);
			studentReference sr = get_student_ref(it->position);
			yflow.create(flow.rows, flow.cols, CV_32FC1);
			int ch[] = {1, 0};
			mixChannels(&flow, 1,  &yflow, 1, ch, 1);
			cv::Mat result;
			double back_ratio = 1;
			if (sr.base_point.y < back_start)
			{
				back_ratio = (double)(sr.base_point.y - back_start) / (0 - back_start) * (end_back_down_up - start_back_down_up)
					+ start_back_down_up;
			}
			threshold(yflow, result, back_ratio * sr.speed * down_up, 255, cv::THRESH_BINARY);
			result.convertTo(result, CV_8UC1);
			int niters = 6;
			dilate(result, result, cv::Mat(), cv::Point(-1,-1), niters);
			erode(result, result, cv::Mat(), cv::Point(-1,-1), niters*2);
			dilate(result, result,cv::Mat(), cv::Point(-1,-1), niters*2);
			std::vector<std::vector<cv::Point> > contours;

			cv::findContours(result, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			for(std::vector<std::vector<cv::Point> >:: iterator it_contour = contours.begin(); it_contour != contours.end(); ++it_contour)
			{
				if (contourArea(*it_contour) > back_ratio* 0.8 * sr.area)
				{
					student_up = false;
					it = up_students.erase(it);
					break;
				}
			}

			if (student_up)
			{
				++it;
			}
		}
	}
}

void studentDetectOP::offb_filter()
{
	int i=0;
	for (std::vector<cv::Rect>::iterator it = foregrounds.begin(); !pre_gray.empty()&&it != foregrounds.end(); ++it)
	{
		cv::Mat flow;
		cv::calcOpticalFlowFarneback(pre_gray(*it), cur_gray(*it), flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		studentReference sr = get_student_ref(*it);
		cv::Mat yflow, xflow;
		yflow.create(flow.rows, flow.cols, CV_32FC1);
		xflow.create(flow.rows, flow.cols, CV_32FC1);
		cv::Mat out[] = {xflow, yflow};
		int ch[] = {0, 0, 1, 1};
		mixChannels(&flow, 1,  out, 2, ch, 2);
		cv::Mat result;
		threshold(yflow, result, -fb_speed_ratio*sr.speed, 255, cv::THRESH_BINARY_INV);
		result.convertTo(result, CV_8UC1);

		i++;
		if(i==1){
		imshow("imgT_OpticalFlow", result);
		waitKey(1);}

		int niters = 6;
		dilate(result, result, cv::Mat(), cv::Point(-1,-1), niters);
		erode(result, result, cv::Mat(), cv::Point(-1,-1), niters*2);
		dilate(result, result,cv::Mat(), cv::Point(-1,-1), niters*2);

		if(i==1){
		imshow("imgT_OpticalFlow2", result);
		waitKey(1);}

		cv::Mat result2 = result.clone();
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(result, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		for (int i = 0; i < contours.size(); i++)
		{
			cv::Rect br = boundingRect(contours[i]) ;
			cv::Rect rc = br +  it->tl();
			studentReference sr2 = get_student_ref(rc);
			cv::Scalar s = cv::mean(xflow(br), result2(br));
			if (contourArea(contours[i]) > sr2.area && fabs(s[0]) < 0.5 * sr2.speed)
			{
				up_student_candidates.push_back(rc);
			}
		}
	}
}

void studentDetectOP::oftvl1_filter()
{
	std::vector<cv::Rect>::iterator it = up_student_candidates.begin();
	//if(up_student_candidates.size()>1){
	//	it_end = up_student_candidates.end();
	//}

	while(it != up_student_candidates.end())
	{
		if(up_students.size()==0){
			upStudent us;
			us.position = *it;
			us.stamp = cur_stamp;
			up_students.push_back(us);
		}
		else{
			std::vector<upStudent>::iterator it1 = up_students.begin();
			while(it1 != up_students.end())
			{
			cv::Rect intersect = *it & it1->position;
			if (intersect.area() > 0 ||
				(cur_stamp - it1->stamp < 1 &&
				(it->x + it->width / 2 > it1->position.x && it->x + it->width/2 < it1->position.br().x) &&
				it->br().y < it1->position.y &&
				it->br().y - it1->position.y < 0.2 * it->br().y))
			{
				*it |= it1->position;
				up_students.erase(it1);

				upStudent us;
				us.position = *it;
				us.stamp = cur_stamp;
				up_students.push_back(us);

				break;
			}

			++it1;
			}
		}
		++it;
	}
}

void studentDetectOP::process(const cv::Mat& img_color, double stamp)
{
	cur_stamp = stamp;
	resize(img_color, cur_color, img_size);
	cv::cvtColor(cur_color, cur_gray, CV_BGR2GRAY);
	bg_filter();
	student_down();
	offb_filter();
	oftvl1_filter();
	foregrounds.clear();
	up_student_candidates.clear();
	std::swap(cur_gray, pre_gray);
	for (int i = 0; i < up_students.size(); i++)
	{
		cv::rectangle(cur_color, up_students[i].position, cv::Scalar(0, 0, 255), 2);
	}
	//imshow("cur_color", cur_color);
}

void studentDetectOP::add_student_ref(const studentReference &student_ref)
{
	double ratio_x = (double) img_size.width / student_ref.size_.width;
	double ratio_y = (double) img_size.height / student_ref.size_.height;
	cv::Rect position(student_ref.position.x * ratio_x, student_ref.position.y * ratio_y,
		student_ref.position.width * ratio_x, student_ref.position.height * ratio_y);

	student_refs.push_back(studentReference(position, student_ref.speed * ratio_y, student_ref.area * ratio_x * ratio_y, img_size));
}

void studentDetectOP::set_back_down_up(int start, double start_ratio, double end_ratio)
{
	back_start = start;
	start_back_down_up = start_ratio;
	end_back_down_up = end_ratio;
}

void studentDetectOP::set_debug(bool on_off)
{
	is_debug = on_off;
}
void studentDetectOP::set_mask(const cv::Mat &mask_c)
{
	mask_on = true;
	mask = mask_c;
}

#define UNKNOWN_FLOW_THRESH 1e9

void studentDetectOP::makecolorwheel(std::vector<cv::Scalar> &colorwheel)
{
    int RY = 15;
    int YG = 6;
    int GC = 4;
    int CB = 11;
    int BM = 13;
    int MR = 6;

    int i;

    for (i = 0; i < RY; i++) colorwheel.push_back(Scalar(255,       255*i/RY,     0));
    for (i = 0; i < YG; i++) colorwheel.push_back(Scalar(255-255*i/YG, 255,       0));
    for (i = 0; i < GC; i++) colorwheel.push_back(Scalar(0,         255,      255*i/GC));
    for (i = 0; i < CB; i++) colorwheel.push_back(Scalar(0,         255-255*i/CB, 255));
    for (i = 0; i < BM; i++) colorwheel.push_back(Scalar(255*i/BM,      0,        255));
    for (i = 0; i < MR; i++) colorwheel.push_back(Scalar(255,       0,        255-255*i/MR));
}

void studentDetectOP::motionToColor(cv::Mat flow, cv::Mat &color)
{
    if (color.empty())
        color.create(flow.rows, flow.cols, CV_8UC3);

    static std::vector<cv::Scalar> colorwheel; //Scalar r,g,b
    if (colorwheel.empty())
        makecolorwheel(colorwheel);

    // determine motion range:
    float maxrad = -1;

    // Find max flow to normalize fx and fy
    for (int i= 0; i < flow.rows; ++i)
    {
        for (int j = 0; j < flow.cols; ++j)
        {
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);
            float fx = flow_at_point[0];
            float fy = flow_at_point[1];
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
                continue;
            float rad = sqrt(fx * fx + fy * fy);
            maxrad = maxrad > rad ? maxrad : rad;
        }
    }

    for (int i= 0; i < flow.rows; ++i)
    {
        for (int j = 0; j < flow.cols; ++j)
        {
            uchar *data = color.data + color.step[0] * i + color.step[1] * j;
            Vec2f flow_at_point = flow.at<Vec2f>(i, j);

            float fx = flow_at_point[0] / maxrad;
            float fy = flow_at_point[1] / maxrad;
            if ((fabs(fx) >  UNKNOWN_FLOW_THRESH) || (fabs(fy) >  UNKNOWN_FLOW_THRESH))
            {
                data[0] = data[1] = data[2] = 0;
                continue;
            }
            float rad = sqrt(fx * fx + fy * fy);

            float angle = atan2(-fy, -fx) / CV_PI;
            float fk = (angle + 1.0) / 2.0 * (colorwheel.size()-1);
            int k0 = (int)fk;
            int k1 = (k0 + 1) % colorwheel.size();
            float f = fk - k0;
            //f = 0; // uncomment to see original color wheel

            for (int b = 0; b < 3; b++)
            {
                float col0 = colorwheel[k0][b] / 255.0;
                float col1 = colorwheel[k1][b] / 255.0;
                float col = (1 - f) * col0 + f * col1;
                if (rad <= 1)
                    col = 1 - rad * (1 - col); // increase saturation with radius
                else
                    col *= .75; // out of range
                data[2 - b] = (int)(255.0 * col);
            }
        }
    }

}


studentReference studentDetectOP::get_student_ref(cv::Rect position)
{
	cv::Point base_point = studentReference::get_base_point(position, img_size.width / 2);
	double distance_square_min = img_size.width * img_size.width + img_size.height * img_size.height;
	int index = -1;
	for (int i = 0; i < student_refs.size(); i++)
	{
		double distance_square = (base_point.x - student_refs[i].base_point.x) * (base_point.x - student_refs[i].base_point.x)
			+ (base_point.y - student_refs[i].base_point.y) * (base_point.y - student_refs[i].base_point.y);
		if (distance_square < distance_square_min)
		{
			distance_square_min = distance_square;
			index = i;
		}
	}
	CV_Assert(index >= 0);

	return student_refs[index];
}
void studentDetectOP::clear(void)
{
	pre_gray.release();
	delete bg_model;
	bg_model = new cv::BackgroundSubtractorMOG2();
	up_students.clear();
}

studentReference::studentReference(cv::Rect position_c, double speed_c, double area_c, cv::Size size_c) : position(position_c),
	speed(speed_c), area(area_c), size_(size_c)
{
	base_point = get_base_point(position, size_.width / 2);
}

cv::Point studentReference::get_base_point(cv::Rect rc, int mid_x)
{
	int bottom_x = rc.x + rc.width/2;
	if (bottom_x > mid_x)
	{
		bottom_x = 2 * mid_x - bottom_x;
	}
	CV_Assert(bottom_x >= 0);
	return cv::Point(bottom_x, rc.y + rc.height/2);
}

