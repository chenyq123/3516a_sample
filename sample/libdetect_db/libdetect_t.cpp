#include "libdetect_t.h"
//#include"blackboard_detect.h"
//#include "../libimagesource/image_source.h"
//#include <string>
//#include "hog.h"
//#include <cstdio>

/*
struct det_t
{
	KVConfig *cfg_;
	BlackboardDetecting *bd_detect_;
	IplImage *masked_;
	bool t_m;
	bool b_m;
	std::string result_str;	// FIXME: 希望足够了;
};
*/


det_t *det_open(const char *cfg_name)//初始化
{
	det_t *ctx = new det_t;
	ctx->cfg_ = new KVConfig(cfg_name);

	ctx->t_m = false;
	ctx->b_m = false;

	//const char *method = ctx->cfg_->get_value("BLACKBOARD_OR_TEACHER", "teacher");
	//fprintf(stderr, "======================> method=%s\n", method);

	 if (strcmp(cfg_name, "bd_detect_trace.config") == 0)
	{
		ctx->b_m = true;
		ctx->bd_detect_ = new BlackboardDetecting(ctx->cfg_);
	}

	return ctx;
}


void det_close(det_t *ctx)
{
	delete ctx->cfg_;

	if(ctx->b_m) { delete ctx->bd_detect_; }

	delete ctx;
}



//将vector向量转换为json字符串格式 ;
void vector_to_json(std::vector < Rect > r, char *buf, int area)
{
	int offset = 0;
	offset = sprintf(buf, "{\"stamp\":%d,", time(0));
	offset += sprintf(buf + offset, "\"area\":%d,",area);
	offset += sprintf(buf + offset, "\"rect\":[");

	for (int i = 0; i < r.size(); i++)
	{
		Rect t = r[i];
		if (i == 0)
			offset +=
			    sprintf(buf + offset,
				    "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
				    t.x, t.y, t.width, t.height);
		else
			offset +=
			    sprintf(buf + offset,
				    ",{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
				    t.x, t.y, t.width, t.height);
	}

	strcat(buf, "]}");

}


#define BUFSIZE 4096

char *det_detect(det_t * ctx, int height,int width,unsigned char *imgdata)//imgdata  YUV
{
	char *str = (char*)alloca(BUFSIZE);
	bool isrect = false;
	std::vector < Rect > r;	//目标框;
	vector < cv::Rect > first_r;//初始蓝框;
	////处理一下 图像数据  彩色图像

	Mat Img = cv::Mat(height, width, CV_8UC3, imgdata);

	//算法逻辑
	blur(Img,Img,Size(3,3));


	//***************************板书探测****************************
	if(ctx->b_m)
	{
		//做掩码的原始图像;
		Mat masked_img = Mat(Img,ctx->bd_detect_->masked_rect);
		ctx->bd_detect_->do_mask(masked_img);

		//直接获取标定区外接矩形图像;
		//ctx->bd_detect_->masked_rect &= Rect(0,0,img_t.cols, img_t.rows);
		isrect = ctx->bd_detect_->one_frame_bd(/*(IplImage *)&*/masked_img, r);
		for (int i = 0; i<r.size( ); i++)
		{
			cv::Rect box = ctx->bd_detect_->masked_rect;
			r[i].x = r[i].x+box.x;
			r[i].y = r[i].y+box.y;
			r[i] &= cv::Rect(0,0,Img.cols,Img.rows);
		}

		//***************调试****************
		if (atoi(ctx->cfg_->get_value("debug", "0")) > 0)
		{
			for (int i = 0; i < r.size(); i++)
			{
				rectangle(Img, r[i], Scalar(0, 0, 255), 2);
			}
			//imshow("rawimage", Img);
			//imshow("masked_img", masked_img);
			//waitKey(1);
		}

		if (isrect)
		{
			bool cal1 = false; bool cal2 = false;
			for(int i = 0; i < r.size( ); i++)
			{
				cv::Point p = Point((r[i].x + r[i].width/2), (r[i].y + r[i].height/2));
				for(int j = 0; j < ctx->bd_detect_->masked_rect_vec.size( ); j++)
				{
					cv::Rect rect = ctx->bd_detect_->masked_rect_vec[j];
					if(p.x >= rect.x && p.x <= (rect.x+rect.width))
					{
						if(j == 0) cal1 = true;
						else if(j == 1) cal2 = true;
					}
				}
			}
			int area = 0;
			if(cal1 == true && cal2 == true) area = 0;
			else if(cal1 == true) area = 1;
			else if(cal2 == true) area = 2;
			else area = 0;
			vector_to_json(r, str, area);
		}
		else
		{
			snprintf(str, BUFSIZE, "{\"stamp\": %d,\"area\":0,\"rect\": [ ] }", time(0));
		}

	}

	ctx->result_str = str;
	return (char*)ctx->result_str.c_str( );
}

