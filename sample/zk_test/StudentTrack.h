#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "ImageArry.h"
#include "FileTan.h"
#include <time.h>

//#define WIDTH 320
//#define HEIGHT 180

unsigned long GetTickCount();

#define CONDIDATECOUNT 10

struct upStudentTarget
{
	long tickcount;//创建时的时间戳
	cv::Rect position;//目标区域

	int tag;//0 站起区域有效 1 时间到而取消 2 有连续坐下光流取消  3 有光流面积取消  4 有连续移动的光流离开该区域
	int deletecount;//删除倒数，只有在tag>0时才有效，为0时就删除，不为0时每次处理减一

	cv::Rect pos0,pos1,pos2;//产生该区域 起作用的连续三幁图像的区域，0 最新 1 前一幁 2 前2幁
	int showposcount;

	cv::Mat pregray2,pregray,curgray;//站起来之前的灰度图

};


class CStudentTrack
{
public:
	CStudentTrack(void);
	virtual ~CStudentTrack(void);

	void setParam(int yfirst,int standupareafirst,int widthfirst,int ylast,int standuparealast,int widthlast);

	// 主过程
	void process(const cv::Mat& img_color);

	void setduration(int sec);

	void setdebug(bool b);

	void readconfig(char *cfg_name);

	void start();

	void set_maskpoint(cv::Point p);

	std::vector<upStudentTarget> up_students;//站起来的区域

	long tt,t1,t2,t3,t4,t5,t6,t7,t8;//记录消耗时间
	long flowcount;

	std::vector<upStudentTarget> up_studentsTmp;//临时用于展示临时变量
	void showStudenttmp();
	int ishowtmp;
	int height_input,width_input;
private:
	int y_firstline,y_lastline;
	int width_first,width_last;
	int standuparea_first,standuparea_last;

	int frameNum;
	int min_student_area;//寻找光流的区域 最小面积10*10
	int up_student_duration;//起立的学生 时长 内，不坐下来 就当做 已经坐下来，ms为单位

	cv::Rect MaxRect,MinRect;
	float splitvalue;

	cv::Mat pre_gray;
	cv::Mat cur_gray;
	cv::Mat cur_color;
	cv::Mat flowimg;//叠加光流的彩色图像
	cv::Mat flowupbw,flowdownbw;//上下光流二值化图

	cv::Mat flowup,flowdown;
	cv::Mat flowleft,flowright;

	cv::Mat fgmask,imgdiff;

	cv::Mat imgdiffforflow;

	cv::Mat flowmove;

	CImageArry imageArry;

	CFileTan fileTan;

	bool is_debug;
	void showDebug();

	//屏保区域
	std::vector<cv::Point> mask_points;


	//光流法通道
	cv::Ptr<cv::DenseOpticalFlow> tvl1;

	//背景建模
	cv::BackgroundSubtractorMOG *bg_model;

	cv::Mat modelBG;

	//学生站起部分
	int ican_head;
	std::vector<cv::Rect> up_student_candidates[CONDIDATECOUNT];
	int getNewCondidate();
	int getPreCondidate(int prenum);//0 当前最新的，1 前一个，2 前2个，...


	void checkupstudent(cv::Mat mat);//寻找光流的区间，连续三幁 有往下的光流，且光流数量在也满足最低条件

	std::vector<upStudentTarget> up_studentsSave;//站起来的区域 保留，用于判断该区域 不允许1秒内有个上升的光流，排除 学生做下来后，往后仰，会产生 上升的光流
	bool checkupStudentSave(cv::Rect rc);//判断站起的待选区域，是否属于刚消失的区域，用于排除学生坐下向后仰的情况

	bool checkupOnlyUpflow(cv::Rect rc);//判断该区域是否向上的光流最多
	bool checkupfg(cv::Rect p0,cv::Rect p1,cv::Rect p2,cv::Rect p);//通过前景来判断，要改变了 才算真的 有人站起来

	//bool check


	//学生坐下部分
	int ican_head_down;
	std::vector<cv::Rect> down_student_candidates[CONDIDATECOUNT];
	int getNewCondidatedown();
	int getPreCondidatedown(int prenum);//0 当前最新的，1 前一个，2 前2个，...
	void checkDownStudentByFlow(cv::Mat mat);//寻找光流的区间，连续三幁 有往下的光流，且光流数量在也满足最低条件，则 减去站立区间
	void checkDownStudentByArea();//根据最近的三幁图像的光流面积，来判断是否减去站立区间
	void checkDownStudentNormal();//根据时间是否到来而删除

	////学生走开
	std::vector<cv::Rect> move_student[CONDIDATECOUNT];//用来判断 是否 学生走开
	void checkMoveStudentByFlow(cv::Mat mat);//判断学生是否已经走开，移动的光流 是否有连续 且 离开站起的区域

	void showStudentup();

	void AddUpstudent(cv::Rect rc,cv::Rect p0,cv::Rect p1,cv::Rect p2);
	void readyDeleteUpstudent(std::vector<upStudentTarget>::iterator &it,int itag,int ideletecount=5);//准备删除节点

	bool InitAndFlow();//返回是否有光流点

	//功能函数
	bool haveSameRect(cv::Rect r1,cv::Rect r2);
	cv::Rect getMaxRect(cv::Rect r1,cv::Rect r2);

	int getMaxFlowDis(cv::Rect p0,cv::Rect p1,cv::Rect p2,int i1=0,int i2=1,int i3=2);
	int getAvrFlowDis(cv::Rect p0,cv::Rect p1,cv::Rect p2,int i1=0,int i2=1,int i3=2);
	int getMaxFlowDis(cv::Rect p0);
	int getAvrFlowDis(cv::Rect p0);

	int getMinArea(cv::Rect r);

	bool checkHead(cv::Mat img1,cv::Mat img2,cv::Mat img3,int headwidth,int upheight,cv::Mat img4,cv::Mat img5,cv::Mat img6);



	//调试图像输出
	bool bdebugsaveimage;
	void saveimage();

};

