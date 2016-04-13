#include"detect_t.h"
#include <cstdio>

Static_s::Static_s()
{
    flag = false;
}

TeacherDetecting::TeacherDetecting(KVConfig * cfg)
 : cfg_(cfg)
{
    //bg_model.set("var_Threshold",80);
    //bg_model.set("fTau",0.2);//阴影消除参数，0-1之间，默认为0.5，越小，阴影消除越厉害
    //mog_learn_rate = atof(cfg_->get_value("t_mog_learn_rate","0.05"));
    up_update.region_interval = atof(cfg_->get_value("t_upbody_region_interval","40"));
    up_update.frame_num = 0;

    video_width_ = atof(cfg_->get_value("video_width", "480"));
    video_height_ = atof(cfg_->get_value("video_height", "270"));

    min_area = (video_width_/480.0)*atof(cfg_->get_value("min_area", "150"));
    min_rect_area = (video_width_/480.0)*atof(cfg_->get_value("min_rect_area", "300"));
    //up_update.min_area = (video_width_/480.0)*atof(cfg_->get_value("t_upbody_min_area", "1000"));
    up_update.min_rect_area = (video_width_/480.0)*atof(cfg_->get_value("upbody_min_rect_area", "1000"));

    up_update.is_upbody = false;

    luv_u_max = atof(cfg_->get_value("luv_u_max", "23"));
    luv_v_max = atof(cfg_->get_value("luv_v_max", "23"));
    luv_L = atof(cfg_->get_value("luv_L", "50"));

    up_update.Y_value = atoi(cfg_->get_value("upbody_L", "40"));
    up_update.upbody_u_max = atof(cfg_->get_value("upbody_u_max", "23"));
    up_update.upbody_v_max = atof(cfg_->get_value("upbody_v_max", "23"));

    init_fillbg_struct(fillbg_struct);

    //图像掩码;
    init_mask();

    //帧差法;
    init_frame_struct(frame_s);

    //初始化背景更新算法;
    reset(ud_bg_s);

    reset_upbody(up_update.bg_upbody);
}

TeacherDetecting::~TeacherDetecting()
{

}

//初始化背景更新算法;
void TeacherDetecting::init_fillbg_struct(Fill_Bg &fillbg_struct)
{
    //静止目标区域更新背景时间;
    fillbg_struct.continued_time =atof(cfg_->get_value("t_continued_time_static", "15"));
    //单目标无帧差背景更新时间;
    fillbg_struct.continued_time_long=atof(cfg_->get_value("t_continued_time_long", "100"));
    //检测到有目标走下讲台区后更新背景时间;
    fillbg_struct.mog2_s.continued_time=atof(cfg_->get_value("t_continued_time_teacherdown", "5"));
    //无目标背景更新时间;
    //fillbg_struct.norect_update_time=atof(cfg_->get_value("mog2_norect_update_time", "20"));

    //人体在图像中大概占的宽度;
    fillbg_struct.body_width = (video_width_/480.0)*atof(cfg_->get_value("t_body_width", "40"));
    fillbg_struct.mog2_interval1 =fillbg_struct.body_width*5;
    fillbg_struct.mog2_interval2 =fillbg_struct.body_width*3;
    fillbg_struct.second_interval =(video_width_/480.0)*atof(cfg_->get_value("mog2_second_interval", "10"));

    fillbg_struct.isfillok = false;
    fillbg_struct.isfillok_end = false;
    fillbg_struct.body_move = false;

    fillbg_struct.nframe = 0;
    fillbg_struct.num = 0;
    fillbg_struct.filltwice_num = 0;

    fillbg_struct.mog2_s.flag = false;
}

//初始化帧差法;
void TeacherDetecting::init_frame_struct(Frame_struct &frame_s)
{
    frame_s.is_body_down = false;
    frame_s.N = 2;
    frame_s.threshold_three = atoi(cfg_->get_value("t_frame_threshold_three", "20"));
    frame_s.threshold_two = atoi(cfg_->get_value("t_frame_threshold_two", "25"));
    frame_s.interval = fillbg_struct.body_width*3;
    frame_s.minarea = (video_width_/480.0)*atoi(cfg_->get_value("t_frame_minarea", "10"));
    frame_s.minrect = (video_width_/480.0)*atoi(cfg_->get_value("t_frame_minrect", "100"));
    frame_s.bottom_inter = (video_width_/480.0)*atoi(cfg_->get_value("t_frame_bottom_inter_new", "50"));
}

//初始化图像掩码;
void TeacherDetecting::init_mask( )
{
    ismask_ = false;
    masked_rect = get_rect(Rect(0, 0, video_width_, video_height_), cfg_->get_value("calibration_data", "0"),cfg_->get_value("calibration_data_2", "0"));
    const char*cb_date,*cb_date_2;
    if(cfg_->get_value("calibration_data", "0"))
        cb_date = "calibration_data";
    else
        cb_date = NULL;
    if(cfg_->get_value("calibration_data_2", "0"))
        cb_date_2 = "calibration_data_2";
    else
        cb_date_2 = NULL;
    ismask_ = build_mask(img_mask_, masked_rect, cb_date, cb_date_2);

    is_up_mask_ = false;
    upbody_masked_rect = get_rect(Rect(0, 0, video_width_, video_height_), cfg_->get_value("upbody_calibration_data", 0));
    const char *up_cb_date;
    if(cfg_->get_value("upbody_calibration_data", "0"))
        up_cb_date = "upbody_calibration_data";
    else
        up_cb_date = NULL;
    is_up_mask_ = build_mask(upbody_img_mask_, upbody_masked_rect, up_cb_date);

    screen_rect = get_rect(Rect(0, 0, 0, 0), cfg_->get_value("upbody_calibration_data_hollow", 0));
    screen_rect = Rect(screen_rect.x, 0, screen_rect.width, video_height_); // 上下扩展 ...
    //-----------------------------
    //ismask_ = false;
    //masked_rect = get_rect(cfg_->get_value("calibration_data", "0"),cfg_->get_value("calibration_data_2", "0"));
    //int expend_height = fillbg_struct.body_width;
    //cv::Rect upbody_rect = get_rect(cfg_->get_value("upbody_calibration_data", 0),cfg_->get_value("up_calibration_data_2", "0"));
    //upbody_masked_rect = cv::Rect(upbody_rect.x,upbody_rect.y-3*expend_height,
    //  upbody_rect.width,(3*expend_height + upbody_rect.height+expend_height));//&&&&&&&&&&&&&&;
    //upbody_masked_rect &= cv::Rect(0,0,video_width_,video_height_);
    //const char*cb_date,*cb_date_2;
    //if(cfg_->get_value("calibration_data", "0"))
    //  cb_date = "calibration_data";
    //else
    //  cb_date = NULL;
    //if(cfg_->get_value("calibration_data_2", "0"))
    //  cb_date_2 = "calibration_data_2";
    //else
    //  cb_date_2 = NULL;
    //img_mask_ = build_mask(cb_date,cb_date_2);
}


//初始化掩码区的区域分割;
void TeacherDetecting::reset(Update_Bg &bg)
{
    bg.region_interval = fillbg_struct.body_width;
    bg.time = fillbg_struct.continued_time_long;
    bg.multiple_target_time = fillbg_struct.continued_time;
    bg.slow_learn_rate = atof(cfg_->get_value("t_slow_learn_rate", "0.1"));
    bg.fast_learn_rate = atof(cfg_->get_value("t_fast_learn_rate", "0.7"));
    bg.region_num = masked_rect.width/bg.region_interval+1;
    for(int i = 0;i<bg.region_num;i++)
    {
        Region temp;
        reset_region(temp);
        reset_static_region( temp );
        Rect t = Rect(i*bg.region_interval,0,bg.region_interval,masked_rect.height);
        t &= Rect(0,0,masked_rect.width,masked_rect.height);
        temp.region = t;
        temp.num = i;//0,1,2,...
        bg.region.push_back(temp);
    }
}

//初始化掩码区的区域分割;
void TeacherDetecting::reset_upbody(Update_Bg &bg)//&&&&&&&&&&&&;
{
    bg.region_interval = fillbg_struct.body_width;
    bg.time = fillbg_struct.continued_time_long;
    bg.multiple_target_time = fillbg_struct.continued_time;
    bg.slow_learn_rate = atof(cfg_->get_value("t_slow_learn_rate", "0.1"));
    bg.fast_learn_rate = atof(cfg_->get_value("t_fast_learn_rate", "0.5"));
    bg.region_num = masked_rect.width/up_update.region_interval+1;
    for(int i = 0;i<bg.region_num;i++)
    {
        Region temp;
        reset_region(temp);
        reset_static_region( temp );
        Rect t = Rect(i*up_update.region_interval,0,up_update.region_interval,upbody_masked_rect.height);
        //t &= Rect(0,0,masked_rect.width,upbody_masked_rect.height);
        temp.region = t;
        temp.num = i;//0,1,2,...
        bg.region.push_back(temp);
    }
}


//面积从大到小排序;
int TeacherDetecting::cmp_area(const Rect & a, const Rect & b)
{
    return (a.width * a.height > b.width * b.height);
}

//两矩形框进行融合;
//输入：两个矩形a、b
//返回：两个矩形融合之后的矩形框;
Rect TeacherDetecting::sort_rect(Rect a, Rect b)
{
    Rect rect_new;
    int small_x = a.x;
    int small_y = a.y;
    int big_x = a.x + a.width;
    int big_y = a.y + a.height;
    if (b.x < small_x)
        small_x = b.x;
    if (b.y < small_y)
        small_y = b.y;
    if (b.x + b.width > big_x)
        big_x = b.x + b.width;
    if (b.y + b.height > big_y)
        big_y = b.y + b.height;
    rect_new = Rect(small_x, small_y, big_x - small_x, big_y - small_y);
    return rect_new;
}

//运动区域融合(矩形框之间间隔小于小于图像宽度的五十分之一时融合为一个框);
void TeacherDetecting::rect_fusion2(vector < Rect > &seq, double interval)
{
    //对运动框先按面积大小进行排序，由大到小 ;
    std::sort(seq.begin(), seq.end(), cmp_area);
    Rect rect_i;
    Rect rect_j;
    int num = 0;
    for (;;) {
        std::vector < cv::Rect >::iterator it1;
        std::vector < cv::Rect >::iterator it2;
        num = 0;
        for (it1 = seq.begin(); it1 != seq.end();)
        {
            for (it2 = it1 + 1; it2 != seq.end();)
            {
                rect_i = *it1;
                rect_j = *it2;
             if (((rect_i.x+interval) <(rect_j.x + rect_j.width)
                || (rect_i.x + rect_i.width ) >(rect_j.x+interval)
                ||(rect_i.y+fillbg_struct.body_width*4) <(rect_j.y + rect_j.height)
                || (rect_i.y + rect_i.height ) >(rect_j.y+fillbg_struct.body_width*4)) &&
                (rect_i.x >(rect_j.x + rect_j.width + 30)
                    || (rect_i.x + rect_i.width + 30) <rect_j.x
                    || rect_i.y >(rect_j.y + rect_j.height + fillbg_struct.body_width*2)
                    || (rect_i.y + rect_i.height + fillbg_struct.body_width*2) <rect_j.y))
                {
                    it2++;
                    continue;
                } else  //当矩形框之间有交集时进行融合;
                {
                    *it1 = sort_rect(rect_i, rect_j);
                    it2 = seq.erase(it2);
                    num++;
                }
            }
            it1++;
        }
        //当所有矩形框之间不可再融合时停止循环;
        if (num == 0)
        {
            //重新按面积排序;
            std::sort(seq.begin(), seq.end(), cmp_area);
            break;
        }

    }

}

std::vector < Rect > TeacherDetecting::refineSegments2(Mat img, Mat & mask,
                               Mat & dst,
                               double interval,double marea,double mrect_area)
{
    vector < Rect > rect;
    int niters = 2;
    vector < vector < Point > >contours;
    vector < vector < Point > >contours_temp;
    vector < Vec4i > hierarchy;
    vector < Rect > right_rect;
    Mat temp;
    cv::dilate(mask, mask, cv::Mat());
    cv::erode(mask, mask, cv::Mat());
    //cv::erode(mask, mask, cv::Mat());
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

    //如果行或列是奇数要加一变成偶数处理，因为cvPyrUp和cvPyrDown只支持偶数;
    //IplImage* pyr=cvCreateImage(cvSize((mask.cols&-2)/2,(mask.rows&-2)/2),IPL_DEPTH_8U,1);
    ////IplImage* pyr=cvCreateImage(cvSize(mask.cols/2,mask.rows/2),IPL_DEPTH_8U,1);
    //cvPyrDown(&(IplImage)mask,pyr,CV_GAUSSIAN_5x5);
    //cvDilate(pyr,pyr,0,1);
    //cvPyrUp(pyr,&(IplImage)mask,CV_GAUSSIAN_5x5);
    //cvReleaseImage(&pyr);

    //找出画出超过一定面积的连通区域;
    findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL,
             CV_CHAIN_APPROX_SIMPLE);
    dst = Mat::zeros(img.size(), CV_8UC3);
    if (contours.size() > 0) {
        Scalar color(255, 255, 255);
        for (int idx = 0; idx < contours.size(); idx++) {
            const vector < Point > &c = contours[idx];
            Rect t = boundingRect(Mat(c));
            double area = fabs(contourArea(Mat(c)));
            if (area >= marea && (t.width*t.height)>=mrect_area)    //temp.rows*temp.cols/150)// &&
            {
                contours_temp.push_back(contours[idx]);
                right_rect.push_back(t);
            }
        }
        drawContours(dst, contours_temp, -1, color, CV_FILLED, 8);
    }
    if (right_rect.size() > 1) {
        rect_fusion2(right_rect, interval);
    }
    return right_rect;

}


std::vector < Rect > TeacherDetecting::upbody_refineSegments2(Mat img, Mat & mask,
                               Mat & dst,
                               double interval,double marea,double mrect_area)
{
    vector < Rect > rect;
    int niters = 2;
    vector < vector < Point > >contours;
    vector < vector < Point > >contours_temp;
    vector < Vec4i > hierarchy;
    vector < Rect > right_rect;
    Mat temp;
    //cv::erode(mask, mask, cv::Mat());
    cv::dilate(mask, mask, cv::Mat());
    cv::erode(mask, mask, cv::Mat());
    //cv::erode(mask, mask, cv::Mat());
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

    //找出画出超过一定面积的连通区域;
    findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL,
             CV_CHAIN_APPROX_SIMPLE);
    dst = Mat::zeros(img.size(), CV_8UC3);
    if (contours.size() > 0) {
        Scalar color(255, 255, 255);
        for (int idx = 0; idx < contours.size(); idx++) {
            const vector < Point > &c = contours[idx];
            Rect t = boundingRect(Mat(c));
            double area = fabs(contourArea(Mat(c)));
            if (t.width*t.height >= mrect_area) //temp.rows*temp.cols/150)// &&
            {
                contours_temp.push_back(contours[idx]);
                right_rect.push_back(t);
            }
        }
        drawContours(dst, contours_temp, -1, color, CV_FILLED, 8);
    }
    if (right_rect.size() > 1) {
        rect_fusion2(right_rect, interval);
    }
    return right_rect;

}

void TeacherDetecting::fillbg_LUV( Mat img)
{
    //获取蓝框;
    if(fillbg_struct.rect_old.size()== 1 && fillbg_struct.nframe == 0)
    {
        if(fillbg_struct.rect_old[0].width<3*fillbg_struct.body_width)
        {
            Rect r;
            //fillbg_struct.bg = img.clone();
            //fillbg_struct.fist_fillrect_t = fillbg_struct.rect_old;
            //当出现的矩形小于定义的可能出现的矩形最大框时正常处理;
            Rect t = fillbg_struct.rect_old[0];
            r = Rect(t.x+t.width/2-(fillbg_struct.body_width-20),t.y+t.height/2-2*fillbg_struct.body_width,
                2*(fillbg_struct.body_width+20),4*fillbg_struct.body_width);
            r &= Rect(0, 0, img.cols, img.rows);
            fillbg_struct.fist_fillrect.push_back(r);
            //fillbg_struct.fist_fillrect_noclear.push_back(r);
            fillbg_struct.nframe = 2;
        }
        else
        {
            fillbg_struct.bg = img.clone();
        }
    }
    if(fillbg_struct.nframe == 2)
    {
        //eliminate_longrect(img);
        bool flag = false;
        for(int i = 0;i<fillbg_struct.rect_old.size();i++)
        {
            Rect t = fillbg_struct.rect_old[i];
            Rect t_f = fillbg_struct.fist_fillrect[0];
            //if((t.x+t.width)<(t_f.x-fillbg_struct.second_interval)||t.x>(t_f.x+t_f.width+fillbg_struct.second_interval))
            if((t.x+t.width)<(t_f.x)||t.x>(t_f.x+t_f.width))
            {
                flag = true;continue;
            }
            else
            {
                flag = false;break;
            }
        }
        if(flag)//可以进行更新啦;
        {
            Mat mask(img.rows, img.cols, CV_8UC3,
                    Scalar(0, 0, 0));
            Rect r_temp = fillbg_struct.fist_fillrect[0];
            r_temp &= Rect(0, 0, img.cols, img.rows);
            Mat specified(mask, r_temp);
            specified.setTo(1);
            img.copyTo(fillbg_struct.bg, mask);
            fillbg_struct.nframe = 3;
            fillbg_struct.isfillok = true;
            //fillbg_struct.fist_fillrect.clear();
        }
    }
}


//判断蓝框更新完之后是否是错误的更新或是蓝框两侧还有没被更新的目标;
//蓝框更新后如果连续10次都是一个目标则表示更新完毕（否则要一直运行着此函数的判断）;
void TeacherDetecting::is_need_fillbg_twice(Mat img)
{
    //判断蓝框外是否是连续5次单目标(这是判断蓝框是否更新对的一个依据);
    if(fillbg_struct.nframe == 3 && fillbg_struct.rect_old.size() == 1 && !fillbg_struct.isfillok_end)
    {
        Rect r = fillbg_struct.rect_old[0];
        Rect r_first = fillbg_struct.fist_fillrect[0];
        //只有一个矩形且矩形在原始蓝框范围外则表明没有错误更新;
        if((r.x>=(r_first.x+r_first.width+10)) ||((r.x+r.width)<=(r_first.x-10)))
        {
            fillbg_struct.filltwice_num++;
            if (atoi(cfg_->get_value("debug", "0")) > 0)
            {fprintf(stderr,"%d\n",fillbg_struct.filltwice_num);}
        }
        if(fillbg_struct.filltwice_num>=5)
        {
            fillbg_struct.isfillok_end = true;
            fillbg_struct.filltwice_num = 0;
        }

    }
    ////静止目标框更新过之后没有目标了的话，那表明可能是错误更新（把静止的人给误更新啦）;
    //if(fillbg_struct.ud_static_begain && fillbg_struct.rect_old.size() == 0)
    //{
    //  fillbg_struct.ud_static_begain = false;
    //  fillbg_struct.bg = img.clone();
    //  fillbg_struct.nframe = 0;
    //  fillbg_struct.isfillok = false;
    //  fillbg_struct.isfillok_end = false;
    //  fillbg_struct.bg = img.clone();
    //  fillbg_struct.fist_fillrect.clear();
    //  fillbg_struct.filltwice_num = 0;
    //}
}


////如果长时间没有目标则更新整个背景区(若一直没目标，则一定时间更新一次).
//void TeacherDetecting::norect_update_bg( Mat img)
//{
//  if(fillbg_struct.rect_old.size() > 0)
//  {
//      fillbg_struct.no_rect.pre_time = fillbg_struct.no_rect.cur_time = clock();
//  }
//  if(fillbg_struct.rect_old.size() == 0)
//  {
//      fillbg_struct.no_rect.cur_time = clock();
//      fillbg_struct.no_rect.continued_time = double(fillbg_struct.no_rect.cur_time-fillbg_struct.no_rect.pre_time)/CLOCKS_PER_SEC;
//      if(fillbg_struct.no_rect.continued_time>fillbg_struct.norect_update_time)
//      {
//          //更新整个背景.
//          //Mat mask(img.rows,img.cols, CV_8UC3, Scalar(0,0,0));
//          img.copyTo(fillbg_struct.bg,img);
//          fillbg_struct.no_rect.pre_time = fillbg_struct.no_rect.cur_time = clock();
//      }
//  }
//}


////在进行第二次更新背景之前，如果出现多于初始框的个数的矩形框时需要把不用的原始位置的矩形去掉.
////目的，去除红框拉的太长的现象.
//void TeacherDetecting::eliminate_longrect(Mat img)
//{
//      std::vector<Rect> temp;
//      if(fillbg_struct.rect_old.size()>fillbg_struct.fist_fillrect_original.size())
//      {
//          for(int i = 0;i<fillbg_struct.rect_old.size();i++)
//          {
//              Rect t = fillbg_struct.rect_old[i];
//              Rect t_f = fillbg_struct.fist_fillrect_original[0];
//              t_f.x = t_f.x+(t_f.width*3.0/8.0);t_f.width = t_f.width/4.0;
//              if((t.x+t.width)<t_f.x||t.x>(t_f.x+t_f.width))
//                  temp.push_back(t);
//          }
//          if(temp.size()>0)
//          {
//              fillbg_struct.rect_old.clear();
//              fillbg_struct.rect_old = temp;
//          }
//          else//融合起来.
//          {
//              double x_max=0, y_max=0, x_min=img.cols, y_min=img.rows;
//              for(int i = 0;i<fillbg_struct.rect_old.size();i++)
//              {
//                  Rect t = fillbg_struct.rect_old[i];
//                  if(x_max<(t.x+t.width)) x_max = t.x+t.width;
//                  if(y_max<(t.y+t.height)) y_max = t.y+t.height;
//                  if(x_min>t.x) x_min = t.x;
//                  if(y_min>t.y) y_min = t.y;
//              }
//              Rect r = Rect(x_min,y_min,abs(x_max-x_min),abs(y_max-y_min));
//              r &= Rect(0,0,img.cols,img.rows);
//              fillbg_struct.rect_old.clear();
//              fillbg_struct.rect_old.push_back(r);
//          }
//      }
//
//}

//YUV算法获取矩形框序列;
void TeacherDetecting::upbody_luv_method(const Mat &img )
{
    Mat luv_m,luv_m_temp,fgimg;//背景减除;
    luv_m.create(Size(img.cols, img.rows), CV_8UC1);
    luv_m.setTo(0);
    luv_m_temp = img.clone();
    luv_m_temp.setTo(Scalar::all(255));
    Mat img_t; Mat bg_t;
    /*cvtColor(img, img_t, CV_BGR2Luv);
    cvtColor(fillbg_struct.bg, bg_t, CV_BGR2Luv);*/
    cvtColor(img, img_t, CV_BGR2YUV);
    cvtColor(up_update.upbody_bg, bg_t, CV_BGR2YUV);

    /*std::vector<Mat> img_t_vec;
    split(img_t,img_t_vec);
    Mat img0 = img_t_vec[0];Mat img1 = img_t_vec[1];Mat img2 = img_t_vec[2];
    std::vector<Mat> bg_t_vec;
    cv::split(bg_t,bg_t_vec);
    Mat bg0 = bg_t_vec[0];Mat bg1 = bg_t_vec[1];Mat bg2 = bg_t_vec[2];
    Mat yuv0,yuv1,yuv2;
    cv::absdiff(img0,bg0,yuv0);
    cv::absdiff(img1,bg1,yuv1);
    cv::absdiff(img2,bg2,yuv2);
    Mat luv0,luv1,luv2;
    cv::threshold(yuv0,luv0,up_update.Y_value,255,CV_THRESH_BINARY);
    cv::threshold(yuv1,luv1,up_update.upbody_u_max,255,CV_THRESH_BINARY);
    cv::threshold(yuv2,luv2,up_update.upbody_v_max,255,CV_THRESH_BINARY);
    Mat luv_temp;
    bitwise_or(luv0,luv1,luv_temp);
    bitwise_or(luv2,luv_temp,luv_m);*/


    for (int i = 0; i < img.cols; i++)
    {
        for (int j = 0; j < img.rows; j++)
        {
            Vec3b bgr1 = img_t.at < Vec3b > (j, i);
            Vec3b bgr2 = bg_t.at < Vec3b > (j, i);
            double L =
                (abs) (bgr1.val[0] - bgr2.val[0]);
            double U =
                (abs) (bgr1.val[1] - bgr2.val[1]);
            double V =
                (abs) ((bgr1.val[2] - bgr2.val[2]));
            if ((U >= up_update.upbody_u_max || V >= up_update.upbody_v_max)&&(L >= up_update.Y_value))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(0,0,0);
            }
            else if ((U >= up_update.upbody_u_max || V >= up_update.upbody_v_max)&&(L < up_update.Y_value))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(0,0,255);
            }
            else if ((U < up_update.upbody_u_max && V < up_update.upbody_v_max)&&(L > up_update.Y_value))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(255,0,0);
            }
            else
            {
                luv_m.at < char >(j, i) = 0;
            }

        }
    }

    up_update.upbody_rect =
        upbody_refineSegments2(img, luv_m, fgimg,
                fillbg_struct.mog2_interval2,up_update.min_area,up_update.min_rect_area);
    if(atoi(cfg_->get_value("debug","0"))>0)
    {
        //imshow("upbody_fgimg",fgimg);
        imshow("upbody_luv_m_temp",luv_m_temp);
        waitKey(1);
    }

}
//YUV算法获取矩形框序列;
void TeacherDetecting::luv_method(const Mat &img )
{
    Mat luv_m,luv_m_temp,fgimg;//背景减除;
    luv_m.create(Size(img.cols, img.rows), CV_8UC1);
    luv_m.setTo(0);
    luv_m_temp = img.clone();
    luv_m_temp.setTo(Scalar::all(255));
    Mat img_t; Mat bg_t;
    /*cvtColor(img, img_t, CV_BGR2Luv);
    cvtColor(fillbg_struct.bg, bg_t, CV_BGR2Luv);*/
    cvtColor(img, img_t, CV_BGR2YUV);
    cvtColor(fillbg_struct.bg, bg_t, CV_BGR2YUV);
    for (int i = 0; i < img.cols; i++)
    {
        for (int j = 0; j < img.rows; j++)
        {
            Vec3b bgr1 = img_t.at < Vec3b > (j, i);
            Vec3b bgr2 = bg_t.at < Vec3b > (j, i);
            double L =
                (abs) (bgr1.val[0] - bgr2.val[0]);
            double U =
                (abs) (bgr1.val[1] - bgr2.val[1]);
            double V =
                (abs) ((bgr1.val[2] - bgr2.val[2]));
            if ((U >= luv_u_max || V >= luv_v_max)&&(L >= luv_L))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(0,0,0);
            }
            else if ((U >= luv_u_max || V >= luv_v_max)&&(L < luv_L))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(0,0,255);
            }
            else if ((U < luv_u_max && V < luv_v_max)&&(L > luv_L))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(255,0,0);
            }
            else
            {
                luv_m.at < char >(j, i) = 0;
            }

        }
    }

    fillbg_struct.rect_old =
        refineSegments2(img, luv_m, fgimg,
                fillbg_struct.mog2_interval2,min_area,min_rect_area);
    if(atoi(cfg_->get_value("debug","0"))>0)
    {
        //imshow("fgimg",fgimg);
        imshow("luv_m_temp",luv_m_temp);
        waitKey(1);
    }

}


//人是否走下讲台区的判定;
//判断给定的矩形框内是否有帧差矩形框，若一定时间没有则更新蓝框(注意有可能是误更新);
void TeacherDetecting::is_teacher_down(Mat raw_img,Mat img2)
{
        Rect t = fillbg_struct.fist_fillrect[0];
        Rect temp = Rect(t.x-50,masked_rect.y,t.width+50*2,masked_rect.height+(video_width_/480.0)*30);
        temp &= Rect(0,0,raw_img.cols,raw_img.rows);
        for(int k = 0;k<frame_s.frame_rect.size();k++)
        {
            if(
            frame_s.frame_rect[k].y>(masked_rect.y+masked_rect.height+(video_width_/480.0)*10)
            &&frame_s.frame_rect[k].y<(masked_rect.y+masked_rect.height+frame_s.bottom_inter)
            &&frame_s.frame_rect[k].x>=temp.x
            &&(frame_s.frame_rect[k].x+frame_s.frame_rect[k].width)<=(temp.x+temp.width))//&& frame_s.frame_rect.size()==1
            {
                frame_s.is_body_down = true;
                fprintf(stderr,"teacher is down!\n");
                break;
            }
        }
        bool no_rect = false;
        //全局都没有矩形框;
        if(frame_s.frame_rect.size()<=0)
            no_rect = true;
        else//给定的矩形框内没有mog矩形;
        {
            bool flag = true;
            for(int i = 0;i<frame_s.frame_rect.size();i++)
            {
                Rect m_r = frame_s.frame_rect[i];
                if( (m_r.x+m_r.width)<(temp.x) || m_r.x>(temp.x+temp.width)
                    || (m_r.y+m_r.height)<(temp.y) || m_r.y>(temp.y+temp.height))
                {continue;}
                else
                {
                    flag = false;break;
                }
            }
            if(flag)
            {no_rect = true;}
        }
        if( no_rect && frame_s.is_body_down)
        {
            if(!fillbg_struct.mog2_s.flag)
            {
                fillbg_struct.mog2_s.cur_time = fillbg_struct.mog2_s.pre_time = clock();
                fillbg_struct.mog2_s.flag = true;
            }
            else
            {
                fillbg_struct.mog2_s.cur_time = clock();
            }
        }
        else
        {
            fillbg_struct.mog2_s.cur_time = fillbg_struct.mog2_s.pre_time = clock();
            frame_s.is_body_down = false;
            fillbg_struct.mog2_s.flag = false;
        }
        double interval_t =((double)(fillbg_struct.mog2_s.cur_time - fillbg_struct.mog2_s.pre_time) / CLOCKS_PER_SEC);

        if(atoi(cfg_->get_value("debug","0"))>0)
        {
            if(interval_t>0)
            {fprintf(stderr,"teacher_down_time = %f\n",interval_t);}
        }
        //更新蓝框区域背景;
        if(interval_t>fillbg_struct.mog2_s.continued_time)
        {
            fillbg_struct.mog2_s.cur_time = fillbg_struct.mog2_s.pre_time = clock();
            updatebg(img2,fillbg_struct.fist_fillrect[0]);
            //这个地方有待改进？？？？？（不能直接就这么算了）;
            fillbg_struct.nframe = 3;
            fillbg_struct.isfillok = true;
            //fillbg_struct.fist_fillrect.clear();

            frame_s.is_body_down = false;
        }

}

//void TeacherDetecting::creat_buffer(IplImage *image)
//{
//  buffer = (IplImage**)malloc(sizeof(buffer[0])*N);
//  //buffer = new IplImage*[N];
//  for(int i = 0;i<N;i++)
//  {
//      buffer[i]= cvCreateImage(cvSize(image->width,image->height),IPL_DEPTH_8U,1);
//      cvSetZero(buffer[i]);
//  }
//}

//**************************两帧差分法***********************************
//输入：img//当前图像;
//输出：silh//灰度图（当前帧图像和保存的上一帧图像相减后图像）;
//buflen设为2：表示连续两帧之间做帧差;
void TeacherDetecting::two_frame_method(Mat img,Mat &silh)
{
    if(frame_s.buffer[0].empty())
    {
        for(int i = 0;i<(frame_s.N);i++)
        {
            cvtColor(img,frame_s.buffer[i],CV_BGR2GRAY);
        }
    }
    cvtColor(img,frame_s.buffer[1],CV_BGR2GRAY);
    absdiff(frame_s.buffer[1],frame_s.buffer[0],silh);
    threshold( silh, silh, frame_s.threshold_two, 255, CV_THRESH_BINARY );
    //cvSmooth(&(IplImage)silh,&(IplImage)silh,CV_MEDIAN,3,0,0,0);
    medianBlur(silh, silh, 3);
    for(int i = 0;i<frame_s.N-1;i++)
    {
        frame_s.buffer[i] = frame_s.buffer[i+1].clone();
    }

}


//**************************三帧差分法***********************************
void TeacherDetecting::three_frame_method(Mat img,Mat &silh)
{
    Mat silh_one;silh_one.setTo(Scalar::all(0));
    Mat silh_two;silh_two.setTo(Scalar::all(0));
    if(frame_s.buffer[0].empty())
    {
        for(int i = 0;i<(frame_s.N+1);i++)
        {
            //frame_s.buffer[i].create(Size(video_width_,video_height_),CV_8UC1);
            cvtColor(img,frame_s.buffer[i],CV_BGR2GRAY);
        }
    }
    cvtColor(img,frame_s.buffer[2],CV_BGR2GRAY);

    absdiff(frame_s.buffer[1],frame_s.buffer[0],silh_one);
    threshold( silh_one, silh_one, frame_s.threshold_three, 255, CV_THRESH_BINARY );

    absdiff(frame_s.buffer[2],frame_s.buffer[1],silh_two);
    threshold( silh_two, silh_two, frame_s.threshold_three, 255, CV_THRESH_BINARY );

    //cvAnd(&(IplImage)silh_one,&(IplImage)silh_two,&(IplImage)silh);
    bitwise_and(silh_one, silh_two, silh);
    //cvSmooth(&(IplImage)silh,&(IplImage)silh,CV_MEDIAN,3,0,0,0);
    medianBlur(silh, silh, 3);
    for(int i = 0;i<frame_s.N-1;i++)
    {
        frame_s.buffer[i] = frame_s.buffer[i+1].clone();
    }

}


//*****************************帧差法*************************************
void TeacherDetecting:: frame_difference_method (Mat raw_image,std::vector<Rect> &frame_rect,std::vector<Rect> &masked_frame_rect)
{
    //全图像帧差法;(目的:判断教师是否走下讲台区);
    frame_s.frame_rect.clear();
    std::vector<Rect> rect_vector_t;
    Mat silh;silh.create(Size(raw_image.cols,raw_image.rows),CV_8UC1);
    /*if(fillbg_struct.isfillok_end)
        frame_s.N = 3;
    if(frame_s.N == 3)
    three_frame_method(raw_image,silh);
    else if(frame_s.N == 2)*/
    two_frame_method(raw_image,silh);
    if(!fillbg_struct.isfillok_end)
    {
        /*Rect mk = Rect(0,(masked_rect.y-3*fillbg_struct.body_width),
            raw_image.cols,(raw_image.rows-(masked_rect.y-3*fillbg_struct.body_width)-frame_s.bottom_inter));*/
        Rect mk = Rect(0,masked_rect.y,
            raw_image.cols,(masked_rect.height+frame_s.bottom_inter));
        mk &= Rect(0,0,raw_image.cols,raw_image.rows);
        Mat image_mk = Mat(raw_image,mk);
        Mat silh_mk = Mat(silh,mk);
        Mat dst_mk;
        rect_vector_t = refineSegments2(image_mk,silh_mk,dst_mk,frame_s.interval,frame_s.minarea,frame_s.minrect);
        for(int i = 0;i<rect_vector_t.size();i++)
        {
            Rect t = rect_vector_t[i];
            //if(t.width*t.height>(pow(fillbg_struct.body_width/3.0,2)))
            if(t.width*t.height>frame_s.minrect)
            {
                t = Rect(t.x+mk.x,t.y+mk.y,t.width,t.height);
                frame_rect.push_back(t);
                /*if(atoi(cfg_->get_value("debug","0"))>0)
                {
                    rectangle(raw_image,Rect(t.x,t.y,t.width,t.height),Scalar(0,0,0),2);
                }*/
            }
        }
    }
    //------------------------------------------
    //掩码部分帧差;
    frame_s.masked_frame_rect.clear();
    std::vector<Rect> masked_rect_vector_t;
    Mat image_mask = Mat(raw_image,masked_rect);
    Mat silh_mask = Mat(silh,masked_rect);
    Mat dst_masked;
    masked_rect_vector_t = refineSegments2(image_mask,silh_mask,dst_masked,frame_s.interval,frame_s.minarea,frame_s.minrect);
    for(int i = 0;i<masked_rect_vector_t.size();i++)
    {
        Rect t = masked_rect_vector_t[i];
        masked_frame_rect.push_back(t);
    }
}


//更新某个区域的背景图;
void TeacherDetecting::updatebg(Mat img,Rect r)
{
    Mat mask(img.rows, img.cols, CV_8UC3,
                    Scalar(0, 0, 0));
    Rect r_temp = r;
    r_temp &= Rect(0, 0, img.cols, img.rows);
    Mat specified(mask, r_temp);
    specified.setTo(1);
    img.copyTo(fillbg_struct.bg, mask);

}

void TeacherDetecting::reset_region( Region &region )
{
    region.has_frame_rect = false;
    region.has_old_rect = false;
    region.cur_tbg = 0;
    region.flage_bg = false;
    region.pre_tbg = 0;
    region.continuetime_bg = 0.0;
}

void TeacherDetecting::reset_static_region( Region &region )
{
    region.cur_static = 0;
    region.flag_static = false;
    region.pre_static = 0;
    region.continuetime_static = 0.0;
}

//缓慢更新某个区域的背景图;
void TeacherDetecting::updatebg_slow(Mat img,Rect r,double learn_rate)
{
    Rect r_temp = r;
    r_temp &= Rect(0, 0, img.cols, img.rows);
    Mat bg_t = fillbg_struct.bg.clone();
    Mat img_t = img.clone();
    Mat dst = Mat(Size(img.cols,img.rows),CV_8UC3);
    double rate = 1 - learn_rate;
    addWeighted(img_t,learn_rate,bg_t,rate,0,dst);

    Mat mask(img.rows, img.cols, CV_8UC3,Scalar(0, 0, 0));
    Mat specified(mask, r_temp);
    specified.setTo(1);
    dst.copyTo(fillbg_struct.bg, mask);
}

//(身高自适应)缓慢更新某个区域的背景图;
void TeacherDetecting::upbody_updatebg_slow(Mat img,Rect r,double learn_rate)
{
    Rect r_temp = r;
    r_temp &= Rect(0, 0, img.cols, img.rows);
    Mat bg_t = up_update.upbody_bg.clone();
    Mat img_t = img.clone();
    Mat dst = Mat(Size(img.cols,img.rows),CV_8UC3);
    double rate = 1 - learn_rate;
    addWeighted(img_t,learn_rate,bg_t,rate,0,dst);

    Mat mask(img.rows, img.cols, CV_8UC3,Scalar(0, 0, 0));
    Mat specified(mask, r_temp);
    specified.setTo(1);
    dst.copyTo(up_update.upbody_bg, mask);
}

//利用帧差法进行背景更新;
void TeacherDetecting::frame_updatebg(Mat raw_img,Mat image)
{
    //每次都要清空;
    for(int j = 0;j<ud_bg_s.region_num;j++)
    {
        ud_bg_s.region[j].has_old_rect = false;
        ud_bg_s.region[j].has_frame_rect = false;
    }
    //************************************************************
    //红色矩形框所占的区域;
    std::vector<int> valid_oldrect;
    for(int i = 0;i<fillbg_struct.rect_old.size();i++)
    {
        Rect t_old = Rect(fillbg_struct.rect_old[i].x-15,fillbg_struct.rect_old[i].y,
            fillbg_struct.rect_old[i].width+30,fillbg_struct.rect_old[i].height);
        t_old &= Rect(0,0,image.cols,image.rows);
        fillbg_struct.rect_old[i] = t_old;//扩展一下红框;
        int left = t_old.x/ud_bg_s.region_interval;//这个地方可以再精确一点;
        int right = (t_old.x+t_old.width)/ud_bg_s.region_interval;
        for(int j = 0;j<ud_bg_s.region_num;j++)
        {
            if(j >= left && j <= right)
            {
                if(!ud_bg_s.region[j].has_old_rect)
                {
                    ud_bg_s.region[j].has_old_rect = true;
                    valid_oldrect.push_back(j);
                }
            }
        }
    }
    //************************************************************
    //不是红框遮盖区的区域都要清零处理;
    for(int j = 0;j<ud_bg_s.region_num;j++)
    {
        bool f = false;
        for(int i = 0;i<valid_oldrect.size();i++)
        {
            if(j == valid_oldrect[i])
            {
                f= true;
                break;
            }
        }
        if(!f)
        {
            reset_region(ud_bg_s.region[j]);
        }
    }
    //************************************************************
    //这里要去掉干扰的帧差(即在红框外的帧差);
    std::vector<Rect> masked_frame_rect_valid;
    for(int k = 0;k<frame_s.masked_frame_rect.size();k++)
    {
        Rect t = frame_s.masked_frame_rect[k];
        for(int i = 0;i<fillbg_struct.rect_old.size();i++)
        {
            Rect t_rectold = fillbg_struct.rect_old[i];
            if(!(t.x >(t_rectold.x + t_rectold.width + 20)
               || (t.x + t.width + 20) <t_rectold.x))
            {
                if(t.width<fillbg_struct.body_width)
                {
                    if((t.x-fillbg_struct.body_width/2)<t_rectold.x)
                    {
                        t = Rect(t_rectold.x,t.y,fillbg_struct.body_width*2,t.height);
                        t &= t_rectold;
                    }
                    else if((t.x+t.width+fillbg_struct.body_width/2)>t_rectold.x+t_rectold.width)
                    {
                        t = Rect(t_rectold.x+t_rectold.width-fillbg_struct.body_width*2,
                            t.y,fillbg_struct.body_width*2,t.height);
                        t &= t_rectold;
                    }
                    else
                    {
                        t = Rect((t.x+t.width/2-fillbg_struct.body_width),t.y,fillbg_struct.body_width*2,t.height);
                        t &= t_rectold;
                    }
                }
                else
                {
                    t = Rect((t.x-10),t.y,t.width+20,t.height);
                }
                t &= Rect(0,0,raw_img.cols,raw_img.rows);
                masked_frame_rect_valid.push_back(t);
                break;
            }
        }
    }
    /*if(atoi(cfg_->get_value("debug","0"))>0)
    {
        for(int k = 0;k<masked_frame_rect_valid.size();k++)
        {
            Rect t = masked_frame_rect_valid[k];
            rectangle(raw_img,Rect(t.x+masked_rect.x,t.y+masked_rect.y,t.width,t.height),Scalar(0,255,0),2);
        }
    }*/
    //************************************************************
    //获取帧差矩形所占的区域;
    std::vector<int> valid_framerect;
    for(int k = 0;k<masked_frame_rect_valid.size();k++)
    {
        Rect t = masked_frame_rect_valid[k];
        int left = t.x/ud_bg_s.region_interval;//这个地方可以再精确一点;
        int right = (t.x+t.width)/ud_bg_s.region_interval;
        for(int j = 0;j<ud_bg_s.region_num;j++)
        {
            if(j >= left && j <= right)
            {
                if(!ud_bg_s.region[j].has_frame_rect)
                {
                    ud_bg_s.region[j].has_frame_rect = true;
                    valid_framerect.push_back(j);
                }
            }
        }
    }
    //************************************************************
    //没有帧差框时除红框外的区域都缓慢的进行更新;
    if(valid_framerect.size()<1)
    {
        for(int j = 0;j<ud_bg_s.region_num;j++)
        {
            if(!ud_bg_s.region[j].has_old_rect)
            {
                updatebg_slow(image,ud_bg_s.region[j].region,ud_bg_s.slow_learn_rate);
            }
        }
    }
    //************************************************************
    //有帧差且背景未彻底更新完成之前时，同时有帧差和红框外的区域进行较快的更新;
    if(valid_framerect.size()>0 && !fillbg_struct.isfillok_end)
    {
        bool flag_t = false;
        for(int j = 0;j<ud_bg_s.region_num;j++)
        {
            if(ud_bg_s.region[j].has_frame_rect && ud_bg_s.region[j].has_old_rect )
            {
                flag_t = true;
                break;
            }
        }
        if(flag_t)
        {
            for(int j = 0;j<ud_bg_s.region_num;j++)
            {
                if(!(ud_bg_s.region[j].has_frame_rect && ud_bg_s.region[j].has_old_rect))
                {
                    updatebg_slow(image,ud_bg_s.region[j].region,ud_bg_s.fast_learn_rate);
                    //判断目标是否从初始位置移动了一段距离;
                    if(fillbg_struct.fist_fillrect.size()>0 && !fillbg_struct.body_move)
                    {
                        Rect t_t = fillbg_struct.fist_fillrect[0];
                        if(t_t.x+t_t.width/2 >= ud_bg_s.region[j].region.x &&
                            t_t.x+t_t.width/2 <= (ud_bg_s.region[j].region.x+ud_bg_s.region[j].region.width))
                        {
                            fillbg_struct.body_move = true;
                        }
                    }
                }
            }
        }
    }
    //************************************************************
    std::vector<int> valid_both;//同时有帧差和红框的区域;
    std::vector<int> valid_one;//只有红框没帧差的区域;
    for(int i = 0;i<valid_oldrect.size();i++)
    {
        int region_num = valid_oldrect[i];
        bool flag_t = false;
        for( int j = 0;j<valid_framerect.size();j++)
        {
            if( valid_framerect[j] == valid_oldrect[i])
            {
                valid_both.push_back(valid_oldrect[i]);
                flag_t = true;
                break;
            }
        }
        if(flag_t)
        {
            reset_region(ud_bg_s.region[region_num]);
            ud_bg_s.region[region_num].has_frame_rect = true;
        }
        if(!flag_t)
        {
            valid_one.push_back(valid_oldrect[i]);
            ud_bg_s.region[region_num].has_frame_rect = false;
            if(!ud_bg_s.region[region_num].flage_bg)
            {
                ud_bg_s.region[region_num].cur_tbg = ud_bg_s.region[region_num].pre_tbg = clock();
                ud_bg_s.region[region_num].flage_bg = true;
            }
            else
            {
                ud_bg_s.region[region_num].cur_tbg = clock();
                ud_bg_s.region[region_num].continuetime_bg =
                    double(ud_bg_s.region[region_num].cur_tbg - ud_bg_s.region[region_num].pre_tbg)/CLOCKS_PER_SEC;

                if(ud_bg_s.region[region_num].continuetime_bg > ud_bg_s.time)
                {
                    //大于一分钟直接更新;
                    updatebg(image,ud_bg_s.region[region_num].region);
                    reset_region(ud_bg_s.region[region_num]);
                }
            }
        }
    }
    //************************************************************
    //帧差区外的有效区域10秒无帧差认为是静止的目标，给予快速更新;
    for(int i = 0;i<ud_bg_s.region_num;i++)
    {
        bool flag = false;
        for(int j = 0;j<valid_one.size();j++)
        {
            if(i == valid_one[j])
            {
                flag = true;
                break;
            }
        }
        if(!flag)
        {
            reset_static_region(ud_bg_s.region[i]);
        }
    }
    for(int i = 0;i<valid_one.size();i++)
    {
        if(ud_bg_s.region[valid_one[i]].flag_static)
        {
            ud_bg_s.region[valid_one[i]].cur_static = clock();
            ud_bg_s.region[valid_one[i]].continuetime_static =
            double(ud_bg_s.region[valid_one[i]].cur_static - ud_bg_s.region[valid_one[i]].pre_static)/CLOCKS_PER_SEC;
            if(ud_bg_s.region[valid_one[i]].continuetime_static > ud_bg_s.multiple_target_time)
            {
                //大于10s开始更新;
                if(valid_oldrect.size()>1)
                {
                    updatebg(image,ud_bg_s.region[valid_one[i]].region);
                    reset_static_region(ud_bg_s.region[valid_one[i]]);
                    reset_region(ud_bg_s.region[valid_one[i]]);
                }
            }
        }
        else if(valid_both.size()> 0 && !ud_bg_s.region[valid_one[i]].flag_static)
        {
            ud_bg_s.region[valid_one[i]].cur_static = ud_bg_s.region[valid_one[i]].pre_static = clock();
            ud_bg_s.region[valid_one[i]].flag_static = true;
        }
        else
        {
            reset_static_region(ud_bg_s.region[valid_one[i]]);
        }
    }

}

void TeacherDetecting::upbody_bgupdate( Mat upbody_img)
{
    // 根据红框更新身高探测区的背景 ...
    if(fillbg_struct.nframe >= 2 && fillbg_struct.rect_old.size() == 1)
    {
        Rect t = Rect(fillbg_struct.rect_old[0].x + masked_rect.x, fillbg_struct.rect_old[0].y + masked_rect.y,
                           fillbg_struct.rect_old[0].width, fillbg_struct.rect_old[0].height);
        t &= Rect(0, 0, video_width_, video_height_);

        if(t.x > upbody_masked_rect.x && t.x < upbody_masked_rect.x + upbody_masked_rect.width)
        {
            Rect temp = Rect(0, 0, (t.x - upbody_masked_rect.x), upbody_masked_rect.height);
            upbody_updatebg_slow(upbody_img, temp, 0.7);
            //rectangle(upbody_img, temp, Scalar(0, 0, 255), 2);
        }
        if((t.x + t.width) < upbody_masked_rect.x + upbody_masked_rect.width && (t.x + t.width) > upbody_masked_rect.x)
        {
            Rect temp = Rect( t.x + t.width - upbody_masked_rect.x, 0, ((upbody_masked_rect.x + upbody_masked_rect.width) -
                              (t.x + t.width)), upbody_masked_rect.height);
            upbody_updatebg_slow(upbody_img, temp, 0.7);
            //rectangle(upbody_img, temp, Scalar(255, 0, 255), 2);
        }
    }
}

void TeacherDetecting::get_upbody(Mat img_upbody )
{
    if(up_update.upbody_bg.empty())
    {
        up_update.frame_num++;
        if( up_update.frame_num == 10)
        {
            img_upbody.copyTo(up_update.upbody_bg);
        }
    }
    if(!up_update.upbody_bg.empty())
    {
        upbody_luv_method(img_upbody);
        upbody_bgupdate(img_upbody);
    }
    if (atoi(cfg_->get_value("debug", "0")) > 0)
    {
        if (!up_update.upbody_bg.empty())
        {
            imshow("bg_upbody", up_update.upbody_bg);
            waitKey(1);
        }
    }
}


bool TeacherDetecting::one_frame_luv(Mat raw_img, Mat img,vector < Rect > &r,
                 vector < Rect > &first_rect)
{
    fillbg_struct.num++;
    bool has_rect = false;

    /*Mat bgmask;
    Mat Img(img_);
    Mat img = Img.clone();*/

    //初始背景;
    if (fillbg_struct.num == 10) {
        //fillbg_struct.bg = img;
         img.copyTo(fillbg_struct.bg);
    }
    //获得背景减除法矩形框;fillbg_struct.rect_old;
    if (!fillbg_struct.bg.empty())
    {
        luv_method(img);
    }

    //原始图像帧差法;
    frame_difference_method(raw_img,frame_s.frame_rect,frame_s.masked_frame_rect);

    //判定人是否走下讲台区;
    if(fillbg_struct.nframe >1 && !fillbg_struct.isfillok_end)
    {
        is_teacher_down(raw_img,img);
    }

    //帧差法动态更新背景;
    if(fillbg_struct.nframe >1)
    {
        frame_updatebg(raw_img,img);
    }

    //开始时没目标时用第一次的(防止开始人不动丢目标) ;
    if (!fillbg_struct.isfillok && fillbg_struct.rect_old.size() <= 0
        &&fillbg_struct.nframe == 2 && !frame_s.is_body_down && !fillbg_struct.body_move)
    {
        Rect t = fillbg_struct.fist_fillrect[0];
        Rect r = Rect(t.x+t.width/2-fillbg_struct.body_width,t.y+t.height/2-fillbg_struct.body_width,
                       2*fillbg_struct.body_width,2*fillbg_struct.body_width);
        r &= Rect(0,0,img.cols,img.rows);
        fillbg_struct.rect_old.push_back(r);
    }

    //判断是否是错误的更新;
    is_need_fillbg_twice(img);

    //LUV算法，初始更新完之后，根据得到得rect和实时的图像以及bg图像，更新bg图;
    if (!fillbg_struct.isfillok)
    {
        fillbg_LUV(img);
    }

    //如果半分钟内一个目标也没有，则更新整个背景区;
    //norect_update_bg(img);

    //调试选项;
    if (atoi(cfg_->get_value("debug", "0")) > 0)
    {
        if (!fillbg_struct.bg.empty())
        {
            imshow("bg", fillbg_struct.bg);
            waitKey(1);
        }
    }
    //两个目标相距一定距离时进行融合;(前面融合的大概是一个目标的距离);
    rect_fusion2( fillbg_struct.rect_old,fillbg_struct.mog2_interval1);
    if(!fillbg_struct.isfillok)
    {
        first_rect = fillbg_struct.fist_fillrect;
    }
    r = fillbg_struct.rect_old;
    if (fillbg_struct.rect_old.size() > 0)
    {
        has_rect = true;
    }
    return has_rect;

}

//**************原始图像进行掩码操作*************************
////填充二值化图像的掩码区 ;
//bool TeacherDetecting::build_mask_internal(const char *key, Mat& img)
//{
//  bool masked = false;
//
//  const char *pts = cfg_->get_value(key, "0");
//  std::vector < Point > points;
//
//  if (pts) {
//      char *data = strdup(pts);
//      char *p = strtok(data, ";");
//      while (p) {
//          // 每个Point 使"x,y" 格式 ;
//          int x, y;
//          if (sscanf(p, "%d,%d", &x, &y) == 2) {
//              CvPoint pt = { x, y };
//              points.push_back(pt);
//          }
//
//          p = strtok(0, ";");
//      }
//      free(data);
//  }
//
//  if (points.size() > 3) {
//      int n = points.size();
//      const Point **pts =
//          (const Point **) alloca(sizeof(const Point *) * points.size());
//      for (int i = 0; i < n; i++) {
//          pts[i] = &points[i];
//      }
//      fillPoly(img, pts, &n, 1, CV_RGB(255, 255, 255));
//      masked = true;
//  }
//
//  return masked;
//}
//
////获得掩码二值化图像（这个应该在初始化时操作） ;
//Mat TeacherDetecting::build_mask(const char *key, const char *key2)
//{
//  /** 从 cfg 中的参数，创建 mask */
//  //CvSize size = {video_width_, video_height_};
//  Size size(video_width_, video_height_);
//  Mat img; img.create(size,CV_8UC3);
//
//  if (!ismask_)
//      img.setTo(0);
//
//  if (key) {
//      ismask_ = build_mask_internal(key, img);
//  }
//
//  if (key2) {
//      build_mask_internal(key2, img);
//  }
//  return img;
//}
//
////输入原始图像，返回掩码后的图像;
//void TeacherDetecting::do_mask(Mat &img)
//{
//  if (ismask_)        //是否填充完毕 ;
//  {
//      bitwise_and(img, img_mask_, img);
//  }
//}
//*************************************************


//****************有效区域进行掩码操作***********************
//************目的：尽量减少图像克隆，节省内存***************
//填充二值化图像的掩码区 ;
bool TeacherDetecting::build_mask_internal(const char *key, Mat& img, Rect mask_rc)
{
    bool masked = false;
    const char *pts = cfg_->get_value(key, "0");
    std::vector < Point > points;
    if (pts) {
        char *data = strdup(pts);
        char *p = strtok(data, ";");
        while (p) {
            // 每个Point 使"x,y" 格式 ;
            int x, y;
            if (sscanf(p, "%d,%d", &x, &y) == 2) {
                CvPoint pt = { x, y };
                pt.x = pt.x-mask_rc.x; pt.y = pt.y - mask_rc.y;//++++++++++++++++;
                points.push_back(pt);
            }

            p = strtok(0, ";");
        }
        free(data);
    }

    if (points.size() > 3) {
        int n = points.size();
        const Point **pts =
            (const Point **) alloca(sizeof(const Point *) * points.size());
        for (int i = 0; i < n; i++) {
            pts[i] = &points[i];
        }
        fillPoly(img, pts, &n, 1, CV_RGB(255, 255, 255));
        masked = true;
    }

    return masked;
}

//获得掩码二值化图像（这个应该在初始化时操作） ;
bool TeacherDetecting::build_mask(Mat &img_mask, Rect mask_rc, const char *key, const char *key2)
{
    /** 从 cfg 中的参数，创建 mask */
    CvSize size = {mask_rc.width, mask_rc.height};
    Mat img; img.create(size,CV_8UC3);

    bool is_mask = false;
    if (!is_mask)
        img.setTo(0);

    if (key) {
        is_mask = build_mask_internal(key, img, mask_rc);
    }

    if (key2) {
        build_mask_internal(key2, img, mask_rc);
    }

    img.copyTo(img_mask);

    return is_mask;
}

//输入原始图像，返回掩码后的图像;
void TeacherDetecting::do_mask(Mat &img, Mat img_mask)
{
    bitwise_and(img, img_mask, img);
}


//获得标定区的所有点;
std::vector<cv::Point> TeacherDetecting::load_roi(const char *pts)
{
    std::vector<cv::Point> points;
    /*if (!pts) {
        points.push_back(cv::Point(0, 0));
        points.push_back(cv::Point(0, atoi(cfg_->get_value("video_width", "960"))));
        points.push_back(cv::Point(atoi(cfg_->get_value("video_width", "960")),
            atoi(cfg_->get_value("video_height", "540"))));
        points.push_back(cv::Point(atoi(cfg_->get_value("video_width", "960")), 0));
        return points;
    }*/

    char key[64];
    if (pts)
    {
        char *data = strdup(pts);
        char *p = strtok(data, ";");
        while (p)
        {
            // 每个Point 使"x,y" 格式;
            int x, y;
            if (sscanf(p, "%d,%d", &x, &y) == 2)
            {
                CvPoint pt = { x, y };
                points.push_back(pt);
            }

            p = strtok(0, ";");
        }
        free(data);
    }

    return points;
}

//从小到大;
int TeacherDetecting::cmp_min_x(const Point & a, const Point & b)
{
    return (a.x < b.x);
}


//从小到大;
int TeacherDetecting::cmp_min_y(const Point & a, const Point & b)
{
    return (a.y < b.y);
}


//获取标定点的外接矩形;
cv::Rect TeacherDetecting::get_point_rect(std::vector<cv::Point> pt)
{
    cv::Rect rect;
    int min_x,min_y;
    int max_x,max_y;
    std::sort(pt.begin(), pt.end(), cmp_min_x);
    min_x = pt[0].x;
    max_x = pt[pt.size()-1].x;
    std::sort(pt.begin(), pt.end(), cmp_min_y);
    min_y = pt[0].y;
    max_y = pt[pt.size()-1].y;
    rect = Rect(min_x,min_y,(max_x-min_x),(max_y-min_y));
    rect &= Rect(0,0,video_width_,video_height_);
    return rect;
}

cv::Rect TeacherDetecting::get_rect(Rect init_rect, const char*cb_date, const char*cb_date_2)
{
    Rect masked;
    std::vector<cv::Point> pt_vector,pt_vector_1,pt_vector_2;
    if(cb_date)
    {
        pt_vector_1 = load_roi(cb_date);
        if(pt_vector_1.size()>2)
        {
            for(int i = 0;i<pt_vector_1.size();i++)
            {
                pt_vector.push_back(pt_vector_1[i]);
            }
        }
    }
    if(cb_date_2)
    {
        pt_vector_2 = load_roi(cb_date_2);
        if(pt_vector_2.size()>2)
        {
            for(int i = 0;i<pt_vector_2.size();i++)
            {
                pt_vector.push_back(pt_vector_2[i]);
            }
        }
    }
    if(pt_vector_1.size() < 3 && pt_vector_2.size() < 3)
    {
        masked = init_rect;
    }
    else
    {
        masked = get_point_rect(pt_vector);
    }
    return masked;
}
