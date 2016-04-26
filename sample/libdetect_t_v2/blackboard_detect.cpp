#include"blackboard_detect.h"


BlackboardDetecting::BlackboardDetecting(KVConfig *cfg)
    : cfg_(cfg),bg_model(NULL, NULL, true)
{

    N = 4;
    buflen = atof(cfg_->get_value("buflen", "3"));;//设为2：表示连续两帧之间做帧差；设为3：表示隔一帧做帧差(隔帧帧差比较敏感)
    buffer = NULL;
    diff_threshold_three = atof(cfg_->get_value("diff_threshold_three", "17"));
    diff_threshold = atof(cfg_->get_value("diff_threshold", "30"));

    video_width_ = atof(cfg_->get_value("video_width", "960"));
    video_height_ = atof(cfg_->get_value("video_height", "540"));

    merge_interval = atof(cfg_->get_value("bg_merge_interval", "30"));
    min_area = atof(cfg_->get_value("bg_min_area", "3"));
    max_area = atof(cfg_->get_value("bg_max_area", "10000"));
    learning_rate = atof(cfg_->get_value("bg_learning_rate", "0.001"));
    mog_threshold = atof(cfg_->get_value("bg_mog_threshold", "135"));
    bg_model.set("fTau",atof(cfg_->get_value("bg_fTau", "0.1")));//阴影消除参数，0-1之间，默认为0.5，越小，阴影消除越厉害
    bg_model.set("varThreshold",atof(cfg_->get_value("bg_varThreshold", "60")));

    luv_u_max = atoi(cfg_->get_value("bg_luv_u_max", "21"));
    luv_v_max = atoi(cfg_->get_value("bg_luv_v_max", "21"));
    luv_u_min = atoi(cfg_->get_value("bg_luv_u_min", "0"));
    luv_v_min = atoi(cfg_->get_value("bg_luv_v_min", "0"));
    luv_L = atoi(cfg_->get_value("bg_luv_L", "55"));

    ismask_ = false;

    masked_rect = get_rect(cfg_->get_value("calibration_data", "0"),cfg_->get_value("calibration_data_2", "0"));

    const char*cb_date,*cb_date_2;
    if(cfg_->get_value("calibration_data", "0"))
        cb_date = "calibration_data";
    else
        cb_date = NULL;
    if(cfg_->get_value("calibration_data_2", "0"))
        cb_date_2 = "calibration_data_2";
    else
        cb_date_2 = NULL;
    img_mask_ = build_mask(cb_date,cb_date_2);

}

BlackboardDetecting::~BlackboardDetecting()
{
}

//void BlackboardDetecting::creat_buffer(IplImage *image)
//{
//  buffer = (IplImage**)malloc(sizeof(buffer[0])*N);
//  //buffer = new IplImage*[N];
//  for(int i = 0;i<N;i++)
//  {
//      buffer[i]= cvCreateImage(cvSize(image->width,image->height),IPL_DEPTH_8U,1);
//      cvSetZero(buffer[i]);
//  }
//}
////**************************两帧差分法***********************************
////输入：img//当前图像;
////输出：silh//灰度图（当前帧图像和保存的上一帧图像相减后图像）
////buflen设为2：表示连续两帧之间做帧差；设为3：表示隔一帧做帧差;
//void BlackboardDetecting:: two_frame_method(IplImage*img,IplImage*silh)
//{
//  cvCvtColor(img,buffer[buflen-1],CV_BGR2GRAY);
//  cvAbsDiff(buffer[buflen-1],buffer[0],silh);
//  for(int i = 0;i<buflen-1;i++)
//  {
//      cvCopy(buffer[i+1],buffer[i]);
//  }
//  cvThreshold( silh, silh, diff_threshold, 255, CV_THRESH_BINARY );
//  cvSmooth(silh,silh,CV_MEDIAN,3,0,0,0);
//
//  /*IplImage* pyr=cvCreateImage(cvSize((silh->width&-2)/2,(silh->height&-2)/2),IPL_DEPTH_8U,1);
//  cvPyrDown(silh,pyr,7);
//  cvDilate(pyr,pyr,0,1);
//  cvPyrUp(pyr,silh,7);
//  cvReleaseImage(&pyr);*/
//
//  cv::dilate((Mat)silh, (Mat)silh, cv::Mat());
//  cv::erode((Mat)silh,(Mat)silh, cv::Mat());
//  cv::erode((Mat)silh, (Mat)silh, cv::Mat());
//  cv::dilate((Mat)silh, (Mat)silh, cv::Mat(),cv::Point(-1,-1),2);
//}
//
////******************矩形按面积由大到小排序的（比较函数）******************
////输入：Rect a 和 Rect b;
////输出：当a的面积小于b的面积时输出1;
//int BlackboardDetecting::cmp_func_area(const CvRect&a,const CvRect&b)
//{
//
//  return (a.width*a.height)>(b.width*b.height);
//
//}
//
////**************************两矩形框进行融合******************************
////输入：两个矩形a、b;
////返回：两个矩形融合之后的矩形框;
//CvRect BlackboardDetecting::merge_rect(CvRect a,CvRect b)
//{
//  CvRect rect_new;
//  int small_x = a.x;
//  int small_y = a.y;
//  int big_x = a.x+a.width;
//  int big_y = a.y+a.height;
//  if(b.x<small_x)
//      small_x=b.x;
//  if(b.y<small_y)
//      small_y=b.y;
//  if(b.x+b.width>big_x)
//      big_x=b.x+b.width;
//  if(b.y+b.height>big_y)
//      big_y=b.y+b.height;
//  rect_new = cvRect(small_x,small_y,big_x-small_x,big_y-small_y);
//  return rect_new;
//}
//
////*****************************运动区域融合*******************************
////输入：seq:运动矩形序列 ，image:图像;
////输出：seq:融合之后的运动矩形序列;
//void BlackboardDetecting::rect_fusion(std::vector<cv::Rect> &seq,IplImage *image)
//{
//  //对运动框先按面积大小进行排序，由大到小;
//  std::sort(seq.begin(),seq.end(),cmp_func_area);
//  CvRect rect_i;
//  CvRect rect_j;
//  int num = 0;int interval = merge_interval;
//  for(;;)
//  {
//      std::vector<cv::Rect>::iterator it1;
//      std::vector<cv::Rect>::iterator it2;
//      num=0;
//      for(it1 = seq.begin();it1!= seq.end();)
//      {
//          for(it2 = it1+1;it2!= seq.end();)
//          {
//              rect_i = *it1;
//              rect_j = *it2;
//              if(rect_i.x>(rect_j.x+rect_j.width+interval)||(rect_i.x+rect_i.width+interval)<rect_j.x
//                  ||rect_i.y>(rect_j.y+rect_j.height+interval)||(rect_i.y+rect_i.height+interval)<rect_j.y)
//              {
//                  it2++;
//                  continue;
//              }
//              else//当矩形框之间有交集时进行融合;
//              {
//                  *it1 = merge_rect(rect_i,rect_j);
//                  it2 = seq.erase(it2);
//                  num++;
//              }
//          }
//          it1++;
//      }
//      //当所有矩形框之间不可再融合时停止循环;
//      if(num==0)
//      {
//          //重新按面积排序;
//          std::sort(seq.begin(),seq.end(),cmp_func_area);
//          break;
//      }
//
//  }
//
//}
//
////*****************************帧差法*************************************
//bool BlackboardDetecting:: frame_difference_method (IplImage *image,std::vector<cv::Rect> &rect_vector)
//{
//  bool has_rect = false;
//
//  CvMemStorage *siln_contours_storage = NULL;
//  CvSeq *siln_contours_seq=NULL;
//  if(!siln_contours_storage)
//  {
//      siln_contours_storage=cvCreateMemStorage(0);
//      siln_contours_seq=cvCreateSeq(0,sizeof(CvSeq),sizeof(CvContour),siln_contours_storage);
//  }
//  else
//      cvClearMemStorage(siln_contours_storage);
//  CvSize size = cvSize(image->width,image->height);
//  IplImage* silh=NULL;
//  silh=cvCreateImage(size,IPL_DEPTH_8U,1);
//  if(!buffer)
//  {
//      creat_buffer(image);
//  }
//  two_frame_method(image,silh);
//  cvShowImage("帧差法",silh);
//  //查找轮廓;
//  cvFindContours(silh,siln_contours_storage,&siln_contours_seq,sizeof(CvContour),
//      CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0));
//  for(;siln_contours_seq;siln_contours_seq=siln_contours_seq->h_next)
//  {
//      CvRect rect = ((CvContour*)siln_contours_seq)->rect;//子类转换为父类例子;
//      if(rect.x<0||rect.y<0||rect.x+rect.width>image->width||rect.y+rect.height>image->height)
//          continue;
//      if(rect.width*rect.height<100)//太小的矩形框排除;
//          continue;
//      rect_vector.push_back(rect);
//
//  }
//  //对矩形框按照面积从大到小进行排序,并进行融合;
//  if(rect_vector.size()>1)
//  {
//      rect_fusion(rect_vector,image);
//  }
//  cvReleaseMemStorage(&siln_contours_storage);
//  cvReleaseImage(&silh);
//  if(rect_vector.size()>0)
//  {
//      has_rect = true;
//  }
//   return has_rect;
//}





//面积从大到小排序;
int BlackboardDetecting::cmp_area(const Rect & a, const Rect & b)
{
    return (a.width * a.height > b.width * b.height);
}

//两矩形框进行融合;
//输入：两个矩形a、b
//返回：两个矩形融合之后的矩形框;
Rect BlackboardDetecting::sort_rect(Rect a, Rect b)
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



//运动区域融合(矩形框之间间隔小于小于图像宽度的五十分之一时融合为一个框)1
void BlackboardDetecting::rect_fusion2(vector < Rect > &seq, double interval)
{
    //对运动框先按面积大小进行排序，由大到小 ;
    std::sort(seq.begin(), seq.end(), cmp_area);
    Rect rect_i;
    Rect rect_j;
    int num = 0;        //int interval = img.cols*interval_;
    for (;;) {
        std::vector < cv::Rect >::iterator it1;
        std::vector < cv::Rect >::iterator it2;
        num = 0;
        for (it1 = seq.begin(); it1 != seq.end();) {
            for (it2 = it1 + 1; it2 != seq.end();) {
                rect_i = *it1;
                rect_j = *it2;
                if (rect_i.x >
                    (rect_j.x + rect_j.width + interval)
                    || (rect_i.x + rect_i.width + interval) <
                    rect_j.x
                    || rect_i.y >
                    (rect_j.y + rect_j.height + interval)
                    || (rect_i.y + rect_i.height + interval) <
                    rect_j.y) {
                    it2++;
                    continue;
                } else  //当矩形框之间有交集时进行融合 ;
                {
                    *it1 = sort_rect(rect_i, rect_j);
                    it2 = seq.erase(it2);
                    num++;
                }
            }
            it1++;
        }
        //当所有矩形框之间不可再融合时停止循环 ;
        if (num == 0) {
            //重新按面积排序 ;
            std::sort(seq.begin(), seq.end(), cmp_area);
            break;
        }

    }

}


std::vector < Rect > BlackboardDetecting::refineSegments2(Mat img, Mat & mask,
                               Mat & dst,
                               double interval,double minarea,double maxarea)
{
    vector < Rect > rect;
    int niters = 2;
    vector < vector < Point > >contours;
    vector < vector < Point > >contours_temp;
    vector < Vec4i > hierarchy;
    vector < Rect > right_rect;
    Mat temp;
    /*dilate(mask, temp, Mat(), Point(-1,-1), 1);
       erode(temp, temp, Mat(), Point(-1,-1), niters);
       dilate(temp, temp, Mat(), Point(-1,-1), niters); */

    cv::dilate(mask, temp, cv::Mat());
    cv::erode(mask, temp, cv::Mat());
    cv::erode(mask, temp, cv::Mat());
    cv::dilate(mask, temp, cv::Mat(), cv::Point(-1, -1), 2);

    //找出画出超过一定面积的连通区域 ;
    findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL,
             CV_CHAIN_APPROX_SIMPLE);
    dst = Mat::zeros(img.size(), CV_8UC3);
    if (contours.size() > 0) {
        Scalar color(255, 255, 255);
        for (int idx = 0; idx < contours.size(); idx++) {
            const vector < Point > &c = contours[idx];
            Rect t = boundingRect(Mat(c));
            double area = fabs(contourArea(Mat(c)));
            if (area >= min_area && area<=maxarea)
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

//YUV算法获取矩形框序列;
void BlackboardDetecting::luv_method(const Mat &img,Mat bg,std::vector<Rect> &r)
{
    Mat luv_m,luv_m_temp,fgimg;//背景减除;
    luv_m.create(Size(img.cols, img.rows), CV_8UC1);
    luv_m.setTo(0);
    luv_m_temp = img.clone();
    luv_m_temp.setTo(Scalar::all(255));
    Mat img_t; Mat bg_t;
    cvtColor(img, img_t, CV_BGR2YUV);
    cvtColor(bg, bg_t, CV_BGR2YUV);
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
            else if ((U < luv_u_max && V < luv_v_max && U > luv_u_min && V > luv_v_min)&&(L > luv_L))
            {
                luv_m.at < char >(j, i) = 255; luv_m_temp.at < Vec3b >(j, i) = Vec3b(255,0,0);
            }
            else
            {
                luv_m.at < char >(j, i) = 0;
            }

        }
    }

    r=refineSegments2(img, luv_m, fgimg,merge_interval,min_area,max_area);
    if(atoi(cfg_->get_value("debug","0"))>0)
    {
        //imshow("bg_luv_fgimg",fgimg);
        imshow("bg_luv_m_temp",luv_m_temp);
        waitKey(1);
    }

}



bool BlackboardDetecting::one_frame_bd(Mat img, vector < Rect > &r)
{
    bool has_rect = false;
    Mat fgmask;
    bg_model(img, fgmask, learning_rate);   //update_bg_model ? -1 : 0
    Mat bg;//获得的背景图像
    bg_model.getBackgroundImage(bg);
    if (atoi(cfg_->get_value("debug", "0")) > 0)
    {
        imshow("bg_background image", bg);
        waitKey(1);
    }
    luv_method(img,bg,r);
    if(r.size()>0)
    {
        has_rect = true;
    }
    return has_rect;
}


//****************有效区域进行掩码操作***********************
//************目的：尽量减少图像克隆，节省内存***************
//填充二值化图像的掩码区 ;
bool BlackboardDetecting::build_mask_internal(const char *key, Mat& img)
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
                pt.x = pt.x-masked_rect.x; pt.y = pt.y-masked_rect.y;//++++++++++++++++;
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
Mat BlackboardDetecting::build_mask(const char *key, const char *key2)
{
    /** 从 cfg 中的参数，创建 mask */
    CvSize size = {masked_rect.width, masked_rect.height};
    Mat img; img.create(size,CV_8UC3);

    if (!ismask_)
        img.setTo(0);

    if (key) {
        ismask_ = build_mask_internal(key, img);
    }

    if (key2) {
        build_mask_internal(key2, img);
    }
    return img;
}

//输入原始图像，返回掩码后的图像;
void BlackboardDetecting::do_mask(Mat &img)
{
    if (ismask_)        //是否填充完毕 ;
    {
        bitwise_and(img, img_mask_, img);
    }
}

////填充二值化图像的掩码区 ;
//bool BlackboardDetecting::build_mask_internal(const char *key, Mat& img)
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
//Mat BlackboardDetecting::build_mask(const char *key, const char *key2)
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
//void BlackboardDetecting::do_mask(Mat &img)
//{
//  if (ismask_)        //是否填充完毕 ;
//  {
//      bitwise_and(img, img_mask_, img);
//  }
//}

//获得标定区的所有点;
std::vector<cv::Point> BlackboardDetecting::load_roi(const char *pts)
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
int BlackboardDetecting::cmp_min_x(const Point & a, const Point & b)
{
    return (a.x < b.x);
}


//从小到大;
int BlackboardDetecting::cmp_min_y(const Point & a, const Point & b)
{
    return (a.y < b.y);
}


//获取标定点的外接矩形;
cv::Rect BlackboardDetecting::get_point_rect(std::vector<cv::Point> pt)
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


cv::Rect BlackboardDetecting::get_rect(const char*cb_date,const char*cb_date_2)
{
    Rect masked;
    std::vector<cv::Point> pt_vector,pt_vector_1,pt_vector_2;
    if(cb_date)
    {
        pt_vector_1 = load_roi(cb_date);
        if(pt_vector_1.size()>2)
        {
            cv::Rect rect1 = get_point_rect(pt_vector_1);
            masked_rect_vec.push_back(rect1);
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
            cv::Rect rect2 = get_point_rect(pt_vector_2);
            masked_rect_vec.push_back(rect2);
            for(int i = 0;i<pt_vector_2.size();i++)
            {
                pt_vector.push_back(pt_vector_2[i]);
            }
        }
    }
    if(pt_vector_1.size()<3 && pt_vector_2.size()<3)
    {
        masked = Rect(0,0,atoi(cfg_->get_value("video_width", "960")),atoi(cfg_->get_value("video_height", "540")));
    }
    else
    {
        masked = get_point_rect(pt_vector);
    }
    return masked;
}
