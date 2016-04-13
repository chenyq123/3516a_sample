#include "libdetect_t.h"
//#include "detect_t.h"
//#include "blackboard_detect.h"
//#include "../libimagesource/image_source.h"
//#include <string>
//#include "hog.h"

/*
struct det_t
{
    KVConfig *cfg_;
    TeacherDetecting *detect_;
    BlackboardDetecting *bd_detect_;
    zk_hog_detector *hog_det;
    IplImage *masked_;
    bool t_m;
    bool b_m;
    std::string result_str; // FIXME: 希望足够了;
};
*/

det_t *det_open(const char *cfg_name)
{
    det_t *ctx = new det_t;
    ctx->cfg_ = new KVConfig(cfg_name);

    ctx->t_m = false;
    ctx->b_m = false;
    ctx->s_m = false;

    //const char *method = ctx->cfg_->get_value("BLACKBOARD_OR_TEACHER", "teacher");
    //fprintf(stderr, "======================> method=%s\n", method);

    if (strcmp(cfg_name, "teacher_detect_trace.config") == 0)
    {
        ctx->t_m = true;
        ctx->detect_ = new TeacherDetecting(ctx->cfg_);
        //ctx->hog_det = new zk_hog_detector("data/hog_upperbody.alt", ctx->cfg_, cv::Size(100, 90));
    }
    else if (strcmp(cfg_name, "bd_detect_trace.config") == 0)
    {
        ctx->b_m = true;
        ctx->bd_detect_ = new BlackboardDetecting(ctx->cfg_);
    }
    else if(strcmp(cfg_name, "student_detect_trace.config") == 0)
    {
        ctx->s_m = true;
        ctx->stu_detect_ = new CStudentTrack();
        char filename[200];
        sprintf(filename, cfg_name);
        ctx->stu_detect_->setdebug(false);
        ctx->stu_detect_->setduration(30);
        ctx->stu_detect_->readconfig(filename);
        ctx->stu_detect_->start();
    }

    return ctx;
}


void det_close(det_t *ctx)
{
    delete ctx->cfg_;

    if(ctx->t_m)
    {
        delete ctx->detect_;
        //delete ctx->hog_det;
    }
    else if(ctx->b_m) { delete ctx->bd_detect_; }
    else if(ctx->s_m) {delete ctx->stu_detect_;}

    delete ctx;
}

// 教师探测 ...
// 身高返回单个矩阵 ...
// 教师区，将vector向量转换为json字符串格式 ;
void vector_to_json_t(std::vector < Rect > r, cv::Rect upbody_rect, bool is_upbody, bool is_rect, char *buf)
{
    int offset = 0;
    offset = sprintf(buf, "{\"stamp\":%d,", time(0));

    if(!is_rect)
    {
        offset += sprintf(buf + offset, "\"rect\":[ ],");
    }
    else
    {
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

        offset += sprintf(buf + offset, "],");
    }

    if(!is_upbody)
    {
        offset += sprintf(buf + offset, "\"up_rect\":{\"x\":0,\"y\":0,\"width\":0,\"height\":0}");
    }
    else
    {
        offset += sprintf(buf + offset, "\"up_rect\":");
        offset += sprintf(buf + offset,"{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
                    upbody_rect.x, upbody_rect.y, upbody_rect.width, upbody_rect.height);
    }

    strcat(buf, "}");
}

// 教师探测身高返回vector向量 ...
//void vector_to_json_t(std::vector < Rect > r, std::vector < Rect > upbody_rect,
//            bool is_upbody, bool is_rect, char *buf)
//{
//  int offset = 0;
//  offset = sprintf(buf, "{\"stamp\":%d,", time(0));
//  if (!is_rect) {
//      offset += sprintf(buf + offset, "\"rect\":[ ],");
//  } else {
//      offset += sprintf(buf + offset, "\"rect\":[");
//      for (int i = 0; i < r.size(); i++) {
//          Rect t = r[i];
//          if (i == 0)
//              offset +=
//                  sprintf(buf + offset,
//                      "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                      t.x, t.y, t.width, t.height);
//          else
//              offset +=
//                  sprintf(buf + offset,
//                      ",{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                      t.x, t.y, t.width, t.height);
//      }
//      offset += sprintf(buf + offset, "],");
//  }
//  if (!is_upbody) {
//      offset +=
//          sprintf(buf + offset,
//              "\"up_rect\":[ ]");
//  } else {
//      offset += sprintf(buf + offset, "\"up_rect\":[");
//      for(int j = 0; j < upbody_rect.size(); j++){
//
//          Rect upbody_t = upbody_rect[j];
//          if(j == 0){
//              offset += sprintf(buf + offset,
//                    "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                    upbody_t.x, upbody_t.y,
//                    upbody_t.width, upbody_t.height);
//          }
//          else{
//              offset += sprintf(buf + offset,
//                    ",{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                    upbody_t.x, upbody_t.y,
//                    upbody_t.width, upbody_t.height);
//          }
//      }
//      offset += sprintf(buf + offset, "]");
//  }
//  strcat(buf, "}");
//}


// 板书探测 ...
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

//char *det_detect(det_t * ctx, zifImage * img)
char *det_detect(det_t * ctx, cv::Mat &Img)
{
    char *str = (char*)alloca(BUFSIZE);
    bool isrect = false;
    std::vector < Rect > r; //目标框;
    vector < cv::Rect > first_r;//初始蓝框;
    //Mat Img = cv::Mat(img->height, img->width, CV_8UC3, img->data[0], img->stride[0]);
//    printf("blur begin:%ld\n",GetTickCount());
    //blur(Img,Img,Size(3,3));
//    printf("blur end:%ld\n",GetTickCount());

    //***************************教师探测****************************
    if(ctx->t_m)
    {
#if 0
        Mat upbody_img;
        if(atoi(ctx->cfg_->get_value("t_upbody_detect", "0"))>0)
        {
            upbody_img = Img.clone();
        }
        // 返回的是个引用，若直接处理会影响原始图像 ...
        Mat masked_img_temp = Mat(Img, ctx->detect_->masked_rect);
        Mat masked_img;
        masked_img_temp.copyTo(masked_img);

        if(ctx->detect_->ismask_)
        {
            ctx->detect_->do_mask(masked_img, ctx->detect_->img_mask_);//做完掩码的原始图像;
        }

        //获取标定区外接矩形图像;
        isrect = ctx->detect_->one_frame_luv(Img, masked_img, r, first_r);
        for (int i = 0; i < r.size(); i++)
        {
            cv::Rect box = ctx->detect_->masked_rect;
            r[i].x = r[i].x + box.x;
            r[i].y = r[i].y + box.y;
            r[i] &= cv::Rect(0, 0, Img.cols, Img.rows);
        }

        //*********身高自适应调节*************
        cv::Rect upbody;
        bool is_up_body = false;
        if(atoi(ctx->cfg_->get_value("t_upbody_detect", "0")) > 0)
        {
             // 去大屏 ...
            Mat screen_img = Mat(ctx->detect_->screen_rect.height, ctx->detect_->screen_rect.width, CV_8UC3, Scalar(0,0,0));
            Mat Imgroi = Mat(Img, ctx->detect_->screen_rect);
            screen_img.copyTo(Imgroi);

            // 探测区掩码 ...
            Mat masked_upbody_temp =  Mat(Img, ctx->detect_->upbody_masked_rect);
            Mat masked_upbody;
            masked_upbody_temp.copyTo(masked_upbody);
            if(ctx->detect_->is_up_mask_)
            {
                 ctx->detect_->do_mask(masked_upbody, ctx->detect_->upbody_img_mask_);
            }

            ctx->detect_->get_upbody(masked_upbody);
            // 一些限制条件 ...
            // 1.同时有一个红框和一个上半身矩形框 ...
            // 2.上半身框宽度要小于身体宽度的4倍才认为是正确的 ...
            // 3.当红框和上半身框没有重叠时也认为是错误上半身 ...
            if(r.size() == 1 && ctx->detect_->up_update.upbody_rect.size() == 1)
            {
                cv::Rect upbody_t = ctx->detect_->up_update.upbody_rect[0];
                cv::Rect t = r[0];
                if( upbody_t.width < t.width * 4 &&
                    !(t.x > upbody_t.x + upbody_t.width + ctx->detect_->upbody_masked_rect.x ||
                      t.x + t.width < upbody_t.x + ctx->detect_->upbody_masked_rect.x))
                {
                     // 1、防止到边缘时目标变矮; 2、防止到大屏两侧时目标变矮;
                    double upbody_center = upbody_t.x + upbody_t.width/2;
                    if((upbody_center < ctx->detect_->upbody_masked_rect.width - 15 && upbody_center > 15) &&
                        (upbody_center + ctx->detect_->upbody_masked_rect.x < ctx->detect_->screen_rect.x - 20 ||
                        upbody_center + ctx->detect_->upbody_masked_rect.x > ctx->detect_->screen_rect.x + ctx->detect_->screen_rect.width + 20)
                       )
                    {
                        is_up_body = true;
                        cv::Rect box = ctx->detect_->upbody_masked_rect;
                        upbody = upbody_t;
                        upbody.x = upbody.x + box.x;
                        upbody.y = upbody.y + box.y;
                        upbody &= cv::Rect(0, 0, Img.cols, Img.rows);
                    }
                }
            }

        }
        vector_to_json_t(r, upbody, is_up_body, isrect, str);

        //***************调试****************
        if (atoi(ctx->cfg_->get_value("debug", "0")) > 0)
        {
            for (int i = 0; i<r.size(); i++)
            {
                rectangle(Img, r[i], Scalar(0, 0, 255), 2);
            }

            for (int i = 0; i < first_r.size(); i++)
            {
                cv::Rect box = ctx->detect_->masked_rect;
                first_r[i].x = first_r[i].x + box.x;
                first_r[i].y = first_r[i].y + box.y - 20;
                first_r[i].height = first_r[i].height + box.y + 40;
                first_r[i] &= cv::Rect(0, 0, Img.cols, Img.rows);
                rectangle(Img, first_r[i], Scalar(255, 0, 0), 2);
            }

            rectangle(Img, upbody, Scalar(255, 255, 255), 2);

            imshow("rawimage", Img);
            waitKey(1);
        }
#endif
        //Mat upbody_img;
        //if(atoi(ctx->cfg_->get_value("t_upbody_detect", "0"))>0)
        //{
        //    upbody_img = Img.clone();
        //}
        // 返回的是个引用，若直接处理会影响原始图像 ...
        Mat masked_img_temp = Mat(Img, ctx->detect_->masked_rect);
        Mat masked_img;
        masked_img_temp.copyTo(masked_img);

        //if(ctx->detect_->ismask_)
        //{
        //    ctx->detect_->do_mask(masked_img, ctx->detect_->img_mask_);//做完掩码的原始图像;
        //}
        ctx->detect_->do_mask(masked_img);
        //获取标定区外接矩形图像;
        isrect = ctx->detect_->one_frame_luv(Img, masked_img, r, first_r);
        for (int i = 0; i < r.size(); i++)
        {
            cv::Rect box = ctx->detect_->masked_rect;
            r[i].x = r[i].x + box.x;
            r[i].y = r[i].y + box.y;
            r[i] &= cv::Rect(0, 0, Img.cols, Img.rows);
        }

        cv::Rect upbody;
        bool is_up_body = false;
        if (atoi(ctx->cfg_->get_value("t_upbody_detect", "0")) > 0)
        {
            Mat masked_upbody = Mat(Img, ctx->detect_->upbody_masked_rect);
            ctx->detect_->get_upbody(masked_upbody);
            if (ctx->detect_->up_update.upbody_rect.size() > 0)
            {
                cv::Rect upbody_t = ctx->detect_->up_update.upbody_rect[0];
                if (!(upbody_t.x + upbody_t.width / 2 > ctx->detect_->upbody_masked_rect.width - 15 || upbody_t.x + upbody_t.width / 2 < 15))
                {
                    is_up_body = true;
                    cv::Rect box = ctx->detect_->upbody_masked_rect;
                    upbody = upbody_t;
                    upbody.x = upbody.x + box.x;
                    upbody.y = upbody.y + box.y;
                    upbody &= cv::Rect(0, 0, Img.cols, Img.rows);
                }
            }
        }
        vector_to_json_t(r, upbody, is_up_body, isrect, str);

    }

    //***************************板书探测****************************
    else if(ctx->b_m)
    {
        //做掩码的原始图像;
        Mat masked_img = Mat(Img,ctx->bd_detect_->masked_rect);
        ctx->bd_detect_->do_mask(masked_img);

        //直接获取标定区外接矩形图像;
        //ctx->bd_detect_->masked_rect &= Rect(0,0,img_t.cols, img_t.rows);
        //isrect = ctx->bd_detect_->one_frame_bd((IplImage*)&masked_img, r);
        isrect = ctx->bd_detect_->one_frame_bd(masked_img, r);
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
    else if(ctx->s_m)
    {
        const char *_pre = "{\"stamp\":1234567,\"rect\":[";
        printf("process begin:%ld\n",GetTickCount());
        ctx->stu_detect_->process(Img);
        printf("process end:%ld\n",GetTickCount());
        strcpy(str, _pre);
        bool first = true;
        std::vector<upStudentTarget>::iterator it = ctx->stu_detect_->up_students.begin();
        while(it != ctx->stu_detect_->up_students.end())
        {
            if(it->tag == 0)
            {
                if(!first)
                {
                    strcat(str, ",");
                }
                char tmp[128];
                int x = 480 * it->position.x / 480;
                int y = 270 * it->position.y / 360;
                int w = 480 * it->position.width / 480;
                int h = 270 * it->position.height / 360;
                sprintf(tmp, "{\"x\":%d,\"y\":%d, \"width\":%d, \"height\":%d}", x, y, w, h);
                strcat(str, tmp);
                first = false;
            }
            ++it;
        }
        strcat(str, "]}");
    }

    ctx->result_str = str;
    //return (char*)ctx->result_str.c_str( );
    return str;
}


