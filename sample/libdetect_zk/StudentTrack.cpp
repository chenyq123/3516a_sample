#include "StudentTrack.h"
//#include "windows.h"
using namespace cv;
using namespace std;
void calcLKOpticalFlow(cv::Mat pre_gray, cv::Mat cur_gray, vector<Point2f> prepoint, vector<Point2f> &nextpoint, vector<uchar> &state);

CStudentTrack::CStudentTrack(void)
{
    frameNum=0;
    ishowtmp=0;
    tvl1 =  cv::createOptFlow_DualTVL1();
    bg_model = new cv::BackgroundSubtractorMOG;

    //flow_gray = cv::Mat::zeros(Size(WIDTH,HEIGHT),CV_8UC1);

    min_student_area=10*10;//16*20;最远的地方，最近的 要乘于1.5
    //最大的点在（400，300）位置，距离<50，最小区域 1.5 距离<100 最小区域为 乘于1.2
    up_student_duration = 30*1000;//30s
    is_debug=false;
    bdebugsaveimage=false;

    MaxRect.x=400;
    MaxRect.y=300;
    MaxRect.width=40;
    MaxRect.height=40;

    MinRect.x=200;
    MinRect.y=100;
    MinRect.width=20;
    MinRect.height=20;

    readconfig("student_detect_trace.config");

    start();


    ////潍坊高密4中
    //set_maskpoint(cv::Point(110,180));
    //set_maskpoint(cv::Point(110,255));
    //set_maskpoint(cv::Point(686,347));
    //set_maskpoint(cv::Point(686,125));
    //set_maskpoint(cv::Point(385,125));

    ////河南
    //set_maskpoint(cv::Point(29,133));
    //set_maskpoint(cv::Point(702,350));
    //set_maskpoint(cv::Point(662,58));
    //set_maskpoint(cv::Point(255,22));


    //640*720
    ////潍坊高密4中
    //set_maskpoint(cv::Point(619,214));
    //set_maskpoint(cv::Point(619,530));
    //set_maskpoint(cv::Point(76,397));
    //set_maskpoint(cv::Point(76,300));
    //set_maskpoint(cv::Point(362,196));

    ////河南
    //set_maskpoint(cv::Point(583,86));
    //set_maskpoint(cv::Point(621,505));
    //set_maskpoint(cv::Point(14,229));
    //set_maskpoint(cv::Point(14,124));
    //set_maskpoint(cv::Point(194,31));

    //中学
    /*set_maskpoint(cv::Point(639,567));
    set_maskpoint(cv::Point(19,229));
    set_maskpoint(cv::Point(19,139));
    set_maskpoint(cv::Point(208,1));
    set_maskpoint(cv::Point(639,0));*/

}

CStudentTrack::~CStudentTrack(void)
{
}

void CStudentTrack::readconfig( char *cfg_name)
{
    if(fileTan.readFile(cfg_name)){
        string ss=fileTan.GetValue("calibration_data");
        string s1,s2,s3;
        while(ss.length()>1){
            int jj=ss.find(";");
            if(jj>0){
                s1=ss.substr(0,jj);
                int kk=s1.find(",");
                if(kk>0){
                    s2=s1.substr(0,kk);
                    s3=s1.substr(kk+1);
                    int x=atoi(s2.c_str());
                    int y=atoi(s3.c_str());
                    set_maskpoint(cv::Point(x,y));
                }
            }
            ss=ss.substr(jj+1);
        }
        if(mask_points.size()>0){
            imageArry.SetMask(mask_points);
        }
    //}

    //if(fileTan.readFile("student_detect_trace.config")){
        ss=fileTan.GetValue("student_detect_max_area");
        //string s1,s2,s3;
        if(ss.length()>1){
            int jj=ss.find(",");
            s1=ss.substr(0,jj);
            int x=atoi(s1.c_str());

            ss=ss.substr(jj+1);
            jj=ss.find(",");
            s2=ss.substr(0,jj);
            int y=atoi(s2.c_str());

            ss=ss.substr(jj+1);
            jj=ss.find(",");
            s1=ss.substr(0,jj);
            int w=atoi(s1.c_str());

            ss=ss.substr(jj+1);
            int h=atoi(ss.c_str());

            MaxRect.x=x;
            MaxRect.y=y;
            MaxRect.width=w;
            MaxRect.height=h;
        }

        ss=fileTan.GetValue("student_detect_min_area");
        //string s1,s2,s3;
        if(ss.length()>1){
            int jj=ss.find(",");
            s1=ss.substr(0,jj);
            int x=atoi(s1.c_str());

            ss=ss.substr(jj+1);
            jj=ss.find(",");
            s2=ss.substr(0,jj);
            int y=atoi(s2.c_str());

            ss=ss.substr(jj+1);
            jj=ss.find(",");
            s1=ss.substr(0,jj);
            int w=atoi(s1.c_str());

            ss=ss.substr(jj+1);
            int h=atoi(ss.c_str());

            MinRect.x=x;
            MinRect.y=y;
            MinRect.width=w;
            MinRect.height=h;
        }
    }
    float ww=((MaxRect.x+MaxRect.width/2-MinRect.x-MinRect.width/2)*(MaxRect.x+MaxRect.width/2-MinRect.x-MinRect.width/2)+(MaxRect.y+MaxRect.height/2-MinRect.y-MinRect.height/2)*(MaxRect.y+MaxRect.height/2-MinRect.y-MinRect.height/2));
    ww=sqrt(ww);
    splitvalue=(MaxRect.width*MaxRect.height-MinRect.width*MinRect.height)/ww;
}

/*程序启动的初始化，在任何时候 调用即重新开始，变量复位*/
void CStudentTrack::start()
{
    frameNum=0;
    ican_head=5;
    ican_head_down=5;
    for(int i=0;i<CONDIDATECOUNT;i++)
    {
        up_student_candidates[i].clear();
        down_student_candidates[i].clear();
        move_student[i].clear();
    }
    up_students.clear();
    up_studentsSave.clear();
}

/*站起光流的待选区域数组变量*/
int CStudentTrack::getNewCondidate()
{
    ican_head=ican_head+1;
    if(ican_head>=CONDIDATECOUNT)ican_head=0;

    return ican_head;
}

int CStudentTrack::getPreCondidate(int prenum)
{
    int i=ican_head-prenum;
    if(i<0)
        i=CONDIDATECOUNT+i;
    return i;
}

/*坐下光流的待选区域数组变量*/
int CStudentTrack::getNewCondidatedown()
{
    ican_head_down=ican_head_down+1;
    if(ican_head_down>=CONDIDATECOUNT)ican_head_down=0;

    return ican_head_down;
}

int CStudentTrack::getPreCondidatedown(int prenum)
{
    int i=ican_head_down-prenum;
    if(i<0)
        i=CONDIDATECOUNT+i;
    return i;
}

/*判断2个区域是否有重叠*/
bool CStudentTrack::haveSameRect(cv::Rect r1,cv::Rect r2)
{
    bool ret=true;
    if(r1.x<=r2.x){
        if(r1.x+r1.width<r2.x)
            ret=false;
    }else{
        if(r2.x+r2.width<r1.x)
            ret=false;
    }

    if(r1.y<=r2.y){
        if(r1.y+r1.height<r2.y)ret=false;
    }
    else{
        if(r2.y+r2.height<r1.y)ret=false;
    }

    return ret;
}

/*获取2个区域的全包围*/
cv::Rect CStudentTrack::getMaxRect(cv::Rect r1,cv::Rect r2)
{
    cv::Rect rr;
    rr.x = min(r1.x,r2.x);
    rr.y = min(r1.y,r2.y);

    rr.width = max(r1.x+r1.width,r2.x+r2.width)-rr.x;
    rr.height = max(r1.y+r1.height,r2.y+r2.height)-rr.y;
    return rr;
}

/*根据区域离 中心点的距离 获取最小区域的面积*/
int CStudentTrack::getMinArea(cv::Rect r)
{
    //16*20;最远的地方，最近的 要乘于1.5
    //最大的点在（400，300）位置，距离<50，最小区域 1.5 距离<100 最小区域为 乘于1.2
    /*int ret=16*20;
    float x=400.0,y=300.0;
    int dis = (int)sqrt((r.x-x)*(r.x-x)+(r.y-y)*(r.y-y));
    if(dis<50)
        ret*=1.5;
    else if(dis<100)
        ret*=1.2;*/
    float ww=((MinRect.x+MinRect.width/2-r.x-r.width/2)*(MinRect.x+MinRect.width/2-r.x-r.width/2)+(MinRect.y+MinRect.height/2-r.y-r.height/2)*(MinRect.y+MinRect.height/2-r.y-r.height/2));
    ww=sqrt(ww);

    int ret = splitvalue*ww+MinRect.width*MinRect.height;

    return ret;
}

/*根据三个连续的小图，判断是否有头部特征 上黑下白 ，而且运动距离 满足upheight个像素  */
bool CStudentTrack::checkHead(cv::Mat img1,cv::Mat img2,cv::Mat img3,int headwidth,int upheight,cv::Mat img4,cv::Mat img5,cv::Mat img6)
{
    static int num=1;
    bool ret=false;
    int w=img1.size().width;
    int h=img1.size().height;
    int step1,step2;
    int pixdis=30;
    //用一个7*7的框，从最后一张图的中间开始往上找，最顶一行到最底一行，平均灰度 要差 超过50个像素
    int hei_find=7;
    int wid_find=7;


    cv::Mat m1 = cv::Mat::zeros(Size(w,h),CV_8UC1);
    cv::Mat m2 = cv::Mat::zeros(Size(w,h),CV_8UC1);
    cv::Mat m3 = cv::Mat::zeros(Size(w,h),CV_8UC1);
    for(int j=0;j<h-3;j++)
    {
        step1 = img1.step*j;
        step2 = img1.step*(j+3);
        uchar *p1 = img1.data+step1;
        uchar *p11 = img1.data+step2;
        uchar *p2 = img2.data+step1;
        uchar *p22 = img2.data+step2;
        uchar *p3 = img3.data+step1;
        uchar *p33 = img3.data+step2;
        uchar *p4 = img4.data+step1;
        uchar *p5 = img5.data+step1;
        uchar *p6 = img6.data+step1;
        for(int i=0;i<w;i++)
        {

            if(*p11 - *p1>pixdis)
            {
                if(!(abs(*p1-*p2)<10 && abs(*p11-*p22)<10))
                {
                    //if( !(abs(*p5-*p6)<10 && abs(*p3-*p4)<10 && abs(*p4-*p5)<10))
                    *(m1.data+step1)=255;
                }
            }

            if(*p22 - *p2>pixdis)
            {
                if(!(abs(*p2-*p3)<10 && abs(*p22-*p33)<10))
                    //if( !(abs(*p1-*p2)<10 && abs(*p4-*p1)<10 && abs(*p4-*p5)<10))
                    *(m2.data+step1)=255;
            }

            if(*p33 - *p3>pixdis)
            {
                if(!(abs(*p2-*p3)<10 && abs(*p22-*p33)<10))
                    //if( !(abs(*p1-*p2)<10 && abs(*p4-*p1)<10 && abs(*p4-*p5)<10))
                    *(m3.data+step1)=255;
            }
            p11++;p1++;
            p22++;p2++;
            p33++;p3++;
            p4++;p5++;p6++;

            step1++;
            step2++;
        }
    }

    ////根据分界线，找上下10*10的部分的灰度图像 来模板匹配，找到




    char str[100];
    //sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_1.bmp",num);
    //cv::imwrite(str, m1);
    //sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_2.bmp",num);
    //cv::imwrite(str, m2);
    //sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_3.bmp",num);
    //cv::imwrite(str, m3);
    //sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_i1.bmp",num);
    //cv::imwrite(str, img1);
    //sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_i2.bmp",num);
    //cv::imwrite(str, img2);
    //sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_i3.bmp",num);
    //cv::imwrite(str, img3);
    /*sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_i4.bmp",num);
    cv::imwrite(str, img4);
    sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_i5.bmp",num);
    cv::imwrite(str, img5);
    sprintf(str,"d:\\lubo2\\tmp3\\7_roi_img%d_i6.bmp",num);
    cv::imwrite(str, img6);*/

    int proY[3][600];
    for(int i=0;i<600;i++)
    {
        proY[0][i]=-1;
        proY[1][i]=-1;
        proY[2][i]=-1;
    }
    for(int i=0;i<w;i++)
    {
        for(int j=0;j<h;j++)
        {
            if(*(m1.data+m1.step*j+i)>0)
            {
                proY[0][i]=j;break;
            }
        }

        for(int j=0;j<h;j++)
        {
            if(*(m2.data+m2.step*j+i)>0)
            {
                proY[1][i]=j;break;
            }
        }

        for(int j=0;j<h;j++)
        {
            if(*(m3.data+m3.step*j+i)>0)
            {
                proY[2][i]=j;break;
            }
        }
    }

    int icount[3],isum[3];
    int minwidth=headwidth;//头部头发到脸的交叉线的最小宽度
    for(int i=0;i<w-minwidth;i++)
    {
        for(int k=0;k<3;k++)
        {
            icount[k]=0;isum[k]=proY[k][i];
            for(int j=i;j<i+minwidth-1;j++)
            {
                if(proY[k][j]<0)break;
                if(abs(proY[k][j]-proY[k][j+1])>2)break;
                icount[k]++;
                isum[k]+=proY[k][i+1];
            }
        }
        if(icount[0]==minwidth-1 && icount[1]==minwidth-1 && icount[2]==minwidth-1){//都找到这样的线
            if(isum[0]>isum[1] && isum[1]>isum[2])//再判断是否按顺序
            {
                if(isum[0]-isum[2]>upheight*minwidth){
                    ret=true;break;
                }
            }
        }
    }

    num++;
    return ret;
}

/*获取3个连续站起区域的光流累计高度*/
int  CStudentTrack::getMaxFlowDis(cv::Rect p0,cv::Rect p1,cv::Rect p2,int i1,int i2,int i3)
{
    int curindex=imageArry.preindex(i1);
    cv::Mat flowup0 = imageArry.imageflowup[curindex];
    int preindex=imageArry.preindex(i2);
    cv::Mat flowup1 = imageArry.imageflowup[preindex];
    int prepreindex=imageArry.preindex(i3);
    cv::Mat flowup2 = imageArry.imageflowup[prepreindex];
    int im0=0,im1=0,im2=0;
    uchar *p ;
    for(int j=p0.y;j<p0.y+p0.height;j++){
        p=flowup0.data+flowup0.step*j+p0.x;
        for(int i=p0.x;i<p0.x+p0.width;i++){
            if(*p>im0)im0=*p;
            p++;
        }
    }
    for(int j=p1.y;j<p1.y+p1.height;j++){
        p=flowup1.data+flowup1.step*j+p1.x;
        for(int i=p1.x;i<p1.x+p1.width;i++){
            if(*p>im1)im1=*p;
            p++;
        }
    }
    for(int j=p2.y;j<p2.y+p2.height;j++){
        p=flowup2.data+flowup2.step*j+p2.x;
        for(int i=p2.x;i<p2.x+p2.width;i++){
            if(*p>im2)im2=*p;
            p++;
        }
    }
    return im0+im1+im2;
}

/*获取3个连续站起区域的光流累计高度*/
int  CStudentTrack::getAvrFlowDis(cv::Rect p0,cv::Rect p1,cv::Rect p2,int i1,int i2,int i3)
{
    int curindex=imageArry.preindex(i1);
    cv::Mat flowup0 = imageArry.imageflowup[curindex];
    int preindex=imageArry.preindex(i2);
    cv::Mat flowup1 = imageArry.imageflowup[preindex];
    int prepreindex=imageArry.preindex(i3);
    cv::Mat flowup2 = imageArry.imageflowup[prepreindex];
    int im0=0,im1=0,im2=0,imcount0=0,imcount1=0,imcount2=0;
    uchar *p ;
    for(int j=p0.y;j<p0.y+p0.height;j++){
        p=flowup0.data+flowup0.step*j+p0.x;
        for(int i=p0.x;i<p0.x+p0.width;i++){
            if(*p>0){
                im0+=*p;
                imcount0++;
            }
            p++;
        }
    }
    for(int j=p1.y;j<p1.y+p1.height;j++){
        p=flowup1.data+flowup1.step*j+p1.x;
        for(int i=p1.x;i<p1.x+p1.width;i++){
            if(*p>0){
                im1+=*p;
                imcount1++;
            }
            p++;
        }
    }
    for(int j=p2.y;j<p2.y+p2.height;j++){
        p=flowup2.data+flowup2.step*j+p2.x;
        for(int i=p2.x;i<p2.x+p2.width;i++){
            if(*p>0){
                im2+=*p;
                imcount2++;
            }
            p++;
        }
    }
    if(imcount0==0)imcount0=1;
    if(imcount1==0)imcount1=1;
    if(imcount2==0)imcount2=1;
    return im0/imcount0+im1/imcount1+im2/imcount2;
}


/*获取1个区域的光流累计最大高度*/
int  CStudentTrack::getMaxFlowDis(cv::Rect p0)
{
    int curindex=imageArry.preindex();
    cv::Mat flowup0 = imageArry.imageflowup[curindex];
    int im0=0,imcount=0;
    uchar *p ;
    for(int j=p0.y;j<p0.y+p0.height;j++){
        p=flowup0.data+flowup0.step*j+p0.x;
        for(int i=p0.x;i<p0.x+p0.width;i++){
            if(*p>im0)im0=*p;
            p++;
        }
    }
    return im0;
}

/*获取1个区域的光流累计平均高度*/
int  CStudentTrack::getAvrFlowDis(cv::Rect p0)
{
    int curindex=imageArry.preindex();
    cv::Mat flowup0 = imageArry.imageflowup[curindex];
    int im0=0,imcount=0;
    uchar *p ;
    for(int j=p0.y;j<p0.y+p0.height;j++){
        p=flowup0.data+flowup0.step*j+p0.x;
        for(int i=p0.x;i<p0.x+p0.width;i++){
            if(*p>0){
                im0+=*p;
                imcount++;
            }
            p++;
        }
    }
    if(imcount==0)
        return 0;
    else
        return im0/imcount;
}


/*屏保区域的坐标设定，必须按顺序*/
void CStudentTrack::set_maskpoint(cv::Point p)
{
    mask_points.push_back(p);
}

/*是否展示调试信息*/
void CStudentTrack::setdebug(bool b)
{
    is_debug=b;
}

/*设置全局参数*/
void CStudentTrack::setParam(int yfirst,int standupareafirst,int widthfirst,int ylast,int standuparealast,int widthlast)
{
    y_firstline = yfirst;
    y_lastline = ylast;
    width_first = widthfirst;
    width_last = widthlast;
    standuparea_first = standupareafirst;
    standuparea_last =standuparealast;
}

/*设置站起的最长时间，超过则取消站起区域*/
void CStudentTrack::setduration(int sec)
{
    up_student_duration = sec*1000;
}

/*展示中间调试结果图*/
void CStudentTrack::showDebug()
{
    if(is_debug)
    {
        //imshow("cur_gray", cur_gray);
        //waitKey(1);

        char str[10];
        sprintf(str,"%d,%d",width_input,height_input);
        cv::putText(flowimg,str,cv::Point(50,imageArry.getHeight()-10),1,1,cv::Scalar(255,255,255));
        rectangle(flowimg, MaxRect, cv::Scalar(100, 100, 100), 2);
        rectangle(flowimg, MinRect, cv::Scalar(100, 100, 100), 2);
        imshow("imgflowimg", flowimg);
        waitKey(1);

        /*imshow("imgdiff", imgdiff);
        waitKey(1);*/

        /*imshow("imgfgmask", fgmask);
        waitKey(1);*/

        //imshow("modelBG", modelBG);
        //waitKey(1);
        /*imshow("imgflowdown", flowdown);
        waitKey(1);*/

        /*imshow("imgflowup", flowupbw);
        waitKey(1);*/

        /*imshow("imgflowgray", flow_gray);
        waitKey(1);*/

        /*imshow("imgdiffforflow", imgdiffforflow);
        waitKey(1);*/

        /*imshow("flowleft", flowleft);
        waitKey(1);

        imshow("flowright", flowright);
        waitKey(1);*/

        /*imshow("flowmove", flowmove);
        waitKey(1);*/
    }
}

/*叠加展示跟踪效果*/
void CStudentTrack::showStudentup()
{
    std::vector<upStudentTarget>::iterator it = up_students.begin();
    while( it != up_students.end())
    {
        if(it->tag==0)
            rectangle(flowimg, it->position, cv::Scalar(0, 0, 255), 1);
        /*else if(it->tag==1)
            rectangle(flowimg, it->position, cv::Scalar(0, 0, 255), 1);
        else if(it->tag==2)
            rectangle(flowimg, it->position, cv::Scalar(0, 255, 0), 1);
        else if(it->tag==3)
            rectangle(flowimg, it->position, cv::Scalar(255,0 , 0), 1);
        else if(it->tag==4)
            rectangle(flowimg, it->position, cv::Scalar(0,255 , 255), 1);*/

        /*if(it->showposcount>0){
            rectangle(flowimg, it->pos0, cv::Scalar(255,255 , 0), 1);
            rectangle(flowimg, it->pos1, cv::Scalar(200,200 , 0), 1);
            rectangle(flowimg, it->pos2, cv::Scalar(0,200 , 255), 1);
        }*/

        ++it;
    }
}


/**/
void CStudentTrack::showStudenttmp()
{
    int icondidates_now=getPreCondidate(0);
    for (int i = 0; i < up_student_candidates[icondidates_now].size(); i++){
        int imax=getAvrFlowDis(up_student_candidates[icondidates_now][i]);//Max
        if(imax>0){
            upStudentTarget us;
            us.position = up_student_candidates[icondidates_now][i];
            us.tickcount=GetTickCount();
            us.tag=0;us.deletecount=0;
            us.showposcount=20;
            us.tag=imax;
            up_studentsTmp.push_back(us);
        }
    }

    std::vector<upStudentTarget>::iterator it = up_studentsTmp.begin();
    while( it != up_studentsTmp.end())
    {
        it->showposcount--;

        if(it->showposcount>0){
            rectangle(fgmask, it->position, cv::Scalar(255, 255, 255), 1);
            char str[10];
            sprintf(str,"%d",it->tag);
            cv::putText(fgmask,str,cv::Point(it->position.x,it->position.y),1,1,cv::Scalar(255,255,255));
            ++it;
        }
        else{
            it = up_studentsTmp.erase(it);
        }
    }
}

// 主过程
void CStudentTrack::process(const cv::Mat& img_color)
{
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    frameNum++;

    tt=GetTickCount();
    t2=0;t3=0;t4=0;t5=0;t6=0;t7=0;t8=0;

    height_input = img_color.size().height;
    width_input = img_color.size().width;

    imageArry.AddNewImage(img_color);

    /*char str[100];
    sprintf(str,"%d",frameNum);
    int curindex111=imageArry.preindex();
    cv::Mat temp111 = imageArry.imagecoo[curindex111];
    cv::putText(temp111,str,cv::Point(10,img_color.size().height-10),1,1,cv::Scalar(255,255,255));
    imshow("imgcolor3", temp111);waitKey(1);*/

    if(frameNum<3) return;


    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    checkDownStudentNormal();//时间到了 自动删除区域
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());


    if(InitAndFlow())//初始化，背景建模和光流法
    {
        //光流法完后，先判断是否有坐下的学生
        //123
        //imwrite("flow.bmp", flowdownbw);
        //Mat flow = flowdownbw.clone();
        //dilate(flowdownbw, flowdownbw, cv::Mat(), cv::Point(-1,-1), 2);
        //dilate(flow, flow, cv::Mat(), cv::Point(-1,-1), 2);
        hi_dilate(flowdownbw, flowdownbw, cv::Mat(), cv::Point(-1,-1), 2);
        //imwrite("flow2.bmp", flowdownbw);
        //hi_dilate(flowdownbw, flowdownbw, cv::Mat(), cv::Point(-1,-1), 3);
        //imwrite("flowopencv.bmp", flow);
        //imwrite("flow2.bmp", flowdownbw);
        //printf("writed\n");

        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
        checkDownStudentByFlow(flowdownbw);//查找坐下的连续区间是否在站起区域内

        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
        checkDownStudentByArea();//再次根据区域的坐下光流面积，来排除站起的区域
        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());

        //判断是否有学生走动离开站起的区域
        //123
        //dilate(flowmove, flowmove, cv::Mat(), cv::Point(-1,-1), 1);
        hi_dilate(flowmove, flowmove, cv::Mat(), cv::Point(-1,-1), 1);
        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
        checkMoveStudentByFlow(flowmove);
        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());

        //跟踪目标
        //找到区域后，判断该区域
        //1231
        //dilate(flowupbw, flowupbw, cv::Mat(), cv::Point(-1,-1), 2);
        hi_dilate(flowupbw, flowupbw, cv::Mat(), cv::Point(-1,-1), 2);

        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
        checkupstudent(flowupbw);
        //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    }
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    //cv::imwrite("d:\\123color.bmp", flowimg);

    /*if(frameNum==300)
    {
        cv::imwrite("d:\\123color.bmp", flowimg);
        cv::imwrite("d:\\123flowup0.bmp", imageArry.imageflowup[curindex]);
        cv::imwrite("d:\\123flowup1.bmp", imageArry.imageflowup[preindex]);
        int ddd=imageArry.preindex(2);
        cv::imwrite("d:\\123flowup2.bmp", imageArry.imageflowup[ddd]);
    }*/

    t7 = GetTickCount()-tt;

    //展示站起来的学生区域
    //showStudentup();
    //imwrite("gray.bmp",cur_gray);

    //showStudenttmp();

    //showDebug();

    //int curindex=imageArry.preindex();
    //imageArry.imagecolorflow[curindex] = flowimg.clone();

    //saveimage();


    t8 = GetTickCount()-tt;

    //char str[100];
    //sprintf(str,"%s",idiffforflowcount);
    //cv::putText(flowimg,str,cv::Point(10,imageArry.getHeight()-10),1,1,cv::Scalar(255,255,255));

    ///////
    /*cv::Mat imgOut;
    resize(img_color, imgOut, Size(480,270));

    std::vector<upStudentTarget>::iterator it = up_students.begin();
    while( it != up_students.end())
    {
        if(it->tag==0){
            int x = 480*it->position.x/320;
            int y = 270*it->position.y/180;
            int w = 480*it->position.width/320;
            int h = 270*it->position.height/180;
            cv::Rect rc;
            rc.x=x;
            rc.y=y;
            rc.width=w;
            rc.height=h;
            rectangle(imgOut, rc, cv::Scalar(0, 0, 255), 1);
        }
        ++it;
    }

    imshow("imgcolorOut", imgOut);
    waitKey(1);*/
}


/*时间到了就删除站起区域*/
void CStudentTrack::checkDownStudentNormal()
{
    std::vector<upStudentTarget>::iterator it = up_students.begin();
    long ttt= GetTickCount();
    while( it != up_students.end())
    {
        if(it->tag>0){//已经要结束的了，延时看看是什么原因结束的
            it->deletecount--;
            if(it->deletecount<=0){//延时时间也够了，删掉
                upStudentTarget us=*it;
                us.tickcount=ttt;
                up_studentsSave.push_back(us);
                it = up_students.erase(it);
            }
            else
                ++it;
        }
        else{//tag==0的情况
            if (tt -  it->tickcount > up_student_duration){//时间到了
                if(it->tag==0){
                    //it->tag=1;it->deletecount=5;//赋值
                    readyDeleteUpstudent(it,1);
                }
            }
            else
                ++it;//未到时间
        }
    }

    ///////
    std::vector<upStudentTarget>::iterator it2 = up_studentsSave.begin();
    while( it2 != up_studentsSave.end())
    {
        if (tt -  it2->tickcount > 1200){//1秒时间到了
                it2 = up_studentsSave.erase(it2);
        }
        else
            ++it2;//未到时间
    }
}

/*检查坐下区域的面积 删除站起区域*/
void CStudentTrack::checkDownStudentByArea()
{
    std::vector<upStudentTarget>::iterator it = up_students.begin();

    int curindex=imageArry.preindex();
    cv::Mat flowdown0 = imageArry.imageflowdown[curindex];
    int preindex = imageArry.preindex(1);
    cv::Mat flowdown1 = imageArry.imageflowdown[preindex];
    int prepreindex = imageArry.preindex(2);
    cv::Mat flowdown2 = imageArry.imageflowdown[prepreindex];
    while( it != up_students.end())
    {
        if(it->tag==0)
        {
            bool student_up = true;
            //判断这个区域，是否有持续3幁 往下的光流，光流数量在1/5
            int y=it->position.y;
            int x=it->position.x;
            int w=it->position.width;
            int h=it->position.height;
            int icount0=0,icount1=0,icount2=0;
            uchar *p0 = flowdown0.data;
            uchar *p1 = flowdown1.data;
            uchar *p2 = flowdown2.data;

            for(int j=y;j<y+h;j++)
            {
                p0 = flowdown0.data+flowdown0.step*j+x;
                for(int i=x;i<x+w;i++)
                {
                    if(*p0++>1)icount0++;
                }
            }
            if(icount0>h*w/5)
            {
                for(int j=y;j<y+h;j++)
                {
                    p1 = flowdown1.data+flowdown1.step*j+x;
                    for(int i=x;i<x+w;i++)
                    {
                        if(*p1++>1)icount1++;
                    }
                }
                //if()
                {
                    for(int j=y;j<y+h;j++)
                    {
                        p2 = flowdown2.data+flowdown2.step*j+x;
                        for(int i=x;i<x+w;i++)
                        {
                            if(*p2++>1)icount2++;
                        }
                    }
                }
            }
            int imin=h*w/5;
            if(icount0>imin && icount1>imin && icount2>imin)//从数量上删除站立区域
            {
                student_up=false;
                //it=up_students.erase(it);
                //it->tag=3;
                //it->deletecount=5;
                readyDeleteUpstudent(it,3);
                ++it;
            }
            //else//还要从运动轨迹判断，连续三幁是从上往下的，而且每幁的范围都需要满足要求（跟站起来一致）
            //{
            //  int y0=-1,y1=-1,y2=-1;
            //  for(int j=y;j<y+h;j++)
            //  {
            //      p0 = flowdown0.data+flowdown0.step*j+x;
            //      for(int i=x;i<x+w;i++)
            //      {
            //          if(*p0++>1){y0=j;break;};
            //      }
            //      if(y0>=0)break;
            //  }
            //  for(int j=y;j<y+h;j++)
            //  {
            //      p1 = flowdown1.data+flowdown1.step*j+x;
            //      for(int i=x;i<x+w;i++)
            //      {
            //          if(*p1++>1){y1=j;break;};
            //      }
            //      if(y1>=0)break;
            //  }
            //  for(int j=y;j<y+h;j++)
            //  {
            //      p2 = flowdown2.data+flowdown2.step*j+x;
            //      for(int i=x;i<x+w;i++)
            //      {
            //          if(*p2++>1){y2=j;break;};
            //      }
            //      if(y2>=0)break;
            //  }
            //  if(y0>=0 && y0<y1 && y1<y2)
            //  {
            //      student_up=false;
            //      it=up_students.erase(it);
            //  }
            //}

            if (student_up)
            {
                ++it;
            }
        }
        else
            ++it;
    }

}

/*检查坐下区域的连续光流 删除站起区域*/
void CStudentTrack::checkDownStudentByFlow(cv::Mat mat)
{
    cv::Mat flowtmp=mat.clone();
    std::vector<std::vector<cv::Point> > contours;
    findContours(flowtmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    int icondidates = getNewCondidatedown();
    down_student_candidates[icondidates].clear();

    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ++it)
    {
        int dis=getMinArea(boundingRect(*it))/2;//坐下的光流区域 减半 来判断
        if (contourArea(*it) > dis)//min_student_area
        {
            down_student_candidates[icondidates].push_back(boundingRect(*it));
            //if (is_debug)
            {
                //rectangle(flowimg, boundingRect(*it), cv::Scalar(255, 0, 0), 2);
            }
        }
    }

    //分析是否在已有的学生站起区域内
    if(up_students.size()>0 )
    {
        int icondidates_now = getPreCondidatedown(0);//当前
        int icondidates_pre1 = getPreCondidatedown(1);//前1幁
        int icondidates_pre2 = getPreCondidatedown(2);//前2幁
        if(down_student_candidates[icondidates_now].size()>0)
        {
            for (int i = 0; i < down_student_candidates[icondidates_now].size(); i++){

                for (int j = 0; j < down_student_candidates[icondidates_pre1].size(); j++){
                    bool bHaveSameRect=haveSameRect(down_student_candidates[icondidates_now][i],down_student_candidates[icondidates_pre1][j]);
                    if(bHaveSameRect && down_student_candidates[icondidates_now][i].y > down_student_candidates[icondidates_pre1][j].y){//站立
                        for (int k = 0; k < down_student_candidates[icondidates_pre2].size(); k++){
                            bool bHaveSameRect2=haveSameRect(down_student_candidates[icondidates_pre1][j],down_student_candidates[icondidates_pre2][k]);
                            if(bHaveSameRect2 && down_student_candidates[icondidates_pre1][j].y > down_student_candidates[icondidates_pre2][k].y){//站立
                                cv::Rect r1=getMaxRect( down_student_candidates[icondidates_now][i],down_student_candidates[icondidates_pre1][j]);
                                cv::Rect r2=getMaxRect( r1,down_student_candidates[icondidates_pre2][k]);

                                //查找相同区域的学生站立区
                                std::vector<upStudentTarget>::iterator it = up_students.begin();
                                while( it != up_students.end())
                                {
                                    if(it->tag==0)
                                    {
                                        bool b=haveSameRect(r2,it->position);
                                        if(b)
                                        {
                                            //it = up_students.erase(it);
                                            //it->tag=2;
                                            //it->deletecount=5;
                                            readyDeleteUpstudent(it,2);
                                            ++it;
                                        }
                                        else
                                            ++it;
                                    }
                                    else
                                        ++it;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/*检查移动区域的连续光流 删除站起区域*/
void CStudentTrack::checkMoveStudentByFlow(cv::Mat mat)
{
    cv::Mat flowtmp=mat.clone();
    std::vector<std::vector<cv::Point> > contours;
    findContours(flowtmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    int icondidates = getPreCondidatedown(0);
    move_student[icondidates].clear();

    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ++it)
    {
        int dis=getMinArea(boundingRect(*it))/2;//移动的光流区域 减半 来判断
        if (contourArea(*it) > dis)//min_student_area
        {
            move_student[icondidates].push_back(boundingRect(*it));
        }
    }

    int pre=5;
    //分析是否在已有的学生站起区域内
    if(up_students.size()>0 )
    {
        std::vector<upStudentTarget>::iterator it = up_students.begin();
        while( it != up_students.end())
        {
            if(it->tag==0)//存在站起区域
            {
                //
                icondidates = getPreCondidatedown(pre);
                std::vector<cv::Rect>::iterator it2 = move_student[icondidates].begin();
                while( it2 != move_student[icondidates].end())
                {
                    bool b=haveSameRect(*it2,it->position);
                    if(b)
                    {
                        cv::Rect rr=*it2;
                        int icount=0;
                        for(int i=pre-1;i>=0;i--){
                            int icon = getPreCondidatedown(i);
                            if(move_student[icondidates].size()>0)
                            {
                                std::vector<cv::Rect>::iterator it3 = move_student[icon].begin();
                                while( it3 != move_student[icon].end())
                                {
                                    bool b2=haveSameRect(*it3,rr);
                                    if(b2)
                                    {
                                        rr=*it3;
                                        icount++;
                                    }
                                    ++it3;
                                }
                            }
                        }
                        if(icount>pre-2){//连续多幁在动
                            bool b3=haveSameRect(rr,it->position);
                            if(b3==false){//离开了 站起的区域
                                readyDeleteUpstudent(it,4);
                            }
                        }
                    }
                    ++it2;
                }
            }
            ++it;
        }

    }
}

/*检查站起区域的连续光流 获取站起区域*/
void CStudentTrack::checkupstudent(cv::Mat mat)
{
    cv::Mat flowtmp=mat.clone();
    std::vector<std::vector<cv::Point> > contours;
    findContours(flowtmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int icondidates = getNewCondidate();//获取新的索引
    up_student_candidates[icondidates].clear();//清空待选的列表

    for (std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ++it)
    {
        int dis=getMinArea(boundingRect(*it));
        if (contourArea(*it) > dis)//min_student_area
        {
            up_student_candidates[icondidates].push_back(boundingRect(*it));
            if (is_debug)
            {
                rectangle(flowimg, boundingRect(*it), cv::Scalar(255, 0, 0), 2);
            }
        }
    }


    //根据连续3幁图像，找到有重叠区域并且 上沿在往上（Y值越来越小），就把这3个图像叠加成一个跟踪目标

    int icondidates_now = getPreCondidate(0);//当前
    int icondidates_pre1 = getPreCondidate(1);//前1幁
    int icondidates_pre2 = getPreCondidate(2);//前2幁
    int icondidates_pre3 = getPreCondidate(3);//前3幁
    bool bFound=false;
    int minWidth=9;//13 是展厅   //9
    int minDisflow=6; //6 是展厅  //3
    if(up_student_candidates[icondidates_now].size()>0)
    {

        for (int i = 0; i < up_student_candidates[icondidates_now].size(); i++){
            // 0,1,2
            bFound=false;
            for (int j = 0; j < up_student_candidates[icondidates_pre1].size(); j++){
                bool bHaveSameRect=haveSameRect(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j]);
                if(bHaveSameRect && up_student_candidates[icondidates_now][i].y <= up_student_candidates[icondidates_pre1][j].y){//站立
                    for (int k = 0; k < up_student_candidates[icondidates_pre2].size(); k++){
                        bool bHaveSameRect2=haveSameRect(up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre2][k]);
                        if(bHaveSameRect2 && up_student_candidates[icondidates_pre1][j].y <= up_student_candidates[icondidates_pre2][k].y){//站立
                            cv::Rect r1=getMaxRect( up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j]);
                            cv::Rect r2=getMaxRect( r1,up_student_candidates[icondidates_pre2][k]);

                            //连续的向上光流 的 点 的累计，而不是区域的累计

                            //增加人脸特征（脸和头发）的判断


                            bool b=checkupOnlyUpflow(r2);//必须是向上的光流比左和右都要多，不然不算

                            if(b && r2.width>minWidth && r2.height>r2.width){//20是展厅的约束
                                //增加已经要取消的站起区域内，不允许马上又站起来，排除学生坐下来后向后仰的情况
                                //判断up_studentsSave里是否有这个区域,而且这个区域必须比原来的站起区域要矮，有则不添加到 站起区域
                                bool b=checkupStudentSave(r2);
                                if(b==false)
                                {
                                    ////增加区域的上升高度，光流偏移量，要满足阈值，阀值要求根据前后不同，而且可以分别设置，中间按比例来算
                                    int dis=getAvrFlowDis(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre2][k]);
                                    int disy=up_student_candidates[icondidates_pre2][k].y-up_student_candidates[icondidates_now][i].y;
                                    if(dis>minDisflow && disy>minDisflow){

                                        //checkupfg(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre2][k],r2);

                                        AddUpstudent(r2,up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre2][k]);//加入到 站立区间里
                                        rectangle(flowimg, r2, cv::Scalar(255, 255, 255), 4);

                                        //bdebugsaveimage=true;
                                        bFound=true;

                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(bFound==false){
                //0,1,3
                for (int j = 0; j < up_student_candidates[icondidates_pre1].size(); j++){
                bool bHaveSameRect=haveSameRect(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j]);
                if(bHaveSameRect && up_student_candidates[icondidates_now][i].y <= up_student_candidates[icondidates_pre1][j].y){//站立
                    for (int k = 0; k < up_student_candidates[icondidates_pre3].size(); k++){
                        bool bHaveSameRect2=haveSameRect(up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre3][k]);
                        if(bHaveSameRect2 && up_student_candidates[icondidates_pre1][j].y <= up_student_candidates[icondidates_pre3][k].y){//站立
                            cv::Rect r1=getMaxRect( up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j]);
                            cv::Rect r2=getMaxRect( r1,up_student_candidates[icondidates_pre3][k]);

                            bool b=checkupOnlyUpflow(r2);//必须是向上的光流比左和右都要多，不然不算

                            //bool

                            if(b && r2.width>minWidth && r2.height>r2.width){//20是展厅的约束
                                //增加已经要取消的站起区域内，不允许马上又站起来，排除学生坐下来后向后仰的情况
                                //判断up_studentsSave里是否有这个区域,而且这个区域必须比原来的站起区域要矮，有则不添加到 站起区域
                                bool b=checkupStudentSave(r2);
                                if(b==false)
                                {
                                    ////增加区域的上升高度，光流偏移量，要满足阈值，阀值要求根据前后不同，而且可以分别设置，中间按比例来算
                                    int dis=getAvrFlowDis(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre3][k],0,1,3);
                                    int disy=up_student_candidates[icondidates_pre3][k].y-up_student_candidates[icondidates_now][i].y;
                                    if(dis>minDisflow && disy>minDisflow){
                                        AddUpstudent(r2,up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre1][j],up_student_candidates[icondidates_pre3][k]);//加入到 站立区间里
                                        rectangle(flowimg, r2, cv::Scalar(255, 255, 255), 4);

                                        //bdebugsaveimage=true;
                                        bFound=true;

                                    }
                                }
                            }
                        }
                    }
                }
            }
            if(bFound==false){
                //0,2,3
                for (int j = 0; j < up_student_candidates[icondidates_pre2].size(); j++){
                    bool bHaveSameRect=haveSameRect(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre2][j]);
                    if(bHaveSameRect && up_student_candidates[icondidates_now][i].y <= up_student_candidates[icondidates_pre2][j].y){//站立
                        for (int k = 0; k < up_student_candidates[icondidates_pre3].size(); k++){
                            bool bHaveSameRect2=haveSameRect(up_student_candidates[icondidates_pre2][j],up_student_candidates[icondidates_pre3][k]);
                            if(bHaveSameRect2 && up_student_candidates[icondidates_pre2][j].y <= up_student_candidates[icondidates_pre3][k].y){//站立
                                cv::Rect r1=getMaxRect( up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre2][j]);
                                cv::Rect r2=getMaxRect( r1,up_student_candidates[icondidates_pre3][k]);

                                bool b=checkupOnlyUpflow(r2);//必须是向上的光流比左和右都要多，不然不算

                                if(b && r2.width>minWidth && r2.height>=r2.width){//20是展厅的约束
                                    //增加已经要取消的站起区域内，不允许马上又站起来，排除学生坐下来后向后仰的情况
                                    //判断up_studentsSave里是否有这个区域,而且这个区域必须比原来的站起区域要矮，有则不添加到 站起区域
                                    bool b=checkupStudentSave(r2);
                                    if(b==false)
                                    {
                                        ////增加区域的上升高度，光流偏移量，要满足阈值，阀值要求根据前后不同，而且可以分别设置，中间按比例来算
                                        int dis=getAvrFlowDis(up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre2][j],up_student_candidates[icondidates_pre3][k],0,2,3);
                                        int disy=up_student_candidates[icondidates_pre3][k].y-up_student_candidates[icondidates_now][i].y;
                                        if(dis>minDisflow && disy>minDisflow){
                                            AddUpstudent(r2,up_student_candidates[icondidates_now][i],up_student_candidates[icondidates_pre2][j],up_student_candidates[icondidates_pre3][k]);//加入到 站立区间里
                                            rectangle(flowimg, r2, cv::Scalar(255, 255, 255), 4);

                                            //bdebugsaveimage=true;
                                            bFound=true;

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            }
        }
    }



}

/*判断参数区域是否在 刚删除的站起区域upStudentSave内*/
bool CStudentTrack::checkupStudentSave(cv::Rect rc)
{
    bool ret=false;
    std::vector<upStudentTarget>::iterator it = up_studentsSave.begin();
    while( it != up_studentsSave.end())
    {
        bool b=haveSameRect(rc,it->position);
        if(b && it->position.y<rc.y)
        {
            //if(it->position.y>rc.y)
                ret=true;break;
        }
        else
            ++it;
    }
    return ret;
}

/*添加确定的站起区域*/
void CStudentTrack::AddUpstudent(cv::Rect rc,cv::Rect p0,cv::Rect p1,cv::Rect p2)
{
    //先把重复区域去掉（如果有的话），再新增这个区域进来
    std::vector<upStudentTarget>::iterator it = up_students.begin();
    while( it != up_students.end())
    {
        bool b=haveSameRect(rc,it->position);
        if(b)
        {
            it = up_students.erase(it);
        }
        else
            ++it;
    }

    upStudentTarget us;
    us.position = rc;us.tickcount=GetTickCount();
    us.tag=0;us.deletecount=0;
    us.pos0=p0;us.pos1=p1;us.pos2=p2;us.showposcount=5;
    up_students.push_back(us);
}

/*准备删除站起区域*/
void CStudentTrack::readyDeleteUpstudent(std::vector<upStudentTarget>::iterator &it,int itag,int ideletecount)
{
    it->tag = itag;
    it->deletecount = ideletecount;
}

/*初始化，背景建模和光流法*/
bool CStudentTrack::InitAndFlow()
{


    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    bool ret=false;
    //cv::Mat imgdiff;//imgdiffforflow;

    //cv::Mat background=cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);

    int curindex=imageArry.preindex();
    cur_color = imageArry.imagecolor[curindex];
    cur_gray = imageArry.imagegray[curindex];

    int preindex = imageArry.preindex(1);
    pre_gray = imageArry.imagegray[preindex];

    t1 = GetTickCount()-tt;

    //图像增强
    //medianBlur(cur_gray,cur_gray,3);
    //blur(cur_gray,cur_gray,Size(3,3),Point(-1,-1));

    flowup = imageArry.imageflowup[curindex]= cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);
    flowdown = imageArry.imageflowdown[curindex]= cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);
    flowleft = imageArry.imageflowleft[curindex] = cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);//二值化
    flowright =  imageArry.imageflowright[curindex] =cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);//二值化

    cv::Mat flowupfrom = imageArry.imageflowupfrom[curindex]= cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);

    flowmove = cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);//学生移动

    t2 = GetTickCount()-tt;

    cur_color.copyTo(flowimg);//copy一份

    //(*bg_model)(cur_gray, imageArry.imagefg[curindex], 0.01);//img_color
    fgmask=imageArry.imagefg[curindex];//赋值一份，用于显示

    //if(frameNum>10)
    //bg_model->getBackgroundImage(modelBG);

    t3 = GetTickCount()-tt;

    //erode(fgmask, fgmask, cv::Mat(), cv::Point(-1,-1), 1);

    //cv::imwrite("d:\\123.jpg", flowimg);

    int disvalue=20;//灵敏度 一般为30
    //先进行前后幁差
    //123
    //printf("pre_gray.step:%d,rows:%d,cols:%d\n",pre_gray.step,pre_gray.rows,pre_gray.cols);
   // Mat imgdiff2;
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    //cv::absdiff(pre_gray, cur_gray,imgdiff);
    hi_absdiff(pre_gray, cur_gray,imgdiff);

    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    //123
    threshold(imgdiff, imgdiff,disvalue, 255, cv::THRESH_BINARY);//30是经验值
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    //erode(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 1);

    //123
    //dilate(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 1);//膨胀一点，用来找光流法的点区域

    //Mat imgdiff2 = imgdiff.clone();
    hi_dilate(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 1);//膨胀一点，用来找光流法的点区域
    //dilate(imgdiff2, imgdiff2, cv::Mat(), cv::Point(-1,-1), 1);//膨胀一点，用来找光流法的点区域
    //imwrite("imgdiff.bmp",imgdiff);
    //imwrite("imgdiffopencv.bmp",imgdiff2);
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());

    //erode(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 1);


    //123
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    //erode(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 2);
    hi_erode(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 2);
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());


    hi_dilate(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 2);
    //dilate(imgdiff, imgdiff, cv::Mat(), cv::Point(-1,-1), 2);
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());

    t4 = GetTickCount()-tt;

    //找运行光流法的点
    imgdiffforflow = cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);

    Point2f p2f;
    vector<Point2f> prepoint,nextpoint;
    vector<uchar> state;
    //vector<int> state;
    vector<float>err;

    int m=0,n=0,idis=0,idis2,idis3,idiffforflowcount=0;
    uchar *pgrayup,*pgrayunder,*pgray=pre_gray.data;
    uchar *p = imgdiff.data;
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());
    for(int j=1;j<imageArry.getHeight()-2;j+=2)
    {
        p = imgdiff.data+imgdiff.step*j;
        pgrayup = pre_gray.data+pre_gray.step*(j-1);
        pgray=pre_gray.data+pre_gray.step*j;
        pgrayunder = pre_gray.data+pre_gray.step*(j+1);
        for(int i=0;i<imageArry.getWidth()-2;i++)
        {
            if(*p++>disvalue)
            {
                m++;

                //idis = abs(*(pgray+i)-*(pgrayup+i)) + abs(*(pgray+i)-*(pgrayunder+i));
                idis = abs(*(pgrayunder+i)-*(pgrayup+i));
                //idis2 = abs(*(pgrayup+i)-*(pgray+i));
                idis3 = abs(*(pgray+i+1)-*(pgray+i));
                if(idis>(disvalue-5) || idis3>(disvalue-5))//取上下纹理的地方来做特征点
                {
                    if(j==p2f.y && i==(p2f.x+1))
                        ;
                    else{
                    p2f.x=i;p2f.y=j;
                    prepoint.push_back(p2f);
                    *(imgdiffforflow.data+imgdiffforflow.step*j+i)=255;
                    idiffforflowcount++;
                    }
                }


                //if(j<(imageArry.getHeight()/2))//后半区
                //{
                //  p2f.x=i;p2f.y=j;
                //      prepoint.push_back(p2f);
                //      *(imgdiffforflow.data+imgdiffforflow.step*j+i)=255;idiffforflowcount++;
                //}
                //else//前半区
                //{
                //  if(p2f.y!=j)
                //  {
                //      p2f.x=i;p2f.y=j;
                //      prepoint.push_back(p2f);
                //      *(imgdiffforflow.data+imgdiffforflow.step*j+i)=255;idiffforflowcount++;
                //  }
                //  else if(i!=(p2f.x+1) )//横向抽样，相邻的不要了，降低光流法的特征点数量
                //  {
                //      p2f.x=i;p2f.y=j;
                //      prepoint.push_back(p2f);
                //      *(imgdiffforflow.data+imgdiffforflow.step*j+i)=255;idiffforflowcount++;
                //  }
                //
                //}
            }
        }
    }
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount());

    t5 = GetTickCount()-tt;

    flowcount = idiffforflowcount;

    char str[10];
    sprintf(str,"%d",idiffforflowcount);
    cv::putText(flowimg,str,cv::Point(10,imageArry.getHeight()-10),1,1,cv::Scalar(255,255,255));

    //通过判断差幁区域 是否有连续上升 或 连续下降
    //有则 套用光流法（或者直接判断 灰度值是否相同 来看方向）

    if(idiffforflowcount>=2000)//太多了，不管了，管计算就会变慢
        prepoint.clear();

    flowdownbw = cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);
    flowupbw = cv::Mat::zeros(Size(imageArry.getWidth(),imageArry.getHeight()),CV_8UC1);//二值化

    //printf("prepointnum:%d\n",prepoint.size());
    if(prepoint.size()>0){
        ret=true;
        int sizewidth=10;//稀疏光流法寻找的半径//8
        int sizeheight=10;
        //for(int i = 0; i < prepoint.size(); i++)

        //printf("calcLKOpticalFLow begin,time:%ld\n",GetTickCount());
        //calcOpticalFlowPyrLK(pre_gray,cur_gray,prepoint,nextpoint,state,err,Size(sizewidth,sizeheight),0);//3``
        //printf("calcLKOpticalFLow end,time:%ld\n",GetTickCount());
        //printf("calcLKOpticalFLow begin,time:%ld\n",GetTickCount());
        calcLKOpticalFlow(pre_gray, cur_gray, prepoint, nextpoint, state);
        //printf("calcLKOpticalFLow end,time:%ld\n",GetTickCount());
        t6 = GetTickCount()-tt;

        //for(int i=0;i<state.size();i++)
        int num = 0;
        //printf("state.size:%d,nextpoint.size:%d\n",state.size(), nextpoint.size());
        int first_ = 1;
        for(int i=0;i<state.size();i++)
        {
            if(state[i]!=0 )
            {
                num++;
                if(first_ == 1)
                {
                    first_ = 0;
                    //printf("prepoint[0].x=%f,prepoint[0].y=%f,nextpoint[0].x=%f,nextpoint[0].y=%f\n",prepoint[0].x, prepoint[0].y, nextpoint[0].x, nextpoint[0].y);
                }
                if(abs(nextpoint[i].y-prepoint[i].y)>sizewidth || abs(nextpoint[i].x-prepoint[i].x)>sizewidth ||
                    nextpoint[i].y>imageArry.getHeight()-1 || nextpoint[i].x>imageArry.getWidth()-1 ||
                    nextpoint[i].y<0 || nextpoint[i].x<0 )
                {
                    continue;
                }

                line(flowmove,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,255));

                if(abs(nextpoint[i].y-prepoint[i].y) <= abs(nextpoint[i].x-prepoint[i].x))//左右方向
                {
                    if(nextpoint[i].x-prepoint[i].x>1)//往右走
                    {
                        //line(flowright,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,255));
                        *(flowright.data+flowright.step*int(nextpoint[i].y)+int(nextpoint[i].x))=255;
                        //line(flowimg,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,0,0));//blue line
                    }
                    else if(nextpoint[i].x-prepoint[i].x<-1)//往左走
                    {
                        //line(flowleft,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,255));
                        *(flowleft.data+flowleft.step*int(nextpoint[i].y)+int(nextpoint[i].x))=255;
                        //line(flowimg,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,0));//blue line
                    }
                    //line(flowupbw,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,255));//把光流都合在一起
                    //line(flowimg,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,0,0));//blue line
                    //continue;
                }

                if((nextpoint[i].y-prepoint[i].y)>1 )//red,sitdown 3
                {
                    int di=nextpoint[i].y-prepoint[i].y;
                    //line(flowimg,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(0,0,255));//red line

                    line(flowdownbw,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,255));
                    line(flowdown,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(di,di,di));
                    //imwrite("flowupbw.bmp",flowupbw);
                    //printf("writed\n");

                }
                else if((nextpoint[i].y-prepoint[i].y)<-1 )//green,standup 3
                {
                    int di=(prepoint[i].y-nextpoint[i].y);
                    //line(flowimg,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(0,255,0));//green line

                    line(flowupbw,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(255,255,255));
                    //line(flowup,Point((int)prepoint[i].x,(int)prepoint[i].y),Point((int)nextpoint[i].x,(int)nextpoint[i].y),Scalar(di,di,di));
                    *(flowup.data+flowup.step*int(nextpoint[i].y)+int(nextpoint[i].x))=di;
                    *(flowupfrom.data+flowupfrom.step*int(prepoint[i].y)+int(prepoint[i].x))=di;
                    //imwrite("flowupbw.bmp",flowupbw);
                    //printf("writed\n");
                }
            }

        }
        //printf("statenum:%d\n",num);
        num = 0;
    }
    return ret;
}

bool CStudentTrack::checkupOnlyUpflow(cv::Rect rc)
{
    bool ret=false;
    int idiffcount_down=0,idiffcount_up=0,idiffcount_left=0,idiffcount_right=0;
    int curindex=imageArry.preindex();
    uchar *p_left,*p_right,*p_up,*p_down;
    cv::Mat flowup = imageArry.imageflowup[curindex];
    cv::Mat flowdown = imageArry.imageflowdown[curindex];
    for(int j=rc.y;j<rc.y+rc.height;j++){
        p_up=flowup.data+flowup.step*j+rc.x;
        p_down=flowdown.data+flowdown.step*j+rc.x;
        p_left=flowleft.data+flowleft.step*j+rc.x;
        p_right=flowright.data+flowright.step*j+rc.x;
        for(int i=rc.x;i<rc.x+rc.width;i++){
            if(*p_up++)idiffcount_up++;
            if(*p_down++)idiffcount_down++;
            if(*p_right++)idiffcount_right++;
            if( *p_left++)idiffcount_left++;
        }
    }

    if(idiffcount_up>idiffcount_left && idiffcount_up>idiffcount_right)ret=true;

    return ret;
}

bool CStudentTrack::checkupfg(cv::Rect p0,cv::Rect p1,cv::Rect p2,cv::Rect p)
{
    bool ret=false;
    static int num=1;
    int curindex=imageArry.preindex();
    cv::Mat image0 = imageArry.imagegray[curindex];
    //rectangle(image0, p2, cv::Scalar(255, 255, 255), 1);
    int preindex = imageArry.preindex(1);
    cv::Mat image1 = imageArry.imagegray[preindex];
    //rectangle(image1, p1, cv::Scalar(255, 255, 255), 1);
    int prepreindex = imageArry.preindex(2);
    cv::Mat image2 = imageArry.imagegray[prepreindex];
    //rectangle(image2, p0, cv::Scalar(255, 255, 255), 1);

    int prei = imageArry.preindex(4);
    cv::Mat image4 = imageArry.imagegray[prei];

    prei = imageArry.preindex(5);
    cv::Mat image5 = imageArry.imagegray[prei];

    prei = imageArry.preindex(6);
    cv::Mat image6 = imageArry.imagegray[prei];

    //获取最顶的那个增量区域
    char str[100];

    //第一，找头发跟脸部的分界线
    //第二，连续三幁图像的，分界线 上升的距离 要满足要求

    cv::Mat roi_img2 = image2(p).clone();
    p2.x-=p.x;p2.y-=p.y;
    //rectangle(roi_img2, p2, cv::Scalar(255, 255, 255), 1);
    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_1.bmp",num);
    //cv::imwrite(str, roi_img2);

    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_1big.bmp",num);
    //cv::imwrite(str, image2);


    cv::Mat roi_img1 = image1(p).clone();
    p1.x-=p.x;p1.y-=p.y;
    //rectangle(roi_img1, p1, cv::Scalar(255, 255, 255), 1);
    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_2.bmp",num);
    //cv::imwrite(str, roi_img1);


    cv::Mat roi_img0 = image0(p).clone();
    p0.x-=p.x;p0.y-=p.y;
    //rectangle(roi_img0, p0, cv::Scalar(255, 255, 255), 1);
    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_3.bmp",num);
    //cv::imwrite(str, roi_img0);

    for(int j=2;j<p.height-2;j++)
    {
        for(int i=2;i<p.width-2;i++)
        {
            ;
        }
    }

    cv::Mat roi_img4 = image4(p).clone();
    cv::Mat roi_img5 = image5(p).clone();
    cv::Mat roi_img6 = image6(p).clone();
    checkHead(roi_img2,roi_img1,roi_img0,8,10,roi_img4,roi_img5,roi_img6);

    image0 = imageArry.imageflowupfrom[curindex];
    roi_img0 =  image0(p).clone();
    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_3_1.bmp",num);
   // cv::imwrite(str, roi_img0);
    image0 = imageArry.imageflowup[curindex];
    roi_img0 = image0(p).clone();
    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_3_2.bmp",num);
  //  cv::imwrite(str, roi_img0);

    sprintf(str,"d:\\lubo2\\tmp1\\7_roi_img%d_3_4.bmp",num);
   // cv::imwrite(str, flowupbw);
    num++;



    return ret;
}

void CStudentTrack::saveimage()
{
    if(bdebugsaveimage){
        //bdebugsaveimage=false;
        ishowtmp++;
        int curindex=imageArry.preindex();
        //cv::Mat image0 = imageArry.imagefg[curindex];
        /*int preindex = imageArry.preindex(1);
        cv::Mat image1 = imageArry.imagefg[preindex];
        int prepreindex = imageArry.preindex(2);
        cv::Mat image2 = imageArry.imagefg[prepreindex];*/

        cv::Mat flowup=imageArry.imageflowup[curindex];

        char str[50],str2[50],str3[50],str4[50];
        sprintf(str,"/app/1_%d_flow.bmp",ishowtmp);
        sprintf(str2,"/app/2_%d_gray.bmp",ishowtmp);
        /*sprintf(str3,"d:\\3_%d_diff.bmp",ishowtmp);
        sprintf(str4,"d:\\5_%d_flowup.bmp",ishowtmp);*/

        /*cv::imwrite("d:\\1_1up.bmp", flowupbw);
        cv::imwrite("d:\\1_1right.bmp", flowright);
        cv::imwrite("d:\\1_1left.bmp", flowleft);*/

        //cv::imwrite(str, flowimg);
        //cv::imwrite(str2, cur_gray);
        /*cv::imwrite(str3, imgdiffforflow);
        cv::imwrite(str4, flowup);*/

        //cv::imwrite("d:\\1_1color.bmp", image1);
        //cv::imwrite("d:\\1_2color.bmp", image2);

        //cv::imwrite("d:\\1_flowleft.bmp", flowleft);
        //cv::imwrite("d:\\1_flowright.bmp", flowright);
    }
}

unsigned long GetTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
unsigned int float_to_s25q7(float num)
{
    unsigned int a = 0;
    a |= (int)num << 7;
    a |= (int)((fabsf(num) - (int)fabsf(num)) * 128 ) & 0x7F;
    return a;
}
float s25q7_to_float(unsigned int num)
{
    float a = (int)num >> 7;
    if(a <  0)
        a -= ((int)(num & 0x7F)) / 128.0;
    else
        a += ((int)(num & 0x7F)) / 128.0;
    return a;
}
float s9q7_to_float(unsigned short num)
{
    float a = (short)num >> 7;
    if(a < 0)
        a -= ((short)(num & 0x7F)) / 128.0;
    else
        a += ((short)(num & 0x7F)) / 128.0;
    return a;
}

HI_S32 hi_CreateIveImageU8C1(IVE_IMAGE_S *pstImage, HI_U16 u16width, HI_U16 u16height)
{
    if(u16width % 16 != 0)
        u16width += 16 - u16width % 16;
    if(u16height % 2 != 0)
        u16height += 1;
    pstImage->enType = IVE_IMAGE_TYPE_U8C1;
    pstImage->u16Stride[0] = u16width;
    pstImage->u16Width = u16width;
    pstImage->u16Height = u16height;
    HI_S32 ret = HI_MPI_SYS_MmzAlloc_Cached(&pstImage->u32PhyAddr[0], (void**)&pstImage->pu8VirAddr[0], "user", HI_NULL, u16width * u16height);
    return ret;
}

void calcLKOpticalFlow(cv::Mat pre_gray, cv::Mat cur_gray, vector<Point2f> prepoint, vector<Point2f> &nextpoint, vector<uchar> &state)
{
    //calcOpticalFlowPyrLK(pre_gray,cur_gray,prepoint,nextpoint,state,err,Size(sizewidth,sizeheight),0);//
    //vector<Point2f> prepoint,nextpoint;
    IVE_HANDLE IveHandle;
    IVE_SRC_IMAGE_S stSrcPre[3];
    IVE_SRC_IMAGE_S stSrcCur[3];
    IVE_SRC_MEM_INFO_S stPoint[3];
    IVE_MEM_INFO_S stMv;
    IVE_LK_OPTICAL_FLOW_CTRL_S stLkOptiFlowCtrl;
    HI_BOOL bInstant = HI_TRUE;
    HI_S32  ret;
    HI_S32  s32Index;
    HI_S32  s32DownIndex;
    IVE_POINT_S25Q7_S pointTmp[200];
    int _first = 1;
    HI_U16 width, height;
    width = pre_gray.cols;
    height = pre_gray.rows;
    Mat pyrimidsPre[3];
    Mat pyrimidsCur[3];
    memset(stSrcPre, 0, sizeof(IVE_SRC_IMAGE_S) * 3);
    memset(stSrcCur, 0, sizeof(IVE_SRC_IMAGE_S) * 3);

    pyrimidsCur[2] = cur_gray;
    pyrimidsPre[2] = pre_gray;

    for(s32Index = 0, s32DownIndex = 8; s32Index < 3; s32Index++)
    {
        s32DownIndex /= 2;
        ret = hi_CreateIveImageU8C1(&stSrcPre[s32Index], width / s32DownIndex, height / s32DownIndex);
        if(ret != HI_SUCCESS)
        {
            printf("hi_CreateIveImageU8C1 with err code %#x\n", ret);
            return;
        }
        ret = hi_CreateIveImageU8C1(&stSrcCur[s32Index], width / s32DownIndex, height / s32DownIndex);
        if(ret != HI_SUCCESS)
        {
            printf("hi_CreateIveImageU8C1 with err code %#x\n", ret);
            return;
        }
    }

#if 0
    int relwidth, relheight;
    relwidth = pre_gray.cols < 64 ? 64 : pre_gray.cols;
    relheight = pre_gray.rows < 64 ? 64 : pre_gray.rows;
    if(relwidth % 16 != 0)
        relwidth += 16 - relwidth % 16;
    if(relheight % 2 != 0)
        relheight += 1;
    ret = HI_MPI_SYS_MmzAlloc_Cached(&stSrcPre.u32PhyAddr[0], (void**)&stSrcPre.pu8VirAddr[0], "user", HI_NULL, relwidth * relheight);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
    }

    stSrcPre.enType = IVE_IMAGE_TYPE_U8C1;
    stSrcPre.u16Stride[0] = relwidth;
    stSrcPre.u16Width = relwidth;
    stSrcPre.u16Height = relheight;

    relwidth = cur_gray.cols < 64 ? 64 : cur_gray.cols;
    relheight = cur_gray.rows < 64 ? 64 : cur_gray.rows;
    if(relwidth % 16 != 0)
        relwidth += 16 - relwidth % 16;
    if(relheight % 2 != 0)
        relheight += 1;
    ret = HI_MPI_SYS_MmzAlloc_Cached(&stSrcCur.u32PhyAddr[0], (void**)&stSrcCur.pu8VirAddr[0], "user", HI_NULL, relwidth * relheight);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
    }
    stSrcCur.enType = IVE_IMAGE_TYPE_U8C1;
    stSrcCur.u16Stride[0] = relwidth;
    stSrcCur.u16Width = relwidth;
    stSrcCur.u16Height = relheight;

    memcpy(stSrcPre.pu8VirAddr[0], pre_gray.data, pre_gray.rows * pre_gray.cols);
    memcpy(stSrcCur.pu8VirAddr[0], cur_gray.data, cur_gray.rows * cur_gray.cols);

#endif
    stLkOptiFlowCtrl.u16CornerNum = prepoint.size() > 200 ? 200 : prepoint.size();
    stLkOptiFlowCtrl.u0q8MinEigThr = 100;
    stLkOptiFlowCtrl.u8IterCount = 5;
    stLkOptiFlowCtrl.u0q8Epsilon = 2;
#if 0
    ret = HI_MPI_SYS_MmzAlloc_Cached(&stPoint.u32PhyAddr, (void**)&stPoint.pu8VirAddr, "user", HI_NULL, stLkOptiFlowCtrl.u16CornerNum * sizeof(IVE_POINT_S25Q7_S));
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
        return;
    }
    stPoint.u32Size = stLkOptiFlowCtrl.u16CornerNum * sizeof(IVE_POINT_S25Q7_S);
#endif
    for(s32Index = 0; s32Index < 3; s32Index++)
    {
        ret = HI_MPI_SYS_MmzAlloc_Cached(&(stPoint[s32Index].u32PhyAddr), (void**)&(stPoint[s32Index].pu8VirAddr), "user", HI_NULL, stLkOptiFlowCtrl.u16CornerNum * sizeof(IVE_POINT_S25Q7_S));
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
            return;
        }
        stPoint[s32Index].u32Size = stLkOptiFlowCtrl.u16CornerNum * sizeof(IVE_POINT_S25Q7_S);
    }

    //if(_first == 1)
    //{
    //    _first = 0;
        //ret = HI_MPI_SYS_MmzAlloc_Cached(&stMv.u32PhyAddr, (void**)&stMv.pu8VirAddr, "user", HI_NULL, 200 * sizeof(IVE_MV_S9Q7_S));
        ret = HI_MPI_SYS_MmzAlloc(&stMv.u32PhyAddr, (void**)&stMv.pu8VirAddr, "user", HI_NULL, 200 * sizeof(IVE_MV_S9Q7_S));
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
            return;
        }
        memset(stMv.pu8VirAddr, 0, 200 * sizeof(IVE_MV_S9Q7_S));
        stMv.u32Size = 200 * sizeof(IVE_MV_S9Q7_S);
    //}
    //stMv.u32Size = stLkOptiFlowCtrl.u16CornerNum * sizeof(IVE_MV_S9Q7_S);
    for(int i = 2; i >= 0; i--)
    {
        for(int j = 0; j < stLkOptiFlowCtrl.u16CornerNum; j++)
        {
            if(i == 2)
            {
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7X = float_to_s25q7(prepoint[j].x);
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7Y = float_to_s25q7(prepoint[j].y);
            }
            else if(i == 1)
            {
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7X = (((IVE_POINT_S25Q7_S*)stPoint[i + 1].pu8VirAddr)[j].s25q7X + 1) / 2;
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7Y = (((IVE_POINT_S25Q7_S*)stPoint[i + 1].pu8VirAddr)[j].s25q7Y + 1) / 2;
            }
            else if(i == 0)
            {
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7X = (((IVE_POINT_S25Q7_S*)stPoint[i + 1].pu8VirAddr)[j].s25q7X + 1) / 2;
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7Y = (((IVE_POINT_S25Q7_S*)stPoint[i + 1].pu8VirAddr)[j].s25q7Y + 1) / 2;
            }
            //printf("point:%f,%f\n",(float)((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7X / (float)128.0, (float)((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7Y / (float)128.0);
        }
    }

    for(int i = 2; i >= 1; i--)
    {
        pyrDown(pyrimidsCur[i], pyrimidsCur[i - 1], pyrimidsCur[i - 1].size());
        pyrDown(pyrimidsPre[i], pyrimidsPre[i - 1], pyrimidsPre[i - 1].size());
    }

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < stSrcPre[i].u16Height; j++)
        {
            memcpy(stSrcPre[i].pu8VirAddr[0] + j * stSrcPre[i].u16Stride[0], pyrimidsPre[i].data + j * pyrimidsPre[i].step[0], stSrcPre[i].u16Width);
            memcpy(stSrcCur[i].pu8VirAddr[0] + j * stSrcCur[i].u16Stride[0], pyrimidsCur[i].data + j * pyrimidsCur[i].step[0], stSrcCur[i].u16Width);
        }
    }


    memset(stMv.pu8VirAddr, 0, stLkOptiFlowCtrl.u16CornerNum * sizeof(IVE_MV_S9Q7_S));
    for(int i = 0; i <= 2; i++)
    {
        ret = HI_MPI_IVE_LKOpticalFlow(&IveHandle, &stSrcPre[i], &stSrcCur[i], &stPoint[i], &stMv, &stLkOptiFlowCtrl ,bInstant);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_LKOpticalFlow Cached failed with err code %#x\n",ret);
            return;
        }
        HI_BOOL bFinish;
        ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_Query with error code %#x\n",ret);
        }
       // printf("(%x,%x)\n",((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx,((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy);
       // printf("-------------------------------------------------\n");
    }

    for(int i = 0; i < stLkOptiFlowCtrl.u16CornerNum; i++)
    {
        cv::Point2f p;
        //p.x = prepoint[i].x + s9q7_to_float(((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx);
        //p.y = prepoint[i].y + s9q7_to_float(((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy);
        //printf("(%f,%f)\n",s9q7_to_float(((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx),s9q7_to_float(((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy));
        p.x = (float)(((IVE_POINT_S25Q7_S*)stPoint[2].pu8VirAddr)[i].s25q7X + ((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx) / (float)128.0;
        p.y = (float)(((IVE_POINT_S25Q7_S*)stPoint[2].pu8VirAddr)[i].s25q7Y + ((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy) / (float)128.0;
        //printf("(%f,%f)\n",s9q7_to_float(((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx),s9q7_to_float(((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy));
       // printf("(%x,%x)\n",((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx,((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy);
        int mstate = ((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s32Status;
        nextpoint.push_back(p);
        state.push_back(mstate == 0 ? 1 : 0);
    }

    ret = HI_MPI_SYS_MmzFree(stMv.u32PhyAddr,stMv.pu8VirAddr);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }


//release resource
#if 0
    ret = HI_MPI_SYS_MmzFree(stPoint.u32PhyAddr, stPoint.pu8VirAddr);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stSrcCur.u32PhyAddr[0], stSrcCur.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stSrcPre.u32PhyAddr[0], stSrcPre.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }
#endif
    for(int i = 0; i < 3; i++)
    {
        ret = HI_MPI_SYS_MmzFree(stPoint[i].u32PhyAddr, stPoint[i].pu8VirAddr);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
        }
        ret = HI_MPI_SYS_MmzFree(stSrcCur[i].u32PhyAddr[0], stSrcCur[i].pu8VirAddr[0]);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
        }

        ret = HI_MPI_SYS_MmzFree(stSrcPre[i].u32PhyAddr[0], stSrcPre[i].pu8VirAddr[0]);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
        }
    }
}
