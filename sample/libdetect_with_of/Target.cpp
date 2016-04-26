#include "Target.h"
//#include "Target.h"
#include <vector>
#include <iostream>
#include "KVConfig.h"
#include "hi_ive.h"
#include "hi_comm_ive.h"
#include "mpi_ive.h"
#include "hi_comm_sys.h"
#include "mpi_sys.h"
#include "opencv2/opencv.hpp"
#include "hi_opencv.h"
using namespace std;
using namespace cv;

//void calcLKOpticalFlow(cv::Mat pre_gray, cv::Mat cur_gray, vector<Point2f> prepoint, vector<Point2f> &nextpoint, vector<uchar> &state);
//void hi_goodFeaturesToTrack(Mat image, vector<Point2f> corners, int maxCorners, double qulityLevel, double minDistance);

Target::Target()
{
}

Target::~Target()
{
}

bool Target::init(KVConfig *cfg, int id, const cv::Rect &roi, const cv::Mat &curr_gray, double stamp)
{
    first_rc_ = roi;
    outer_.x = 0, outer_.y = 0, outer_.width = curr_gray.cols, outer_.height = curr_gray.rows;
    stamp_ = stamp;
    cfg_ = cfg;
    id_ = id;

    PTS pts;
    printf("goodFeaturesToTrack begin\n");
    cv::goodFeaturesToTrack(curr_gray(roi), pts, 300, 0.01, 1.5);
    //imwrite("save.bmp",curr_gray(roi));
    //hi_goodFeaturesToTrack(curr_gray(roi), pts, 200, 0.01, 10);
    //imwrite("save.bmp",curr_gray(roi));
    printf("goodFeaturesToTrack\n");

    //printf("pts.size:%d\n",pts.size());
    if (pts.size() < 15) {
        return false;
    }

    l2g(pts, roi.tl());

    layers_.push_back(pts);

    brc_ = roi;
    last_rc_ = cv::boundingRect(pts);

    return true;
}

bool Target::track(const cv::Mat &prev, const cv::Mat &curr, double stamp)
{
    // TODO: 应该根据轨迹的方向扩展搜索范围 ...
    // FIXME: 简单的四周扩展 ...
    // 第一次得到的特征点，总有部分不在活动目标上，所以应该在N帧之后，扔掉这些点，让“跟踪点”真正落在目标上 ...

    printf("begin\n");
    int exp = 60;   // 不知道这个距离是否合理 ...
    cv::Rect search_roi = last_rc_;
    search_roi.x -= exp;
    search_roi.y -= exp;
    search_roi.width += 2 * exp;
    search_roi.height += 2 * exp;
    search_roi &= outer_;

    PTS last_pts = layers_.back(), curr_pts;
    g2l(last_pts, search_roi.tl());

    cv::Mat status, err;
    vector<unsigned char> states;
    //cv::calcOpticalFlowPyrLK(prev(search_roi), curr(search_roi), last_pts, curr_pts, status, err);
    printf("calc begin\n");
    //cv::calcOpticalFlowPyrLK(prev(search_roi), curr(search_roi), last_pts, curr_pts, states, err);
    calcLKOpticalFlow(prev(search_roi), curr(search_roi), last_pts, curr_pts, states);
    printf("calc end\n");

    //for (int r = 0; r < status.rows; r++) {
    //    // 标记找错的
    //    if (status.at<uchar>(r, 0) != 1) {
    //        curr_pts[r].x = -10000;
    //    }
    //}
    for(int i = 0; i < states.size(); i++)
    {
        if(states[i] != 1)
        {
            curr_pts[i].x = -10000;
        }
    }

    /// 删除错误点对应的轨迹 ...
    PTS valid_pts;
    printf("line=%d\n",__LINE__);
    for (int i = (int)curr_pts.size() - 1; i >= 0; i--) {
        if (curr_pts[i].x < -5000) {
            remove_path(i);
        }
        else {
            valid_pts.push_back(curr_pts[i]);
        }
    }
    printf("line=%d\n",__LINE__);

    if (valid_pts.size() < 10) {
        return false;
    }

    printf("line=%d\n",__LINE__);
    std::reverse(valid_pts.begin(), valid_pts.end());   // 需要反序 ...

    printf("line=%d\n",__LINE__);
    l2g(valid_pts, search_roi.tl());

    printf("line=%d\n",__LINE__);
    layers_.push_back(valid_pts);
    printf("line=%d\n",__LINE__);
    last_rc_ = cv::boundingRect(valid_pts);
    printf("line=%d\n",__LINE__);
    brc_ |= last_rc_;
    printf("line=%d\n",__LINE__);

    check_paths(stamp);
    printf("line=%d\n",__LINE__);

    return true;
}

bool Target::is_crossed(const cv::Rect &rc) const
{
    return (rc & brc_).area() > 10;
}

bool Target::is_stopped() const
{
    /** 如果path的最后5个距离的平均值小于 2.0，则认为目标停止活动了 ...
     */
    if (layers_.size() <= 5 || layers_[0].size() < 5) {
        return false;
    }

    std::vector<PATH> paths = get_sorted_paths((int)layers_.size() - 6, (int)layers_.size());

    // 统计距离最长的5条路径的最后5帧的距离 ...
    double sum = 0.0;
    for (size_t i = 0; i < 5; i++) {
        sum += distance(paths[i]);
    }
    sum /= 5.0 * 5.0;

    return sum < 2.0;
}

void Target::debug_draw_paths(cv::Mat &rgb, cv::Scalar &color) const
{
    for (int i = 0; i < (int)layers_[0].size(); i++) {
        PATH path = get_path(i);
        if (distance(path) > 5.0) {
            debug_draw_path(rgb, path, color);
        }
    }

    // 画第一层的外接，和最后一层的外接
    if (layers_.size() >= 2) {
        cv::rectangle(rgb, cv::boundingRect(layers_.front()), cv::Scalar(0, 0, 127));
        cv::rectangle(rgb, cv::boundingRect(layers_.back()), cv::Scalar(0, 128, 0));
    }

    char info[64];
    sprintf(info, "%d", id_);
    cv::putText(rgb, info, brc_.br(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(128, 255, 255));
    cv::rectangle(rgb, brc_, cv::Scalar(0, 255, 255));
}

void Target::debug_draw_path(cv::Mat &rgb, const Target::PATH &path, cv::Scalar &color) const
{
    if (path.size() >= 2) {
        cv::circle(rgb, path[0], 2, cv::Scalar(0, 0, 200));
    }

    for (size_t i = 1; i < path.size(); i++) {
        cv::line(rgb, path[i - 1], path[i], color);
    }
}

void Target::check_paths(double stamp)
{
    /** 如果持续时间很久，或者 layers_ 很大时，开始检查
     */

    if (layers_.size() > 30) {
        // XXX: 占用内存很小，也没有必要扔掉 ....
    }
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
#if 0
void hi_CopyDataToIveImageU8C1(Mat Src, IVE_IMAGE_S *pstDst)
{
    if(Src.data == NULL || pstDst->pu8VirAddr == NULL)
        return ;
    for(int i = 0; i < pstDst->u16Height; i++)
    {
        memcpy(pstDst->pu8VirAddr[0] + i * pstDst->u16Stride[0], Src.data + i * Src.step[0], pstDst->u16Width);
    }
}

void hi_CopyDataToMatU8C1(IVE_IMAGE_S *pstSrc, Mat &Dst)
{
    if(Dst.data == NULL || pstSrc->pu8VirAddr == NULL)
        return ;
    for(int i = 0; i < pstSrc->u16Height; i++)
    {
        memcpy(Dst.data + i * Dst.step[0], pstSrc->pu8VirAddr[0] + i * pstSrc->u16Stride[0], pstSrc->u16Width);
    }
}

HI_S32 hi_CreateIveImageU8C1(IVE_IMAGE_S *pstImage, HI_U16 u16width, HI_U16 u16height)
{
    if(u16width % 16 != 0)
        u16width += 16 - u16width % 16;
    if(u16height % 2 != 0)
        u16height += 1;

    u16width = u16width > 64 ? u16width : 64;
    u16height = u16height > 64 ? u16height : 64;

    pstImage->enType = IVE_IMAGE_TYPE_U8C1;
    pstImage->u16Stride[0] = u16width;
    pstImage->u16Width = u16width;
    //pstImage->u16Stride[0] = u16stride;
    //pstImage->u16Width = u16stride;
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

//cv::goodFeaturesToTrack(curr_gray(roi), pts, 300, 0.05, 1.5);
void hi_goodFeaturesToTrack(Mat image, vector<Point2f> corners, int maxCorners, double qulityLevel, double minDistance)
{
    IVE_HANDLE IveHandle;
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stCandiCorner;
    IVE_DST_MEM_INFO_S stCorner;
    IVE_ST_CORNER_CTRL_S stStCornerCtrl;
    IVE_ST_CANDI_CORNER_CTRL_S stStCandiCornerCtrl;
    HI_BOOL bInstant = HI_TRUE;
    HI_S32 ret;
    HI_U16 width, height;
    width = image.cols;
    height = image.rows;
    int cornernum;
    IVE_POINT_U16_S *pastCorner;
    ret = hi_CreateIveImageU8C1(&stSrc, width, height);
    if(ret != HI_SUCCESS)
    {
        printf("hi_CreateIveImageU8C1 with err code %#x\n", ret);
        goto GOOD_END4;
    }

    ret = hi_CreateIveImageU8C1(&stCandiCorner, width, height);
    if(ret != HI_SUCCESS)
    {
        printf("hi_CreateIveImageU8C1 with err code %#x\n", ret);
        goto GOOD_END3;
    }

    hi_CopyDataToIveImageU8C1(image, &stSrc);

    stStCandiCornerCtrl.u0q8QualityLevel = (unsigned int)(qulityLevel * 256);

    stStCandiCornerCtrl.stMem.u32Size = 4 * stSrc.u16Width * stSrc.u16Height + sizeof(IVE_ST_MAX_EIG_S);
    ret = HI_MPI_SYS_MmzAlloc(&stStCandiCornerCtrl.stMem.u32PhyAddr, (void**)&stStCandiCornerCtrl.stMem.pu8VirAddr, "user", HI_NULL, stStCandiCornerCtrl.stMem.u32Size);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
        goto GOOD_END2;
    }

    ret = HI_MPI_IVE_STCandiCorner(&IveHandle, &stSrc, &stCandiCorner, &stStCandiCornerCtrl, bInstant);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_STCandiCorner failed with err code %#x\n",ret);
        goto GOOD_END2;
    }

    stCorner.u32Size = sizeof(IVE_ST_CORNER_INFO_S);
    ret = HI_MPI_SYS_MmzAlloc(&stCorner.u32PhyAddr, (void**)&stCorner.pu8VirAddr, "user", HI_NULL, stCorner.u32Size);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
        goto GOOD_END1;
    }

    stStCornerCtrl.u16MaxCornerNum = maxCorners;
    stStCornerCtrl.u16MinDist = minDistance;

    ret = HI_MPI_IVE_STCorner(&stCandiCorner, &stCorner, &stStCornerCtrl);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_STCorner failed with err code %#x\n",ret);
        goto GOOD_END3;
    }

    cornernum = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->u16CornerNum;
    pastCorner = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->astCorner;
    for(int i = 0; i < cornernum; i++)
    {
        Point2f p;
        p.x = pastCorner[i].u16X;
        p.y = pastCorner[i].u16Y;
        corners.push_back(p);
    }

GOOD_END1:
    ret = HI_MPI_SYS_MmzFree(stCorner.u32PhyAddr, stCorner.pu8VirAddr);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }

GOOD_END2:
    ret = HI_MPI_SYS_MmzFree(stStCandiCornerCtrl.stMem.u32PhyAddr, stStCandiCornerCtrl.stMem.pu8VirAddr);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }

GOOD_END3:
    ret = HI_MPI_SYS_MmzFree(stCandiCorner.u32PhyAddr[0], stCandiCorner.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }

GOOD_END4:
    ret = HI_MPI_SYS_MmzFree(stSrc.u32PhyAddr[0], stSrc.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
    }

}
#endif
