#include "hi_opencv.h"
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sys/time.h>
using namespace std;
#if 1
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
    memset(pstImage->pu8VirAddr[0], 0, pstImage->u16Stride[0] * pstImage->u16Height);
    return ret;
}
#endif

HI_S32 hi_AllocCacheU8C1(IVE_IMAGE_S *pstImage, HI_U16 u16Stride,HI_U16 u16Width, HI_U16 u16Height)
{
    pstImage->enType = IVE_IMAGE_TYPE_U8C1;
    pstImage->u16Stride[0] = u16Stride;
    pstImage->u16Width = u16Width;
    pstImage->u16Height = u16Height;
    //HI_S32 ret = HI_MPI_SYS_MmzAlloc_Cached(&pstImage->u32PhyAddr[0], (void**)&pstImage->pu8VirAddr[0], "user", HI_NULL, u16Stride * u16Height);
    HI_S32 ret = HI_MPI_SYS_MmzAlloc(&pstImage->u32PhyAddr[0], (void**)&pstImage->pu8VirAddr[0], "user", HI_NULL, u16Stride * u16Height);
    return ret;
}

void hi_CopyDataToIveImageU8C1(Mat Src, IVE_IMAGE_S *pstDst)
{
    //printf("Src.cols=%d,Src.rows=%d\n",Src.cols, Src.rows);
    if(Src.data == NULL || pstDst->pu8VirAddr == NULL)
        return ;
    //for(int i = 0; i < pstDst->u16Height; i++)
    for(int i = 0; i < Src.rows; i++)
    {
        //memcpy(pstDst->pu8VirAddr[0] + i * pstDst->u16Stride[0], Src.data + i * Src.step[0], pstDst->u16Width);
        memcpy(pstDst->pu8VirAddr[0] + i * pstDst->u16Stride[0], Src.data + i * Src.step[0], Src.cols);
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

void hi_dilate(Mat Src, Mat &Dst, Mat element, Point anchor, int iterations)
{
    HI_U8 mask3x3[25] = {0, 0,   0,   0,   0,
                         0, 255, 255, 255, 0,
                         0, 255, 255, 255, 0,
                         0, 255, 255, 255, 0,
                         0, 0,   0,   0,   0};
    //HI_U8 mask3x3[25] = {0, 0,     255,   0, 0,
    //                     0, 0,     255,   0, 0,
    //                     255, 255, 255, 255, 255,
    //                     0, 0,     255,   0, 0,
    //                     0, 0,     255,   0, 0};
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stDst;
    IVE_IMAGE_S *pSrc, *pDst;
    IVE_DILATE_CTRL_S stDilateCtrl;
    IVE_HANDLE IveHandle;
    HI_BOOL bInstant = HI_TRUE;
    HI_S32 ret;
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount1());
    memset(&stSrc, 0, sizeof(IVE_SRC_IMAGE_S));
    memset(&stDst, 0, sizeof(IVE_DST_IMAGE_S));
   // printf("line=%d,time=%d\n",__LINE__,GetTickCount1());
    ret = hi_AllocCacheU8C1(&stSrc, Src.step[0], Src.cols, Src.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount1());

    for(int i = 0; i < stSrc.u16Height; i++)
    {
        memcpy(stSrc.pu8VirAddr[0] + i * stSrc.u16Stride[0], Src.data + i * Src.step[0], stSrc.u16Width);
    }
    //printf("line=%d,time=%d\n",__LINE__,GetTickCount1());

    ret = hi_AllocCacheU8C1(&stDst, Dst.step[0], Dst.cols, Dst.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    memcpy(stDilateCtrl.au8Mask, mask3x3, sizeof(HI_U8) * 25);
    pSrc = &stSrc;
    pDst = &stDst;
    for(int i = 0; i < iterations; i++)
    {
        ret = HI_MPI_IVE_Dilate(&IveHandle, pSrc, pDst, &stDilateCtrl, bInstant);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_Dilate with error code %#x\n",ret);
        }
        HI_BOOL bFinish;
        ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_Query with error code %#x\n",ret);
        }
        swap(pSrc, pDst);
    }
    swap(pSrc, pDst);
    for(int i = 0; i < stDst.u16Height; i++)
    {
        memcpy(Dst.data + i * Dst.step[0], pDst->pu8VirAddr[0] + i * pDst->u16Stride[0], pDst->u16Width);
    }

    ret = HI_MPI_SYS_MmzFree(stSrc.u32PhyAddr[0], stSrc.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stDst.u32PhyAddr[0], stDst.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }
}

void hi_erode(Mat Src, Mat &Dst, Mat element, Point anchor, int iterations)
{
    HI_U8 mask3x3[25] = {0, 0,   0,   0,   0,
                         0, 255, 255, 255, 0,
                         0, 255, 255, 255, 0,
                         0, 255, 255, 255, 0,
                         0, 0,   0,   0,   0};
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stDst;
    IVE_IMAGE_S *pSrc, *pDst;
    IVE_ERODE_CTRL_S stErodeCtrl;
    IVE_HANDLE IveHandle;
    HI_BOOL bInstant = HI_TRUE;
    HI_S32 ret;
    memset(&stSrc, 0, sizeof(IVE_SRC_IMAGE_S));
    memset(&stDst, 0, sizeof(IVE_DST_IMAGE_S));
    ret = hi_AllocCacheU8C1(&stSrc, Src.step[0], Src.cols, Src.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    for(int i = 0; i < stSrc.u16Height; i++)
    {
        memcpy(stSrc.pu8VirAddr[0] + i * stSrc.u16Stride[0], Src.data + i * Src.step[0], stSrc.u16Width);
    }

    ret = hi_AllocCacheU8C1(&stDst, Dst.step[0], Dst.cols, Dst.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    memcpy(stErodeCtrl.au8Mask, mask3x3, sizeof(HI_U8) * 25);
    pSrc = &stSrc;
    pDst = &stDst;
    for(int i = 0; i < iterations; i++)
    {
        ret = HI_MPI_IVE_Erode(&IveHandle, pSrc, pDst, &stErodeCtrl, bInstant);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_Erode with error code %#x\n",ret);
        }
        HI_BOOL bFinish;
        ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
        if(ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_Query with error code %#x\n",ret);
        }
        swap(pSrc, pDst);
    }
    swap(pSrc, pDst);
    for(int i = 0; i < stDst.u16Height; i++)
    {
        memcpy(Dst.data + i * Dst.step[0], pDst->pu8VirAddr[0] + i * pDst->u16Stride[0], pDst->u16Width);
    }

    ret = HI_MPI_SYS_MmzFree(stSrc.u32PhyAddr[0], stSrc.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stDst.u32PhyAddr[0], stDst.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }
}
void hi_absdiff(Mat src1, Mat src2, Mat &dst)
{
    IVE_SRC_IMAGE_S stSrc1, stSrc2;
    IVE_DST_IMAGE_S stDst;
    HI_S32 ret;
    IVE_SUB_CTRL_S stSubCtrl;
    IVE_HANDLE IveHandle;

    dst = src1.clone();
    ret = hi_AllocCacheU8C1(&stSrc1, src1.step[0], src1.cols, src1.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    ret = hi_AllocCacheU8C1(&stSrc2, src2.step[0], src2.cols, src2.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    ret = hi_AllocCacheU8C1(&stDst, dst.step[0], dst.cols, dst.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }
    hi_CopyDataToIveImageU8C1(src1, &stSrc1);
    hi_CopyDataToIveImageU8C1(src2, &stSrc2);
    stSubCtrl.enMode = IVE_SUB_MODE_ABS;
    ret = HI_MPI_IVE_Sub(&IveHandle, &stSrc1, &stSrc2, &stDst, &stSubCtrl, HI_TRUE);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Sub with error code %#x\n",ret);
    }
    HI_BOOL bFinish;
    ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Query with error code %#x\n",ret);
    }
    hi_CopyDataToMatU8C1(&stDst, dst);

    ret = HI_MPI_SYS_MmzFree(stSrc1.u32PhyAddr[0], stSrc1.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stSrc2.u32PhyAddr[0], stSrc2.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stDst.u32PhyAddr[0], stDst.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }
}

double hi_threshold(Mat src, Mat &dst, double thresh, double maxval, int type)
{
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stDst;
    HI_S32 ret;
    IVE_THRESH_CTRL_S stThrCtrl;
    IVE_HANDLE IveHandle;
    double result;
    ret = hi_AllocCacheU8C1(&stSrc, src.step[0], src.cols, src.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    ret = hi_AllocCacheU8C1(&stDst, dst.step[0], dst.cols, dst.rows);
    if(ret != HI_SUCCESS)
    {
        printf("hi_AllocCacheU8C1 with error code %#x\n",ret);
    }

    hi_CopyDataToIveImageU8C1(src, &stSrc);
    hi_CopyDataToIveImageU8C1(dst, &stDst);

    memset(&stThrCtrl, 0, sizeof(IVE_THRESH_CTRL_S));

    if(type == THRESH_BINARY)
    {
        stThrCtrl.enMode = IVE_THRESH_MODE_BINARY;
        stThrCtrl.u8LowThr = thresh;
        stThrCtrl.u8MinVal = 0;
        stThrCtrl.u8MaxVal = maxval;
    }
    else if(type == THRESH_BINARY_INV)
    {
        stThrCtrl.enMode = IVE_THRESH_MODE_BINARY;
        stThrCtrl.u8LowThr = thresh;
        stThrCtrl.u8MinVal = maxval;
        stThrCtrl.u8MaxVal = 0;
    }
    else if(type == THRESH_TRUNC)
    {
        stThrCtrl.enMode = IVE_THRESH_MODE_TRUNC;
        stThrCtrl.u8LowThr = thresh;
        stThrCtrl.u8MaxVal = thresh;
    }
    else if(type == THRESH_TOZERO)
    {
        stThrCtrl.enMode = IVE_THRESH_MODE_TO_MINVAL;
        stThrCtrl.u8LowThr = thresh;
        stThrCtrl.u8MinVal = 0;
    }
    else if(type == THRESH_TOZERO_INV)
    {
        stThrCtrl.enMode = IVE_THRESH_MODE_TRUNC;
        stThrCtrl.u8LowThr = thresh;
        stThrCtrl.u8MaxVal = 0;
    }
    ret = HI_MPI_IVE_Thresh(&IveHandle, &stSrc, &stDst, &stThrCtrl, HI_TRUE);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Thresh with error code %#x\n",ret);
    }

    HI_BOOL bFinish;
    ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Query with error code %#x\n",ret);
    }
    hi_CopyDataToMatU8C1(&stDst, dst);

    ret = HI_MPI_SYS_MmzFree(stSrc.u32PhyAddr[0], stSrc.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }

    ret = HI_MPI_SYS_MmzFree(stDst.u32PhyAddr[0], stDst.pu8VirAddr[0]);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzFree with error code %#x\n",ret);
    }

    return result;
}


void calcLKOpticalFlow(cv::Mat pre_gray, cv::Mat cur_gray, vector<Point2f> &prepoint, vector<Point2f> &nextpoint, vector<uchar> &state)
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

    ret = HI_MPI_SYS_MmzAlloc(&stMv.u32PhyAddr, (void**)&stMv.pu8VirAddr, "user", HI_NULL, 200 * sizeof(IVE_MV_S9Q7_S));
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc Cached failed with err code %#x\n",ret);
        return;
    }
    memset(stMv.pu8VirAddr, 0, 200 * sizeof(IVE_MV_S9Q7_S));
    stMv.u32Size = 200 * sizeof(IVE_MV_S9Q7_S);

    for(int i = 2; i >= 0; i--)
    {
        for(int j = 0; j < stLkOptiFlowCtrl.u16CornerNum; j++)
        {
            if(i == 2)
            {
                //((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7X = float_to_s25q7(prepoint[j].x);
                //((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7Y = float_to_s25q7(prepoint[j].y);
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7X = (unsigned int)(prepoint[j].x * 128);
                ((IVE_POINT_S25Q7_S*)stPoint[i].pu8VirAddr)[j].s25q7Y = (unsigned int)(prepoint[j].y * 128);
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
    }

    for(int i = 0; i < stLkOptiFlowCtrl.u16CornerNum; i++)
    {
        cv::Point2f p;
        p.x = (float)(((IVE_POINT_S25Q7_S*)stPoint[2].pu8VirAddr)[i].s25q7X + ((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dx) / (float)128.0;
        p.y = (float)(((IVE_POINT_S25Q7_S*)stPoint[2].pu8VirAddr)[i].s25q7Y + ((IVE_MV_S9Q7_S*)stMv.pu8VirAddr)[i].s9q7Dy) / (float)128.0;

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
void hi_goodFeaturesToTrack(Mat image, vector<Point2f> &corners, int maxCorners, double qulityLevel, double minDistance)
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
    IVE_ST_CORNER_INFO_S *pstCornerInfo;
    HI_BOOL bFinish;
    printf("cols:%d,row:%d\n", width, height);
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
    printf("Level:%x\n",stStCandiCornerCtrl.u0q8QualityLevel);

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

    ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Query with error code %#x\n",ret);
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
    printf("MaxCorner:%x\n",stStCornerCtrl.u16MaxCornerNum);
    printf("Distance:%x\n",stStCornerCtrl.u16MinDist);

    ret = HI_MPI_IVE_STCorner(&stCandiCorner, &stCorner, &stStCornerCtrl);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_STCorner failed with err code %#x\n",ret);
        goto GOOD_END3;
    }

    ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
    if(ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Query with error code %#x\n",ret);
    }

    //cornernum = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->u16CornerNum;
    //pastCorner = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->astCorner;
    pstCornerInfo = (IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr;
    printf("u16CornerNum:%d\n",pstCornerInfo->u16CornerNum);
    for(int i = 0; i < pstCornerInfo->u16CornerNum; i++)
    {
        Point2f p;
        //p.x = pastCorner[i].u16X;
        //p.y = pastCorner[i].u16Y;
        p.x = pstCornerInfo->astCorner[i].u16X;
        p.y = pstCornerInfo->astCorner[i].u16Y;
        if((int)p.x == image.cols || (int)p.y == image.rows)
            continue;
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
