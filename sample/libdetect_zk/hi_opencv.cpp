#include "hi_opencv.h"
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sys/time.h>
using namespace std;


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
