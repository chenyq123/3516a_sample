#include <stdio.h>
#include <string.h>
#include "sample_comm.h"
#include "hi_comm_ive.h"
#include "hi_ive.h"
#include "mpi_ive.h"
#include "detect_main.h"
#include <opencv2/opencv.hpp>


#define WIDTH  480
#define HEIGHT 270

PIC_SIZE_E enSize = PIC_HD1080;
const VI_CHN ExtChn = VIU_EXT_CHN_START;
const VI_CHN ViChn = 0;
VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;

int main(int argc, char const *argv[])
{
    VI_EXT_CHN_ATTR_S stExtChnAttr;
    VB_BLK hBlock;
    VB_POOL hPool = VB_INVALID_POOLID;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32BlkSize, u32DstBlkSize;
    VIDEO_FRAME_INFO_S FrameInfo;
    IVE_HANDLE IveHandle;
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stDst;
    IVE_CSC_CTRL_S stCscCtrl;
    HI_BOOL bInstant = HI_TRUE;
    HI_BOOL bFinish;
    IplImage *iplImage;
    zifImage img;

    int k=1;

    iplImage = cvCreateImageHeader(cvSize(WIDTH,HEIGHT), IPL_DEPTH_8U, 3);

	stExtChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
	stExtChnAttr.s32BindChn = ViChn;
	stExtChnAttr.stDestSize.u32Width = WIDTH;
	stExtChnAttr.stDestSize.u32Height = HEIGHT;
	stExtChnAttr.s32DstFrameRate = -1;
	stExtChnAttr.s32SrcFrameRate = -1;
	stExtChnAttr.enCompressMode = COMPRESS_MODE_NONE;
    HI_MPI_VI_DisableChn(ExtChn);
	s32Ret = HI_MPI_VI_SetExtChnAttr(ExtChn, &stExtChnAttr);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_SetExtChnAttr failed with err code %#x\n", s32Ret);
		return -1;
	}
	s32Ret = HI_MPI_VI_SetFrameDepth(ExtChn, 1);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_SetFrameDepth failed with err code %#x\n", s32Ret);
		return -1;
	}
    stSrc.enType = IVE_IMAGE_TYPE_YUV420SP;
    //stDst.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
    stDst.enType = IVE_IMAGE_TYPE_U8C3_PACKAGE;
    stCscCtrl.enMode = IVE_CSC_MODE_PIC_BT709_YUV2RGB;

#if 1
	s32Ret = HI_MPI_VI_EnableChn(ExtChn);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_EnableChn failed with err code %#x\n", s32Ret);
		return -1;
	}
#endif
    printf("begin\n");
    void *pdet = det_stu_open("student_detect_trace.config");
    while(k)
    {
        s32Ret = HI_MPI_VI_GetFrame(ExtChn, &FrameInfo, -1);
        if(HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_VI_GetFrame failed with err code %#x!\n",s32Ret);
        }
        //printf("get frame!\n");

        u32DstBlkSize = FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height * 3;
        s32Ret = HI_MPI_SYS_MmzAlloc_Cached(&stDst.u32PhyAddr[0],(void**) &stDst.pu8VirAddr[0], "user", HI_NULL, u32DstBlkSize);
        HI_MPI_SYS_MmzFlushCache(stDst.u32PhyAddr[0], (void *)stDst.pu8VirAddr[0], u32DstBlkSize);
#if 0
        stDst.u32PhyAddr[1] = stDst.u32PhyAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;
        stDst.u32PhyAddr[2] = stDst.u32PhyAddr[1] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;

        stDst.pu8VirAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(stDst.u32PhyAddr[0], u32DstBlkSize);
        stDst.pu8VirAddr[1] = stDst.pu8VirAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;
        stDst.pu8VirAddr[2] = stDst.pu8VirAddr[1] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;

        stDst.u16Stride[0] = FrameInfo.stVFrame.u32Stride[0];
        stDst.u16Stride[1] = FrameInfo.stVFrame.u32Stride[0];
        stDst.u16Stride[2] = FrameInfo.stVFrame.u32Stride[0];

        stDst.u16Width = FrameInfo.stVFrame.u32Width;
        stDst.u16Height = FrameInfo.stVFrame.u32Height;
#endif
#if 1
        stDst.pu8VirAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(stDst.u32PhyAddr[0], u32DstBlkSize);
        stDst.u16Stride[0] = FrameInfo.stVFrame.u32Stride[0];
        stDst.u16Width = FrameInfo.stVFrame.u32Width;
        stDst.u16Height = FrameInfo.stVFrame.u32Height;
#endif
        stSrc.u32PhyAddr[0] = FrameInfo.stVFrame.u32PhyAddr[0];
        stSrc.u32PhyAddr[1] = FrameInfo.stVFrame.u32PhyAddr[1];
        stSrc.u32PhyAddr[2] = FrameInfo.stVFrame.u32PhyAddr[2];

        stSrc.pu8VirAddr[0] = (HI_U8*)HI_MPI_SYS_Mmap(FrameInfo.stVFrame.u32PhyAddr[0], u32DstBlkSize / 2);
        stSrc.pu8VirAddr[1] = stSrc.pu8VirAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;
        stSrc.pu8VirAddr[2] = stSrc.pu8VirAddr[1] + 1;


        stSrc.u16Stride[1] = FrameInfo.stVFrame.u32Stride[0];
        stSrc.u16Stride[0] = FrameInfo.stVFrame.u32Stride[0];
        stSrc.u16Stride[2] = 0;

        stSrc.u16Width = FrameInfo.stVFrame.u32Width;
        stSrc.u16Height = FrameInfo.stVFrame.u32Height;

        s32Ret = HI_MPI_IVE_CSC(&IveHandle, &stSrc, &stDst, &stCscCtrl, bInstant);
        if(s32Ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_CSC failed with error code %#x\n",s32Ret);
            continue;
        }
        s32Ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
        if(s32Ret != HI_SUCCESS)
        {
            printf("HI_MPI_IVE_Query failed with error code %#x\n",s32Ret);
            continue;
        }

        img.data[0] = stDst.pu8VirAddr[0];
        img.width = WIDTH;
        img.height = HEIGHT;
        img.stride[0] = stDst.u16Stride[0] * 3;
        //printf("stride:%d\n",iplImage->widthStep);

        //cvSetData(iplImage, stDst.pu8VirAddr[0], iplImage->widthStep);
        //cvSaveImage("saveImage.jpg",iplImage);
        //cv::Mat frame(img.height, img.width, CV_8UC3, stDst.pu8VirAddr[0], img.stride[0]);
        //imwrite("1.jpg",frame);
        printf("=======================================================\n");
        const char *str = det_detect(pdet, &img);
        printf("=======================================================\n");
        //printf("%s\n",str);

        HI_MPI_SYS_Munmap(stDst.pu8VirAddr[0], u32DstBlkSize);
        HI_MPI_SYS_Munmap(stSrc.pu8VirAddr[0], u32DstBlkSize / 2);
        HI_MPI_SYS_MmzFree(stDst.u32PhyAddr[0], stDst.pu8VirAddr[0]);
        HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfo);
    }
    det_stu_close(pdet);
}
