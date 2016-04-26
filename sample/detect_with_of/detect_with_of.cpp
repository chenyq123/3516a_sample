#include <iostream>
#include <unistd.h>
#include "DetectWithOf.h"
#include "sample_comm.h"
#include "hi_comm_ive.h"
#include "hi_ive.h"
#include "mpi_ive.h"
#include "KVConfig.h"
#include "Detect.h"
#include "opencv2/opencv.hpp"
#include "opencv2/legacy/legacy.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#define IMG_WIDTH 960
#define IMG_HEIGHT 540

using namespace std;
using namespace cv;
int getframe_init(int width, int height, int ExtChn);
int getframe(Mat *Img, int ExtChn);

//const VI_CHN_ExtChn = VIU_EXT_CHN_START;

static const char *build_result(const std::vector<cv::Rect> &rcs)
{
    // 构造 json 格式
    std::stringstream ss;
    const int _buf_size = 4096;
    static char *_buf = (char*)malloc(_buf_size);
    const char *_pre = "{ \"stamp\":12345, \"rect\":[";

    strcpy(_buf, _pre);
    bool first = true;
    for (std::vector<cv::Rect>::const_iterator it = rcs.begin(); it != rcs.end(); ++it) {
        if (!first) {
            strcat(_buf, ",");
        }
        else {
            first = false;
        }

        char tmp[128];
        snprintf(tmp, sizeof(tmp), "{\"x\":%d, \"y\":%d, \"width\":%d, \"height\":%d}", it->x, it->y, it->width, it->height);
        strcat(_buf, tmp);
    }
    strcat(_buf, " ]");

    if (true) {
        char tmp[64];
        snprintf(tmp, sizeof(tmp), ", \"flipped_idx\": %d", -1);
        strcat(_buf, tmp);
    }

    strcat(_buf, "}");

    return _buf;
}

int  main()
{
#if 1
    int num = 0;
    KVConfig cfg("student_detect_trace.config");
    getframe_init(IMG_WIDTH, IMG_HEIGHT, VIU_EXT_CHN_START);
    Detect *det = new DetectWithOf(&cfg);
    Mat frame(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, 0);
    frame.create(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
    while(1)
    {
        printf("line=%d,time=%ld,num=%d\n", __LINE__, GetTickCount(),num++);
        getframe(&frame, VIU_EXT_CHN_START);
        std::vector<cv::Rect> standups;
        det->detect(frame, standups);
        const char *result = build_result(standups);
        printf("%s\n", result);
    }
    return 0;
#endif
#if 0
    Mat frame;
    KVConfig cfg("student_detect_trace.config");
    VideoCapture cp;
    cp.open("/app/test.avi");
    if(!cp.isOpened())
    {
        cout << "fail to open!"<<endl;
        return 0;
    }
    Detect *det = new DetectWithOf(&cfg);
    while(1)
    {
        cp >> frame;
        vector<cv::Rect> standups;
        Mat img;
        resize(frame, img, Size(960, 540));
        det->detect(img, standups);
        const char *result = build_result(standups);
        printf("%s\n", result);
    }
#endif
}

int getframe_init(int width, int height, int ExtChn)
{
    VI_CHN Vichn = 0;
    VI_EXT_CHN_ATTR_S stExtChnAttr;
    HI_S32 s32Ret;

    stExtChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    stExtChnAttr.s32BindChn = Vichn;
    stExtChnAttr.stDestSize.u32Width = width;
    stExtChnAttr.stDestSize.u32Height = height;
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

    s32Ret = HI_MPI_VI_EnableChn(ExtChn);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_EnableChn failed with err code %#x\n", s32Ret);
        return -1;
    }
    return 1;
}

int getframe(Mat *Img, int ExtChn)
{
    HI_S32 s32Ret;
    HI_U32 u32BlkSize, u32DstBlkSize;
    VIDEO_FRAME_INFO_S FrameInfo;
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stDst;
    IVE_CSC_CTRL_S stCscCtrl;
    HI_BOOL bInstant = HI_TRUE;
    HI_BOOL bFinish;
    IVE_HANDLE IveHandle;


    stSrc.enType = IVE_IMAGE_TYPE_YUV420SP;
    stDst.enType = IVE_IMAGE_TYPE_U8C3_PACKAGE;
    stCscCtrl.enMode = IVE_CSC_MODE_PIC_BT709_YUV2RGB;

    s32Ret = HI_MPI_VI_GetFrame(ExtChn, &FrameInfo, -1);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_GetFrame failed with err code %#x!\n", s32Ret);
        return -1;
    }

    u32DstBlkSize = FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height * 3;
    s32Ret = HI_MPI_SYS_MmzAlloc_Cached(&stDst.u32PhyAddr[0], (void**)&stDst.pu8VirAddr[0], "user", HI_NULL, u32DstBlkSize);
    if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc_Cached failed with err code %#x!\n", s32Ret);
    }
    HI_MPI_SYS_MmzFlushCache(stDst.u32PhyAddr[0], (void**)stDst.pu8VirAddr[0], u32DstBlkSize);

    stDst.pu8VirAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(stDst.u32PhyAddr[0], u32DstBlkSize);
    stDst.u16Stride[0] = FrameInfo.stVFrame.u32Stride[0];
    stDst.u16Width = FrameInfo.stVFrame.u32Width;
    stDst.u16Height = FrameInfo.stVFrame.u32Height;

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
    if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_CSC failed with error code %#x\n", s32Ret);
    }

    s32Ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
    if (s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Query failed with error code %#x\n", s32Ret);
    }

    memcpy((void*)(Img->data), (void*)stDst.pu8VirAddr[0], u32DstBlkSize);

    HI_MPI_SYS_Munmap(stDst.pu8VirAddr[0], u32DstBlkSize);
    HI_MPI_SYS_Munmap(stSrc.pu8VirAddr[0], u32DstBlkSize / 2);
    HI_MPI_SYS_MmzFree(stDst.u32PhyAddr[0], stDst.pu8VirAddr[0]);
    HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfo);
}


