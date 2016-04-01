#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "hi_common.h"
#include "hi_comm_video.h"
#include "hi_comm_sys.h"
#include "mpi_sys.h"
#include "hi_comm_vb.h"
#include "mpi_vb.h"
#include "hi_comm_vo.h"
#include "mpi_vo.h"
#include "mpi_vgs.h"

#define MAX_FRM_CNT 256

static HI_S32 s_s32MemDev = 0;

typedef struct hiDUMP_MEMBUF_S
{
    VB_BLK  hBlock;
    VB_POOL hPool;
    HI_U32  u32PoolId;

    HI_U32  u32PhyAddr;
    HI_U8*   pVirAddr;
    HI_S32  s32Mdev;
} DUMP_MEMBUF_S;

HI_S32 memopen( void )
{
    if (s_s32MemDev <= 0)
    {
        s_s32MemDev = open ("/dev/mem", O_CREAT | O_RDWR | O_SYNC);
        if (s_s32MemDev <= 0)
        {
            return -1;
        }
    }
    return 0;
}

HI_VOID memclose()
{
    close(s_s32MemDev);
}

void* memmap( HI_U32 u32PhyAddr, HI_U32 u32Size )
{
    HI_U32 u32Diff;
    HI_U32 u32PagePhy;
    HI_U32 u32PageSize;
    HI_U8* pPageAddr;

    u32PagePhy = u32PhyAddr & 0xfffff000;
    u32Diff = u32PhyAddr - u32PagePhy;

    /* size in page_size */
    u32PageSize = ((u32Size + u32Diff - 1) & 0xfffff000) + 0x1000;
    pPageAddr = mmap ((void*)0, u32PageSize, PROT_READ | PROT_WRITE,
                      MAP_SHARED, s_s32MemDev, u32PagePhy);
    if (MAP_FAILED == pPageAddr )
    {
        return NULL;
    }
    return (void*) (pPageAddr + u32Diff);
}

HI_S32 memunmap(HI_VOID* pVirAddr, HI_U32 u32Size )
{
    HI_U32 u32PageAddr;
    HI_U32 u32PageSize;
    HI_U32 u32Diff;

    u32PageAddr = (((HI_U32)pVirAddr) & 0xfffff000);
    /* size in page_size */
    u32Diff     = (HI_U32)pVirAddr - u32PageAddr;
    u32PageSize = ((u32Size + u32Diff - 1) & 0xfffff000) + 0x1000;

    return munmap((HI_VOID*)u32PageAddr, u32PageSize);
}



/* sp420 转存为 p420 ; sp422 转存为 p422  */
void sample_yuv_dump(VIDEO_FRAME_S* pVBuf, FILE* pfd)
{
    unsigned int w, h;
    char* pVBufVirt_Y;
    char* pVBufVirt_C;
    char* pMemContent;
    unsigned char TmpBuff[4096];                //如果这个值太小，图像很大的话存不了
    HI_U32 phy_addr, Ysize, Csize;
    PIXEL_FORMAT_E  enPixelFormat = pVBuf->enPixelFormat;
    HI_U32 u32UvHeight;/* 存为planar 格式时的UV分量的高度 */

    Ysize = (pVBuf->u32Stride[0]) * (pVBuf->u32Height);
    if (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == enPixelFormat)
    {
        Csize = (pVBuf->u32Stride[1]) * (pVBuf->u32Height) / 2;
        u32UvHeight = pVBuf->u32Height / 2;
    }
    else
    {
        Csize = (pVBuf->u32Stride[1]) * (pVBuf->u32Height);
        u32UvHeight = pVBuf->u32Height;
    }

    phy_addr = pVBuf->u32PhyAddr[0];

    //printf("phy_addr:%x, size:%d\n", phy_addr, size);
    pVBufVirt_Y = (HI_CHAR*) HI_MPI_SYS_Mmap(phy_addr, Ysize);
    if (NULL == pVBufVirt_Y)
    {
        return;
    }

    pVBufVirt_C = (HI_CHAR*) HI_MPI_SYS_Mmap(pVBuf->u32PhyAddr[1], Csize);
    if (NULL == pVBufVirt_C)
    {
        HI_MPI_SYS_Munmap(pVBufVirt_Y, Ysize);
        return;
    }

    /* save Y ----------------------------------------------------------------*/
    fprintf(stderr, "saving......Y......");
    fflush(stderr);
    for (h = 0; h < pVBuf->u32Height; h++)
    {
        pMemContent = pVBufVirt_Y + h * pVBuf->u32Stride[0];
        fwrite(pMemContent, pVBuf->u32Width, 1, pfd);
    }
    fflush(pfd);


    /* save U ----------------------------------------------------------------*/
    fprintf(stderr, "U......");
    fflush(stderr);
    for (h = 0; h < u32UvHeight; h++)
    {
        pMemContent = pVBufVirt_C + h * pVBuf->u32Stride[1];

        pMemContent += 1;

        for (w = 0; w < pVBuf->u32Width / 2; w++)
        {
            TmpBuff[w] = *pMemContent;
            pMemContent += 2;
        }
        fwrite(TmpBuff, pVBuf->u32Width / 2, 1, pfd);
    }
    fflush(pfd);

    /* save V ----------------------------------------------------------------*/
    fprintf(stderr, "V......");
    fflush(stderr);
    for (h = 0; h < u32UvHeight; h++)
    {
        pMemContent = pVBufVirt_C + h * pVBuf->u32Stride[1];

        for (w = 0; w < pVBuf->u32Width / 2; w++)
        {
            TmpBuff[w] = *pMemContent;
            pMemContent += 2;
        }
        fwrite(TmpBuff, pVBuf->u32Width / 2, 1, pfd);
    }
    fflush(pfd);

    fprintf(stderr, "done %d!\n", pVBuf->u32TimeRef);
    fflush(stderr);

    HI_MPI_SYS_Munmap(pVBufVirt_Y, Ysize);
    HI_MPI_SYS_Munmap(pVBufVirt_C, Csize);

}

HI_S32 SAMPLE_MISC_VoDump(VO_DEV VoDev, VO_CHN VoChn, HI_U32 u32Cnt)
{
    HI_S32 i, s32Ret;
    VIDEO_FRAME_INFO_S stFrame;
    //VIDEO_FRAME_INFO_S astFrame[256];
    HI_CHAR szYuvName[128];
    HI_CHAR szPixFrm[10];
    FILE* pfd;
    VB_POOL hPool  = VB_INVALID_POOLID;
    HI_U32  u32BlkSize = 0;

    DUMP_MEMBUF_S stMem = {0};
    VIDEO_FRAME_INFO_S stFrmInfo;
    VGS_HANDLE hHandle;
    VGS_TASK_ATTR_S stTask;
    HI_U32 u32LumaSize              = 0;
    HI_U32 u32ChrmSize              = 0;
    HI_U32 u32PicLStride            = 0;
    HI_U32 u32PicCStride            = 0;
    HI_U32 u32Width                 = 0;
    HI_U32 u32Height                = 0;
    HI_BOOL bSendToVgs              = HI_FALSE;

    printf("\nNOTICE: This tool only can be used for TESTING !!!\n");
    printf("usage: ./vou_chn_dump [vodev] [vochn] [frmcnt]. sample: ./vou_chn_dump 0 0 5\n\n");

    /* Get Frame to make file name*/
    s32Ret = HI_MPI_VO_GetChnFrame(VoDev, VoChn, &stFrame, 0);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VO(%d)_GetChnFrame errno %#x\n", VoDev, s32Ret);
        return -1;
    }

    /* make file name */
    strcpy(szPixFrm, (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == stFrame.stVFrame.enPixelFormat) ? "p420" : "p422");
    sprintf(szYuvName, "./vo(%d,%d)_%d_%d_%s_%d.yuv", VoDev, VoChn,
            stFrame.stVFrame.u32Width, stFrame.stVFrame.u32Height, szPixFrm, u32Cnt);
    printf("Dump YUV frame of vo(%d,%d) to file: \"%s\"\n", VoDev, VoChn, szYuvName);

    bSendToVgs = (stFrame.stVFrame.enCompressMode > 0) || (stFrame.stVFrame.enVideoFormat > 0);

    HI_MPI_VO_ReleaseChnFrame(VoDev, VoChn, &stFrame);

    /* open file */
    pfd = fopen(szYuvName, "wb");

    if (NULL == pfd)
    {
        return -1;
    }

    memopen();

    u32PicLStride = stFrame.stVFrame.u32Stride[0];
    u32PicCStride = stFrame.stVFrame.u32Stride[0];
    u32LumaSize = stFrame.stVFrame.u32Stride[0] * stFrame.stVFrame.u32Height;
    u32ChrmSize = (stFrame.stVFrame.u32Stride[0] * stFrame.stVFrame.u32Height) >> (1 + (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == stFrame.stVFrame.enPixelFormat));
    u32Width    = stFrame.stVFrame.u32Width;
    u32Height   = stFrame.stVFrame.u32Height;

    if (bSendToVgs)
    {
        u32BlkSize = (PIXEL_FORMAT_YUV_SEMIPLANAR_420 == stFrame.stVFrame.enPixelFormat)
                     ? (stFrame.stVFrame.u32Stride[0] * stFrame.stVFrame.u32Height * 3 >> 1) : (stFrame.stVFrame.u32Stride[0] * stFrame.stVFrame.u32Height * 2);

        /*create comm vb pool*/
        hPool   = HI_MPI_VB_CreatePool( u32BlkSize, 2, NULL);
        if (hPool == VB_INVALID_POOLID)
        {
            printf("HI_MPI_VB_CreatePool failed! \n");
            goto END1;
        }

        stMem.hPool = hPool;
    }

    /* get VO frame  */
    for (i = 0; i < u32Cnt; i++)
    {
        s32Ret = HI_MPI_VO_GetChnFrame(VoDev, VoChn, &stFrame, 20);
        if (HI_SUCCESS != s32Ret)
        {
            printf("get vo(%d,%d) frame err\n", VoDev, VoChn);
            printf("only get %d frame\n", i);
            break;
        }

        if (bSendToVgs)
        {
            while ((stMem.hBlock = HI_MPI_VB_GetBlock(stMem.hPool, u32BlkSize, NULL)) == VB_INVALID_HANDLE)
            {
                ;
            }

            stMem.u32PhyAddr = HI_MPI_VB_Handle2PhysAddr(stMem.hBlock);


            stMem.pVirAddr = (HI_U8*) HI_MPI_SYS_Mmap( stMem.u32PhyAddr, u32BlkSize );
            if (stMem.pVirAddr == NULL)
            {
                printf("Mem dev may not open\n");
                HI_MPI_VB_ReleaseBlock(stMem.hBlock);
                goto END2;
            }

            memset(&stFrmInfo.stVFrame, 0, sizeof(VIDEO_FRAME_S));
            stFrmInfo.stVFrame.u32PhyAddr[0] = stMem.u32PhyAddr;
            stFrmInfo.stVFrame.u32PhyAddr[1] = stFrmInfo.stVFrame.u32PhyAddr[0] + u32LumaSize;
            stFrmInfo.stVFrame.u32PhyAddr[2] = stFrmInfo.stVFrame.u32PhyAddr[1] + u32ChrmSize;

            stFrmInfo.stVFrame.pVirAddr[0] = stMem.pVirAddr;
            stFrmInfo.stVFrame.pVirAddr[1] = (HI_U8*) stFrmInfo.stVFrame.pVirAddr[0] + u32LumaSize;
            stFrmInfo.stVFrame.pVirAddr[2] = (HI_U8*) stFrmInfo.stVFrame.pVirAddr[1] + u32ChrmSize;

            stFrmInfo.stVFrame.u32Width     = u32Width;
            stFrmInfo.stVFrame.u32Height    = u32Height;
            stFrmInfo.stVFrame.u32Stride[0] = u32PicLStride;
            stFrmInfo.stVFrame.u32Stride[1] = u32PicCStride;
            stFrmInfo.stVFrame.u32Stride[2] = u32PicCStride;

            stFrmInfo.stVFrame.enCompressMode = COMPRESS_MODE_NONE;
            stFrmInfo.stVFrame.enPixelFormat  = stFrame.stVFrame.enPixelFormat;
            stFrmInfo.stVFrame.enVideoFormat  = VIDEO_FORMAT_LINEAR;

            stFrmInfo.stVFrame.u64pts     = (i * 40);
            stFrmInfo.stVFrame.u32TimeRef = (i * 2);

            stFrmInfo.u32PoolId = hPool;

            s32Ret = HI_MPI_VGS_BeginJob(&hHandle);
            if (s32Ret != HI_SUCCESS)
            {
                printf("HI_MPI_VGS_BeginJob failed\n");
                HI_MPI_VB_ReleaseBlock(stMem.hBlock);
                HI_MPI_VO_ReleaseChnFrame(VoDev, VoChn, &stFrame);
                goto END2;
            }

            memcpy(&stTask.stImgIn, &stFrame.stVFrame, sizeof(VIDEO_FRAME_INFO_S));
            memcpy(&stTask.stImgOut , &stFrmInfo, sizeof(VIDEO_FRAME_INFO_S));
            s32Ret = HI_MPI_VGS_AddScaleTask(hHandle, &stTask);
            if (s32Ret != HI_SUCCESS)
            {
                printf("HI_MPI_VGS_AddScaleTask failed\n");
                HI_MPI_VGS_CancelJob(hHandle);
                HI_MPI_VB_ReleaseBlock(stMem.hBlock);
                HI_MPI_VO_ReleaseChnFrame(VoDev, VoChn, &stFrame);
                goto END2;
            }

            s32Ret = HI_MPI_VGS_EndJob(hHandle);
            if (s32Ret != HI_SUCCESS)
            {
                printf("HI_MPI_VGS_EndJob failed\n");
                HI_MPI_VGS_CancelJob(hHandle);
                HI_MPI_VB_ReleaseBlock(stMem.hBlock);
                HI_MPI_VO_ReleaseChnFrame(VoDev, VoChn, &stFrame);
                goto END2;
            }

            /* save VO frame to file */
            sample_yuv_dump(&stFrmInfo.stVFrame, pfd);

            HI_MPI_VB_ReleaseBlock(stMem.hBlock);
        }
        else
        {
            /* save VO frame to file */
            sample_yuv_dump(&stFrame.stVFrame, pfd);
        }

        /* release frame after using */
        s32Ret = HI_MPI_VO_ReleaseChnFrame(VoDev, VoChn, &stFrame);
        if (HI_SUCCESS != s32Ret)
        {
            printf("Release vo(%d,%d) frame err\n", VoDev, VoChn);
            printf("only get %d frame\n", i);
            break;
        }
    }


END2:
    if (hPool != VB_INVALID_POOLID)
    {
        HI_MPI_VB_DestroyPool( hPool );
    }

END1:

    memclose();

    fclose(pfd);

    return 0;
}

HI_S32 main(int argc, char* argv[])
{
    VO_DEV VoDev = 0;
    VO_CHN VoChn = 0;
    HI_U32 u32FrmCnt = 1;

    /* VO设备号*/
    if (argc > 1)
    {
        VoDev = atoi(argv[1]);
    }

    /* VO通道号 */
    if (argc > 2)
    {
        VoChn = atoi(argv[2]);
    }

    /* 需要采集的帧数目*/
    if (argc > 3)
    {
        u32FrmCnt = atoi(argv[3]);
    }

    SAMPLE_MISC_VoDump(VoDev, VoChn, u32FrmCnt);

    return HI_SUCCESS;
}


