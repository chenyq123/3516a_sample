#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "hi_common.h"
#include "hi_comm_video.h"
#include "hi_comm_sys.h"
#include "hi_comm_vi.h"
#include "hi_comm_vo.h"
#include "mpi_vb.h"
#include "mpi_sys.h"
#include "mpi_vi.h"
#include "mpi_vo.h"

#include "hi_comm_isp.h"
#include "mpi_isp.h"
#include "hi_sns_ctrl.h"

#include "mpi_af.h"
#include "mpi_awb.h"
#include "mpi_ae.h"


#define MAX_FRM_CNT     256
#define MAX_FRM_WIDTH   4096        //如果这个值太小，图像很大的话存不了


#define ALIGN_BACK(x, a)              ((a) * (((x) / (a))))


#if 1
static HI_S32 s_s32MemDev = -1;

#define MEM_DEV_OPEN() \
    do {\
        if (s_s32MemDev <= 0)\
        {\
            s_s32MemDev = open("/dev/mem", O_CREAT|O_RDWR|O_SYNC);\
            if (s_s32MemDev < 0)\
            {\
                perror("Open dev/mem error");\
                return -1;\
            }\
        }\
    }while(0)

#define MEM_DEV_CLOSE() \
    do {\
        HI_S32 s32Ret;\
        if (s_s32MemDev > 0)\
        {\
            s32Ret = close(s_s32MemDev);\
            if(HI_SUCCESS != s32Ret)\
            {\
                perror("Close mem/dev Fail");\
                return s32Ret;\
            }\
            s_s32MemDev = -1;\
        }\
    }while(0)

void usage(void)
{
    printf(
        "\n"
        "*************************************************\n"
        "Usage: ./vi_bayerdump [nbit] [FrmCnt] [VCNum]\n"
        "nbit: \n"
        "   The bit num to be dump\n"
        "FrmCnt: \n"
        "   the count of frame to be dump\n"
        "VCNum: \n"
        "   line mode:-1:. wdr mode:0 or 1, the frame vc num,long or short frame\n"
        "e.g : ./vi_bayerdump 16 1 -1(line mode)\n"
        "e.g : ./vi_bayerdump 16 1 0(wdr mode:VC0)\n"
        "e.g : ./vi_bayerdump 16 1 1(wdr mode:VC1)\n"
        "*************************************************\n"
        "\n");
    exit(1);
}

HI_VOID* COMM_SYS_Mmap(HI_U32 u32PhyAddr, HI_U32 u32Size)
{
    HI_U32 u32Diff;
    HI_U32 u32PagePhy;
    HI_U32 u32PageSize;
    HI_U8* pPageAddr;

    /* The mmap address should align with page */
    u32PagePhy = u32PhyAddr & 0xfffff000;
    u32Diff    = u32PhyAddr - u32PagePhy;

    /* The mmap size shuld be mutliples of 1024 */
    u32PageSize = ((u32Size + u32Diff - 1) & 0xfffff000) + 0x1000;
    pPageAddr   = mmap ((void*)0, u32PageSize, PROT_READ | PROT_WRITE,
                        MAP_SHARED, s_s32MemDev, u32PagePhy);
    if (MAP_FAILED == pPageAddr )
    {
        perror("mmap error");
        return NULL;
    }
    return (HI_VOID*) (pPageAddr + u32Diff);
}

HI_S32 COMM_SYS_Munmap(HI_VOID* pVirAddr, HI_U32 u32Size)
{
    HI_U32 u32PageAddr;
    HI_U32 u32PageSize;
    HI_U32 u32Diff;

    u32PageAddr = (((HI_U32)pVirAddr) & 0xfffff000);
    u32Diff     = (HI_U32)pVirAddr - u32PageAddr;
    u32PageSize = ((u32Size + u32Diff - 1) & 0xfffff000) + 0x1000;

    return munmap((HI_VOID*)u32PageAddr, u32PageSize);
}
#endif

int sample_bayer_dump(VIDEO_FRAME_S* pVBuf, HI_U32 u32Nbit, FILE* pfd)
{
    unsigned int w, h;
    HI_U16* pVBufVirt_Y;
    HI_U8  au8Data[MAX_FRM_WIDTH];
    HI_U16 au16Data[MAX_FRM_WIDTH];
    HI_U32 phy_addr, size;
    HI_U8* pUserPageAddr[2];

    size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height) * 2;

    phy_addr = pVBuf->u32PhyAddr[0];

    MEM_DEV_OPEN();

    pUserPageAddr[0] = (HI_U8*) COMM_SYS_Mmap(phy_addr, size);
    if (NULL == pUserPageAddr[0])
    {
        return -1;
    }

    pVBufVirt_Y = (HI_U16*)pUserPageAddr[0];

    /* save Y ----------------------------------------------------------------*/
    fprintf(stderr, "saving......Raw data......u32Stride[0]: %d, width: %d\n", pVBuf->u32Stride[0], pVBuf->u32Width);
    fflush(stderr);
    for (h = 0; h < pVBuf->u32Height; h++)
    {
        HI_U16 u16Data;
        for (w = 0; w < pVBuf->u32Width; w++)
        {
            if (8 == u32Nbit)
            {
                au8Data[w] = (pVBufVirt_Y[h * pVBuf->u32Width + w] >> 8);
            }
            else if (10 == u32Nbit)
            {
                u16Data = (pVBufVirt_Y[h * pVBuf->u32Width + w] >> 6);
                au16Data[w] = u16Data;
            }
            else if (12 == u32Nbit)         //12bit 在高位
            {
                u16Data = (pVBufVirt_Y[h * pVBuf->u32Width + w] >> 4);
                au16Data[w] = u16Data;
            }
            else if (14 == u32Nbit)         //14bit 在高位
            {
                u16Data = (pVBufVirt_Y[h * pVBuf->u32Width + w] >> 2);
                au16Data[w] = u16Data;
            }
            else if (16 == u32Nbit)
            {
                u16Data = pVBufVirt_Y[h * pVBuf->u32Width + w];
                au16Data[w] = u16Data;
            }
            else
            {
                printf("Err! Bayer data can't support %d bits!eg: 8bits;10bits;12bits.\n", u32Nbit);
                return -1;
            }
        }

        if (8 == u32Nbit)
        {
            fwrite(au8Data, pVBuf->u32Width, 1, pfd);
        }
        else
        {
            fwrite(au16Data, pVBuf->u32Width, 2, pfd);
        }

    }
    fflush(pfd);

    fprintf(stderr, "done u32TimeRef: %d!\n", pVBuf->u32TimeRef);
    fflush(stderr);

    COMM_SYS_Munmap(pUserPageAddr[0], size);

    MEM_DEV_CLOSE();

    return 0;
}

HI_S32 VI_DumpBayer(VI_DEV ViDev, HI_U32 u32Nbit, HI_U32 u32Cnt, HI_S32 s32VcNum)
{
    int i, j;
    VI_FRAME_INFO_S stFrame;
    VI_FRAME_INFO_S astFrame[MAX_FRM_CNT];
    HI_CHAR szYuvName[128];
    FILE* pfd;
    HI_S32 s32MilliSec = 2000;
    VI_CHN ViChn;
    HI_U32 u32CapCnt;

    VIU_GET_RAW_CHN(ViDev, ViChn);

    if (HI_MPI_VI_SetFrameDepth(ViChn, 1))
    {
        printf("HI_MPI_VI_SetFrameDepth err, vi chn %d \n", ViChn);
        return -1;
    }

    usleep(5000);

    if (HI_MPI_VI_GetFrame(ViChn, &stFrame.stViFrmInfo, s32MilliSec))
    {
        printf("HI_MPI_VI_GetFrame err, vi chn %d \n", ViChn);
        return -1;
    }

    /* get VI frame  */
    for (i = 0; i < u32Cnt; i++)
    {
        if (HI_MPI_VI_GetFrame(ViChn, &astFrame[i].stViFrmInfo, s32MilliSec) < 0)
        {
            printf("get vi chn %d frame err\n", ViChn);
            printf("only get %d frame\n", i);
            break;
        }
    }
    u32CapCnt = i;

    /* make file name */
    if (-1 == s32VcNum)
    {
        sprintf(szYuvName, "./vi_dev_%d_%d_%d_%d_%dbits.raw", ViDev,
                stFrame.stViFrmInfo.stVFrame.u32Width, stFrame.stViFrmInfo.stVFrame.u32Height,
                u32CapCnt, u32Nbit);

    }
    else
    {
        sprintf(szYuvName, "./vi_dev_%d_%d_%d_%d_%dbits_vc%d.raw", ViDev,
                stFrame.stViFrmInfo.stVFrame.u32Width, stFrame.stViFrmInfo.stVFrame.u32Height,
                u32CapCnt, u32Nbit, s32VcNum);
    }

    printf("Dump raw frame of vi chn %d  to file: \"%s\"\n", ViChn, szYuvName);
    HI_MPI_VI_ReleaseFrame(ViChn, &stFrame.stViFrmInfo);

    /* open file */
    pfd = fopen(szYuvName, "wb");
    if (NULL == pfd)
    {
        return -1;
    }

    for (j = 0; j < i; j++)
    {
        /* save VI frame to file */
        sample_bayer_dump(&astFrame[j].stViFrmInfo.stVFrame, u32Nbit, pfd);

        /* release frame after using */
        HI_MPI_VI_ReleaseFrame(ViChn, &astFrame[j].stViFrmInfo);
    }

    fclose(pfd);

    return 0;
}

HI_S32 main(int argc, char* argv[])
{
    VI_DEV ViDev = 0;
    HI_S32 s32Ret = 0;
    HI_U32 u32Nbit = 8;
    HI_U32 u32FrmCnt = 1;
    HI_S32 s32VcNum = -1;
    HI_U32 u32DesIdAddr = 0x20580210;
    HI_U32 u32BackupValue;

    printf("\nNOTICE: This tool only can be used for TESTING !!!\n");
    if (argc < 2)
    {
        usage();
        return s32Ret;
    }

    s32Ret = HI_MPI_SYS_GetReg(u32DesIdAddr, &u32BackupValue);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_SYS_GetReg failed, s32Ret = %d!\n", s32Ret);
        return s32Ret;
    }

    if (0 == u32BackupValue)
    {
        s32VcNum = -1;
    }
    else if (1 == u32BackupValue)
    {
        s32VcNum = 0;
    }
    else if (2 == u32BackupValue)
    {
        s32VcNum = 1;
    }
    else
    {
        s32VcNum = -1;
    }

    if (argc > 1)
    {
        u32Nbit = atoi(argv[1]);    /* nbit of Raw data:8bit;10bit;12bit */
    }
    if (argc > 2)
    {
        u32FrmCnt = atoi(argv[2]);/* the frame number */
    }
    if (argc > 3)
    {
        s32VcNum = atoi(argv[3]);/* long or short frame */
    }

    s32Ret = HI_MPI_VI_EnableBayerDump(ViDev);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_EnableBayerDump failed(0x%x)!\n", s32Ret);
        return s32Ret;
    }

    if (-1 == s32VcNum)
    {
        s32Ret = HI_MPI_SYS_SetReg(u32DesIdAddr, 0x0);
        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_SYS_SetReg failed, s32Ret = 0x%x!\n", s32Ret);
            return s32Ret;
        }
    }
    else if (0 == s32VcNum)
    {
        s32Ret = HI_MPI_SYS_SetReg(u32DesIdAddr, 0x1);
        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_SYS_SetReg failed, s32Ret = 0x%x!\n", s32Ret);
            return s32Ret;
        }
    }
    else if (1 == s32VcNum)
    {
        s32Ret = HI_MPI_SYS_SetReg(u32DesIdAddr, 0x2);
        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_SYS_SetReg failed, s32Ret = 0x%x!\n", s32Ret);
            return s32Ret;
        }
    }
    else
    {
        printf("Invalid VC num!\n");
    }

    printf("===press any key to continue.\n");
    getchar();

    s32Ret = VI_DumpBayer(ViDev, u32Nbit, u32FrmCnt, s32VcNum);
    if (HI_SUCCESS != s32Ret)
    {
        printf("VI_StartBayerData failed!\n");
        goto EXIT;
    }

EXIT:
    s32Ret = HI_MPI_VI_DisableBayerDump(0);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_DisableBayerDump failed!\n");
        return s32Ret;
    }

    s32Ret = HI_MPI_SYS_SetReg(u32DesIdAddr, u32BackupValue);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_SYS_SetReg failed, s32Ret = 0x%x!\n", s32Ret);
        return s32Ret;
    }

    return HI_SUCCESS;
}



