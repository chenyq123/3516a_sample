#include <stdio.h>
#include <string.h>
#include "sample_comm.h"
#include "hi_comm_ive.h"
#include "hi_ive.h"
#include "mpi_ive.h"


#define WIDTH  720
#define HEIGHT 576

PIC_SIZE_E enSize = PIC_HD1080;
const VI_CHN ExtChn = VIU_EXT_CHN_START;
const VI_CHN ViChn = 0;
VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;

int main(int argc, char const *argv[])
{
    VB_CONF_S stVbConf;
    VI_EXT_CHN_ATTR_S stExtChnAttr;
	SAMPLE_VI_CONFIG_S stViConfig;
    SAMPLE_RC_E enRcMode = SAMPLE_RC_CBR;
    VB_BLK hBlock;
    VB_POOL hPool = VB_INVALID_POOLID;
    HI_S32 s32ChnNum = 1;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32BlkSize;
    SIZE_S stSize;
    VIDEO_FRAME_INFO_S FrameInfo;
    IVE_HANDLE IveHandle;
    IVE_SRC_IMAGE_S stSrc;
    IVE_DST_IMAGE_S stDst;
    IVE_CSC_CTRL_S stCscCtrl;
    HI_BOOL bInstant = HI_TRUE;
    HI_BOOL bFinish;
    FILE *fp;
    int k = 1;

    memset(&stVbConf, 0, sizeof(VB_CONF_S));
    stVbConf.u32MaxPoolCnt = 128;
    SAMPLE_COMM_VI_GetSizeBySensor(&enSize);
    u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm,
                enSize, SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
    stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
    stVbConf.astCommPool[0].u32BlkCnt = 12;

	s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
	if (HI_SUCCESS != s32Ret)
	{
		printf("system init failed with err code %#x!\n", s32Ret );
	}

	stViConfig.enViMode = SENSOR_TYPE;
	stViConfig.enRotate = ROTATE_NONE;
	stViConfig.enNorm = VIDEO_ENCODING_MODE_AUTO;
	stViConfig.enViChnSet = VI_CHN_SET_NORMAL;

	s32Ret = SAMPLE_COMM_VI_StartVi(&stViConfig);
	if (HI_SUCCESS != s32Ret)
	{
		printf("start vi failed with err code %#x!\n", s32Ret);
	//	goto END_1;
	}
	stExtChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
	stExtChnAttr.s32BindChn = ViChn;
	stExtChnAttr.stDestSize.u32Width = WIDTH;
	stExtChnAttr.stDestSize.u32Height = HEIGHT;
	stExtChnAttr.s32DstFrameRate = -1;
	stExtChnAttr.s32SrcFrameRate = -1;
	stExtChnAttr.enCompressMode = COMPRESS_MODE_NONE;
	s32Ret = HI_MPI_VI_SetExtChnAttr(ExtChn, &stExtChnAttr);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_SetExtChnAttr failed with err code %#x\n", s32Ret);
		return -1;
	}
	s32Ret = HI_MPI_VI_SetFrameDepth(ExtChn, 5);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_SetFrameDepth failed with err code %#x\n", s32Ret);
		return -1;
	}
    stSrc.enType = IVE_IMAGE_TYPE_YUV420SP;
    stDst.enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
    stCscCtrl.enMode = IVE_CSC_MODE_VIDEO_BT601_YUV2RGB;

	s32Ret = HI_MPI_VI_EnableChn(ExtChn);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_EnableChn failed with err code %#x\n", s32Ret);
		return -1;
	}
#if 1
    while(k)
    {
        s32Ret = HI_MPI_VI_GetFrame(ExtChn, &FrameInfo, -1);
        if(HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_VI_GetFrame failed with err code %#x!\n",s32Ret);
        }
        printf("get frame!\n");
        HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfo);

        u32BlkSize = FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height * 3;
        hPool = HI_MPI_VB_CreatePool(u32BlkSize, 2, NULL);
        if(hPool == VB_INVALID_POOLID)
        {
            printf("HI_MPI_VB_CreatePool failed !\n");
        }
        hBlock = HI_MPI_VB_GetBlock(hPool, u32BlkSize, NULL);
        if(hBlock == VB_INVALID_HANDLE)
        {
            printf("HI_MPI_VB_GetBlock failed !\n");
        }
        stDst.u32PhyAddr[0] = HI_MPI_VB_Handle2PhysAddr(hBlock);
        stDst.u32PhyAddr[1] = stDst.u32PhyAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;
        stDst.u32PhyAddr[2] = stDst.u32PhyAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height * 2;

        stDst.pu8VirAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(stDst.u32PhyAddr[0], u32BlkSize);
        stDst.pu8VirAddr[1] = stDst.pu8VirAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;
        stDst.pu8VirAddr[2] = stDst.pu8VirAddr[0] + FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height * 2;

        stDst.u16Stride[0] = FrameInfo.stVFrame.u32Stride[0];
        stDst.u16Stride[1] = FrameInfo.stVFrame.u32Stride[0];
        stDst.u16Stride[2] = FrameInfo.stVFrame.u32Stride[0];

        stDst.u16Width = FrameInfo.stVFrame.u32Width;
        stDst.u16Height = FrameInfo.stVFrame.u32Height;

        stSrc.u32PhyAddr[0] = FrameInfo.stVFrame.u32PhyAddr[0];
        stSrc.u32PhyAddr[1] = FrameInfo.stVFrame.u32PhyAddr[1];

        stDst.pu8VirAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(stSrc.u32PhyAddr[0], u32BlkSize);
        stSrc.pu8VirAddr[1] = (HI_U8*)FrameInfo.stVFrame.pVirAddr[1];

        stSrc.u16Stride[1] = FrameInfo.stVFrame.u32Stride[0];
        stSrc.u16Stride[0] = FrameInfo.stVFrame.u32Stride[0];

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
#if 0
        fp = fopen("save.yuv","wb");
        fwrite(stSrc.pu8VirAddr[0], 720 * 576 * 3 , 1, fp);
        fclose(fp);
#endif
        HI_MPI_SYS_Munmap(stDst.pu8VirAddr[0], u32BlkSize);
        HI_MPI_VB_ReleaseBlock(hBlock);
        if(hPool != VB_INVALID_POOLID)
        {
            HI_MPI_VB_DestroyPool(hPool);
        }
        HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfo);
    }
#endif
}
