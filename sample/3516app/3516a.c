#include <stdio.h>
#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include "teacher.h"
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include "sample_comm.h"
#include "math.h"
#include <termios.h>
//#include <cv.h>

extern HI_S32 SnapCnt;

const VPSS_GRP VpssGrp = 0;
const VPSS_CHN VpssChn = 0;
const VPSS_GRP SendVencGrp = 0;
const VPSS_CHN SendVencChn = 0;

const VENC_CHN SnapVencChn = 2;
const VENC_GRP SnapVencGrp = 1;

const VI_DEV ViDev = 0;
const VI_CHN ViChn = 0;
const VI_CHN ExtChn = VIU_EXT_CHN_START;
const VI_CHN SnapExtChn = VIU_EXT_CHN_START + 1;

int X1 = 0, Y1 = 10, X2 = 72, Y2 = 100;

#define WIDTH 720
#define HEIGHT 576

short LPoi = 0, RPoi = 720;
short FLen = 0;
const unsigned char YSpeed = 0x10;
const unsigned char ZSpeed = 0x20;


#pragma pack(1)
typedef struct
{
	unsigned int head;
	unsigned char YSpeed;
	unsigned char ZSpeed;
	unsigned int YPoi;
	unsigned int ZPoi;
	unsigned char tail;
} PoiCtrlCode;

typedef struct
{
	unsigned int head;
	unsigned int FLen;
	unsigned char tail;
} FLenCtrlCode;
#pragma pack()

VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;
PIC_SIZE_E enSize = PIC_HD1080;
HI_BOOL isSnap = HI_FALSE, isSend = HI_TRUE;
HI_BOOL isThreadStop = HI_FALSE;
HI_BOOL isChangePic = HI_FALSE;
int analyzeStart = 1;
int changeConfFile(char *KeyName, short Val);
void getLRPoi();
unsigned short getHPoi();
unsigned short getFLen();
size_t writeTTY(const unsigned char *buf, int count);
int SetPanTiltPoi(unsigned char YSpeed, unsigned char ZSpeed, unsigned short YPoi, unsigned short ZPoi);
int SetPanTiltFLen(short FLen);

unsigned int NumToContlCode(short num)
{
	unsigned int contlcode = 0;
	int i;
	for (i = 0; i < 4; i++)
		contlcode ^= ( (unsigned int)num & ( 0xF << ( i * 4 ))) << ( i * 4 );
	return contlcode;
}

double CalAngle(int m)
{
	double x = ( X2 - X1 ) * (m / 100.0) + X1;
	//printf("x=%lf\n",x);
	if ( x > 360)
		return (double)( atan( (double)( x - 360 ) / ( 360.0 * 24.7 )) * 180.0 / 3.1416);
	else
		return -(double)( atan( (double)( 360 - x ) / ( 360.0 * 24.7 )) * 180.0 / 3.1416);
}

int GetProfileString(char *profile, char *KeyName, char *KeyVal)
{
	char buf[100];
	char keyname[50];
	FILE *fp;
	if ((fp = fopen(profile, "r")) == NULL)
	{
		perror("open error");
		return -1;
	}
	fseek(fp, 0, SEEK_SET);
	while (!feof(fp) && fgets(buf, 100, fp) != NULL)
	{
		if (strstr(buf, KeyName) != NULL)
		{
			buf[strlen(buf) - 1] = '\0';
			strcpy(KeyVal, buf + strlen(KeyName) + 1);
			break;
		}
	}
	fclose(fp);
	return 0;
}

void getCoord()
{
	char x1_str[4], x2_str[4], y1_str[4], y2_str[4];

	if (-1 == GetProfileString("/home/tmp.conf", "X1", x1_str))
	{
		X1 = 0;
	}
	X1 = atoi(x1_str);

	if (-1 == GetProfileString("/home/tmp.conf", "X2", x2_str))
	{
		X2 = 720;
	}
	X2 = atoi(x2_str);

	if (-1 == GetProfileString("/home/tmp.conf", "Y1", y1_str))
	{
		Y1 = 100;
	}
	Y1 = atoi(y1_str);

	if (-1 == GetProfileString("/home/tmp.conf", "Y2", y2_str))
	{
		Y2 = 300;
	}
	Y2 = atoi(y2_str);

}

void AnalyzePic()
{
	HI_U32 s32Ret;
	VIDEO_FRAME_INFO_S FrameInfoA;
	struct timeval ustime;
	time_t timep;
	int fd;
	struct termios options;

	getCoord();
	getLRPoi();
	getFLenformfile();
	int flag = 0;

	JSSetparam(X1, Y1, X2, Y2);

	while (1)
	{
		while (analyzeStart)
		{
			//获取图像帧
			//	time(&timep);
			//gettimeofday(&ustime,NULL);
			//printf("%ld\n",((long)ustime.tv_sec)*1000+(long)ustime.tv_usec/1000);
			// printf("%ld\n",timep);
			char *pImg;
			int size;
			char str1[20] = {};
			char num_str[4] = {};
			short YPoi = 0;
			char* str;
            int num =0;
			s32Ret = HI_MPI_VI_GetFrame(ExtChn, &FrameInfoA, -1);
			if (HI_SUCCESS != s32Ret)
			{
				printf("HI_MPI_VI_GetFrameTimeOut faileded with err code %#x!\n", s32Ret);
			}
			//分析图片
			size = FrameInfoA.stVFrame.u32Stride[0] * FrameInfoA.stVFrame.u32Height;
			pImg = (char*)HI_MPI_SYS_Mmap(FrameInfoA.stVFrame.u32PhyAddr[0], size);
			//printf("%s\n",JSMyTest(pImg));
			str = JSMyTest(pImg, str1);
			if (str != NULL)
			{
				printf("%s\n", str);
				//strncpy(num_str, str, strstr(str, ",") - str);
				//YPoi = CalAngle(atoi(num_str));
				//YPoi = -233 + 670 * (((X2 - X1) * (atoi(num_str)) / 100 + X1) / 720.0);
				//if (atoi(num_str) > 0 )
                num = atoi(num_str);
				if (num > 0 )
				{
					//printf("LPoi=%d,RPoi=%d,RPoi-LPoi:%d\n",LPoi,RPoi,RPoi-LPoi);
					YPoi = (short)LPoi + ((short)RPoi - (short)LPoi) * (((X2 - X1) * (num) / 100 + X1) / 720.0);
					SetPanTiltPoi(YSpeed, ZSpeed, YPoi, 0);
					if (flag == 1)
					{
						SetPanTiltFLen(FLen);
						flag = 0;
					}
					time(&timep);
				}
				else
				{
					if (time(NULL) - timep > 5)
					{
						SetPanTiltFLen(0);
						SetPanTiltPoi(0x99, 0x99, 0, 0);
						flag = 1;
					}

				}
			}
			HI_MPI_SYS_Munmap(pImg, size);
			HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfoA);
			//printf("============================================\n");
		}

	}
}

void accept_thread()
{
	char buff[100];
	int ret;
	int readfd, writefd;
	char str[25] = {};
	if (access("myfifo", F_OK) == -1)
	{
		if ((mkfifo("myfifo", 0666) < 0) && (errno != EEXIST))
		{
			printf("Can Not create fifo file!\n");
			exit(-1);
		}
	}
	while (!isThreadStop)
	{
		if ((readfd = open("myfifo", O_RDONLY)) == -1)
		{
			printf("open fifo error!\n");
			exit(1);
		}
		memset(buff, 0, sizeof(buff));
		if ((ret = read(readfd, buff, sizeof(buff))) > 0)
		{
			close(readfd);
			if (!strcmp(buff, "snap"))
			{
				isSnap = HI_TRUE;
#if 0
				if ((writefd = open("myfifo", O_WRONLY)) == -1)
				{
					printf("open fifo error!\n");
					exit(1);
				}
				sprintf(str, "snap_%d.jpg", SnapCnt);
				write(writefd, str, strlen(str) + 1);
				close(writefd);
#endif
				continue;
			}
			if (!strcmp(buff, "analyze"))
			{
				analyzeStart = !analyzeStart;
				continue;
			}
			if (!strcmp(buff, "setcoord"))
			{
				getCoord();
				//printf("%d,%d,%d,%d\n",X1,Y1,X2,Y2);
				JSSetparam(X1, Y1, X2, Y2);
				continue;
			}
			if (!strcmp(buff, "setleftpoi"))
			{
				//printf("setleftpoi\n");
				unsigned short leftpoi;
				leftpoi = getHPoi();
				changeConfFile("leftpoi", leftpoi);
				LPoi =  leftpoi;
				continue;
			}
			if (!strcmp(buff, "setrightpoi"))
			{
				//printf("setrightpoi\n");
				unsigned short rightpoi;
				rightpoi = getHPoi();
				changeConfFile("rightpoi", rightpoi);
				RPoi = rightpoi;
				continue;
			}
			if (!strcmp(buff, "saveflen"))
			{
				//printf("setrightpoi\n");
				unsigned short FLength;
				FLength = getFLen();
				changeConfFile("flen", FLength);
				FLen = FLength;
				continue;
			}
		}
		close(readfd);
		usleep(50);
	}
}

HI_VOID SnapPic()
{
	while (!isThreadStop)
	{
		if (isSnap)
		{
			isSnap = HI_FALSE;
			//printf("snap!!!!!\n");
			HI_S32 s32Ret = SAMPLE_COMM_VENC_SnapProcess(SnapVencChn);
			if (HI_SUCCESS != s32Ret)
			{
				printf("%s: snap process failed!\n", __FUNCTION__);
				continue;
			}
		}
		usleep(50);
	}
}
#if 1
void SendVideoStream(int Sockfd)
{
	HI_S32 s32Ret;
	HI_S32 Vencfd, maxfd = 0;
	PAYLOAD_TYPE_E enPayLoad = PT_H264;
	SAMPLE_RC_E enRcMode = SAMPLE_RC_CBR;
	struct timeval TimeoutVal;
	fd_set read_fds;
	int connfd;
	VENC_STREAM_S stStream;
	VENC_CHN_STAT_S stStat;
	HI_S32 i;
	int SendSockfd;

	struct sockaddr_in SendServaddr;

	SendSockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (SendSockfd == -1)
	{
		perror("send socket error");
		exit(0);
	}
	memset(&SendServaddr, 0, sizeof(SendServaddr));
	SendServaddr.sin_family = AF_INET;
	SendServaddr.sin_addr.s_addr = inet_addr("10.1.2.60");
	SendServaddr.sin_port = htons(6666);

	while (!isThreadStop)
	{
#if 0
		if (!isChangePic)
		{
			connfd = accept(SendSockfd, NULL, NULL);
			printf("connect!\n");
		}
		isChangePic = HI_FALSE;
#endif
		s32Ret = SAMPLE_COMM_VENC_Start(SendVencGrp, SendVencChn, enPayLoad,
		                                gs_enNorm, enSize, enRcMode);
		if (HI_SUCCESS != s32Ret)
		{
			printf("SAMPLE_COMM_VENC_Start failed with err code %#x\n", s32Ret);
			continue;
		}
		s32Ret = SAMPLE_COMM_VENC_BindVpss(SendVencGrp, VpssGrp, VpssChn);
		if (HI_SUCCESS != s32Ret)
		{
			printf("SAMPLE_COMM_VENC_BindVpss failed with err code %#x\n", s32Ret);
			continue;
		}
		Vencfd = HI_MPI_VENC_GetFd(SendVencChn);
		if (Vencfd < 0)
		{
			printf("HI_MPI_VENC_GetFd faild with%#x!\n", Vencfd);
			return HI_FAILURE;
		}
		while (isSend)
		{

			FD_ZERO(&read_fds);
			FD_SET(Vencfd, &read_fds);
			TimeoutVal.tv_sec = 20000;
			TimeoutVal.tv_usec = 0;
			s32Ret = select(Vencfd + 1, &read_fds, NULL, NULL, &TimeoutVal);

			if (s32Ret < 0)
			{
				perror("select failed!\n");
				break;
			}
			else if (s32Ret == 0)
			{
				printf("get sendvenc stream time out,exit thread\n");
				continue;
			}
			else
			{
				if (FD_ISSET(Vencfd, &read_fds))
				{
					memset(&stStream, 0, sizeof(stStream));
					s32Ret = HI_MPI_VENC_Query(SendVencChn, &stStat);
					if (HI_SUCCESS != s32Ret)
					{
						printf("HI_MPI_VENC_Query failed with err code %#x!\n", s32Ret);
						break;
					}
					stStream.pstPack = (VENC_PACK_S*)malloc(sizeof(VENC_PACK_S) * stStat.u32CurPacks);
					if (NULL == stStream.pstPack)
					{
						printf("malloc stream pack failed!\n");
						break;
					}
					stStream.u32PackCount = stStat.u32CurPacks;
					s32Ret = HI_MPI_VENC_GetStream(SendVencChn, &stStream, HI_TRUE);
					if (HI_SUCCESS != s32Ret)
					{
						free(stStream.pstPack);
						stStream.pstPack = NULL;
						printf("HI_MPI_VENC_GetStream failed with %#x!\n", s32Ret);
						break;
					}
					for (i = 0; i < stStream.u32PackCount; i++)
					{
						/*
												//write(connfd, stStream.pstPack[i].pu8Addr[0],
												  //    stStream.pstPack[i].u32Len[0]);

						                          sendto(SendSockfd,stStream.pstPack[i].pu8Addr[0],
						                                  stStream.pstPack[i].u32Len[0],0,
						                                  (struct sockaddr*)&SendServaddr,
						                                  sizeof(SendServaddr));


												if (stStream.pstPack[i].u32Len[1] > 0)
												{
													//write(connfd, stStream.pstPack[i].pu8Addr[1],
													  //    stStream.pstPack[i].u32Len[1]);

						                          sendto(SendSockfd,stStream.pstPack[i].pu8Addr[1],
						                                  stStream.pstPack[i].u32Len[1],0,
						                                  (struct sockaddr*)&SendServaddr,
						                                  sizeof(SendServaddr));
												}
						*/
						sendto(SendSockfd, stStream.pstPack[i].pu8Addr + stStream.pstPack[i].u32Offset,
						       stStream.pstPack[i].u32Len - stStream.pstPack[i].u32Offset, 0,
						       (struct sockaddr*)&SendServaddr, sizeof(SendServaddr));
					}
					s32Ret = HI_MPI_VENC_ReleaseStream(SendVencChn, &stStream);
					if (HI_SUCCESS != s32Ret)
					{
						free(stStream.pstPack);
						stStream.pstPack = NULL;
						break;
					}
					free(stStream.pstPack);
					stStream.pstPack = NULL;
				}
			}
		}
		SAMPLE_COMM_VENC_StopGetStream();
		SAMPLE_COMM_VENC_UnBindVpss(SendVencGrp, VpssGrp, VpssChn);
		SAMPLE_COMM_VENC_Stop( SendVencChn);
		usleep(100);
	}
}
#endif

int main(int argc, char const *argv[])
{
	PAYLOAD_TYPE_E enPayLoad = PT_H264;
	VB_CONF_S stVbConf;
	SAMPLE_VI_CONFIG_S stViConfig;
	VPSS_GRP_ATTR_S stVpssGrpAttr;
	VPSS_CHN_ATTR_S stVpssChnAttr;
	VPSS_CHN_MODE_S stVpssChnMode;
	VI_EXT_CHN_ATTR_S stExtChnAttr;
	SAMPLE_RC_E enRcMode = SAMPLE_RC_CBR;
	HI_S32 s32ChnNum = 1;
	HI_S32 s32Ret = HI_SUCCESS;
	HI_U32 u32BlkSize;
	SIZE_S stSize;
	int sockfd, SendSockfd;
	struct sockaddr_in SendServaddr;
	pthread_t sendvideotid, AnalyzePictid, snaptid, accepttid;
	int ret;
	MPP_CHN_S stSrcChn;
	MPP_CHN_S stDestChn;


	memset(&stVbConf, 0 , sizeof(VB_CONF_S));

	stVbConf.u32MaxPoolCnt = 128;

	SAMPLE_COMM_VI_GetSizeBySensor(&enSize);

	u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm,
	             enSize, SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
	stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
	stVbConf.astCommPool[0].u32BlkCnt = 12;

//	u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm,
//	             PIC_QVGA, SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
//	stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
//	stVbConf.astCommPool[0].u32BlkCnt = 10;

	//初始化系统
	printf("-----------------------V0.02-----------------------\n");
	s32Ret = SAMPLE_COMM_SYS_Init(&stVbConf);
	if (HI_SUCCESS != s32Ret)
	{
		printf("system init failed with err code %#x!\n", s32Ret );
	}
	stViConfig.enViMode = SENSOR_TYPE;
	//stViConfig.enViMode = SONY_IMX122_DC_1080P_30FPS;
	stViConfig.enRotate = ROTATE_NONE;
	stViConfig.enNorm = VIDEO_ENCODING_MODE_AUTO;
	stViConfig.enViChnSet = VI_CHN_SET_NORMAL;


	//配置并启动VI
	s32Ret = SAMPLE_COMM_VI_StartVi(&stViConfig);
	if (HI_SUCCESS != s32Ret)
	{
		printf("start vi failed with err code %#x!\n", s32Ret);
		goto END_1;
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
	s32Ret = HI_MPI_VI_SetFrameDepth(1, 5);
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

	stExtChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
	stExtChnAttr.s32BindChn = ViChn;
	stExtChnAttr.stDestSize.u32Width = WIDTH;
	stExtChnAttr.stDestSize.u32Height = HEIGHT;
	stExtChnAttr.s32DstFrameRate = -1;
	stExtChnAttr.s32SrcFrameRate = -1;
	stExtChnAttr.enCompressMode = COMPRESS_MODE_NONE;
	s32Ret = HI_MPI_VI_SetExtChnAttr(SnapExtChn, &stExtChnAttr);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_SetExtChnAttr failed with err code %#x\n", s32Ret);
		return -1;
	}

	s32Ret = HI_MPI_VI_EnableChn(SnapExtChn);
	if (HI_SUCCESS != s32Ret)
	{
		printf("HI_MPI_VI_EnableChn failed with err code %#x\n", s32Ret);
		return -1;
	}

#if 1

	s32Ret = SAMPLE_COMM_SYS_GetPicSize(gs_enNorm, enSize, &stSize);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_SYS_GetPicSize failed with err code %#x!\n", s32Ret);
		goto END_2;
	}

	//配置并启动VPSS组
	stVpssGrpAttr.u32MaxW = stSize.u32Width;
	stVpssGrpAttr.u32MaxH = stSize.u32Height;
	stVpssGrpAttr.bDciEn = HI_FALSE;
	stVpssGrpAttr.bIeEn = HI_FALSE;
	stVpssGrpAttr.bNrEn = HI_TRUE;
	stVpssGrpAttr.bHistEn = HI_FALSE;
	stVpssGrpAttr.enDieMode = VPSS_DIE_MODE_NODIE;
	stVpssGrpAttr.enPixFmt = SAMPLE_PIXEL_FORMAT;
	s32Ret = SAMPLE_COMM_VPSS_StartGroup(VpssGrp, &stVpssGrpAttr);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_VPSS_StartGroup failed with err code %#x!\n", s32Ret);
		goto END_3;
	}

	//绑定VI和VPSS
	s32Ret = SAMPLE_COMM_VI_BindVpss(stViConfig.enViMode);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_vi_BindVpss failed with err code %#x\n", s32Ret);
		goto END_4;
	}

	//配置并启动VPSS通道
	memset(&stVpssChnAttr, 0, sizeof(stVpssChnAttr));
	stVpssChnAttr.s32SrcFrameRate = -1;
	stVpssChnAttr.s32DstFrameRate = -1;

	stVpssChnMode.enChnMode     = VPSS_CHN_MODE_USER;
	stVpssChnMode.bDouble       = HI_FALSE;
	stVpssChnMode.enPixelFormat = SAMPLE_PIXEL_FORMAT;
	stVpssChnMode.u32Width      = stSize.u32Width;
	stVpssChnMode.u32Height     = stSize.u32Height;
	stVpssChnMode.enCompressMode = COMPRESS_MODE_NONE;
	s32Ret = SAMPLE_COMM_VPSS_EnableChn(VpssGrp, VpssChn, &stVpssChnAttr, &stVpssChnMode, HI_NULL);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_VPSS_EnableChn failed with err code %#x\n", s32Ret);
		goto END_5;
	}

#if 0
	s32Ret = SAMPLE_COMM_VENC_BindVpss(SnapVencGrp, VpssGrp, VpssChn);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_VENC_BindVpss failed with err code %#x\n", s32Ret);
	}
	gs_enNorm = PIC_D1;
	s32Ret = SAMPLE_COMM_SYS_GetPicSize(gs_enNorm, enSize, &stSize);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_SYS_GetPicSize failed with err code %#x\n", s32Ret);
	}
#endif

#endif

	stSrcChn.enModId = HI_ID_VIU;
	stSrcChn.s32DevId = 0;
	stSrcChn.s32ChnId = SnapExtChn;

	stDestChn.enModId = HI_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = SnapVencChn;

	s32Ret = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);
	if (s32Ret != HI_SUCCESS)
	{
		SAMPLE_PRT("failed with %#x!\n", s32Ret);
		//return HI_FAILURE;
	}

	stSize.u32Width = 720;
	stSize.u32Height = 576;
	//printf("%d,%d\n",stSize.u32Width,stSize.u32Height);
	s32Ret = SAMPLE_COMM_VENC_SnapStart(SnapVencChn, &stSize);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_VENC_SnapStart failed with err code %#x\n", s32Ret);
	}

#if 1
#if 0
	//创建发送视频流socket
	//SendSockfd = socket(AF_INET, SOCK_STREAM, 0);
	SendSockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (SendSockfd == -1)
	{
		perror("send socket error");
		exit(0);
	}
	memset(&SendServaddr, 0, sizeof(SendServaddr));
	SendServaddr.sin_family = AF_INET;
	//SendServaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	SendServaddr.sin_addr.s_addr = inet_addr("10.1.2.60");
	SendServaddr.sin_port = htons(6666);
#endif
#if 0
	if (bind(SendSockfd, (struct sockaddr *)&SendServaddr, sizeof(SendServaddr)) == -1)
	{
		perror("send bind error");
		exit(0);
	}
	if (listen(SendSockfd, 10) == -1)
	{
		perror("send listen error");
		exit(0);
	}
#endif
#if 0
	//ret = pthread_create(&sendvideotid, 0, (HI_VOID*)SendVideoStream, SendSockfd);
	ret = pthread_create(&sendvideotid, 0, (HI_VOID*)SendVideoStream, 0);
	if (ret != 0)
	{
		perror("create SendVideoStream error");
		exit(0);
	}
	printf("create SendVideoStream thread successfully!\n");
#endif
#endif
	ret = pthread_create(&AnalyzePictid, 0, (HI_VOID*)AnalyzePic, NULL);
	if (ret != 0)
	{
		perror("create AnalyzePic error");
		exit(0);
	}
	ret = pthread_create(&snaptid, 0, (HI_VOID*)SnapPic, NULL);
	if (ret != 0)
	{
		perror("create SnapPic error");
		exit(0);
	}
	ret = pthread_create(&accepttid, 0, (HI_VOID*)accept_thread, NULL);
	if (ret != 0)
	{
		perror("create accept_thread error");
		exit(0);
	}
	while (1)
	{
		sleep(10);
	}

END_5:
	SAMPLE_COMM_VPSS_DisableChn(VpssGrp, VpssChn);
END_4:
	SAMPLE_COMM_VI_UnBindVpss(stViConfig.enViMode);
END_3:
	SAMPLE_COMM_VPSS_StopGroup(VpssGrp);
END_2:
	SAMPLE_COMM_VI_StopVi(&stViConfig);
END_1:
	SAMPLE_COMM_SYS_Exit();

	close(SendSockfd);
	pthread_join(sendvideotid, NULL);
	return 0;
}

int changeConfFile(char *KeyName, short Val)
{
	FILE *fp;
	char str[20][20] = {};
	char buf[50] = {};
	char Val_str[10] = {};
	char *p;
	int i, k;
	int ishad = 0;
	if ((fp = fopen("/home/tmp.conf", "r")) == NULL)
	{
		perror("open error");
		return -1;
	}
	fseek(fp, 0, SEEK_SET);
	i = 0;
	while (!feof(fp) && fgets(buf, 50, fp) != NULL)
	{
		if ((p = strstr(buf, KeyName)) != NULL)
		{
			sprintf(Val_str, "%d\n\0", Val);
			strcpy(p + strlen(KeyName) + 1, Val_str);
			ishad = 1;
	}
		strcpy(str[i], buf);
		i++;
	}
	fclose(fp);

	if ((fp = fopen("/home/tmp.conf", "w")) == NULL)
	{
		perror("open error");
		return -1;
	}
	for (k = 0; k < i; k++)
	{
		fprintf(fp, "%s", str[k]);
	}
	if (ishad == 0)
	{
		fprintf(fp, "%s=%d\n", KeyName, Val);
	}
	fclose(fp);
}


unsigned short getHPoi()
{
	unsigned char answercode[11] = {};
	unsigned char ctrlcode[] = {0x87, 0x09, 0x06, 0x12, 0xFF};
	unsigned short poi = 0;
	int ret;
	int fd;
	int i;
	int flag;
	struct termios Opt;
	fd = open("/dev/ttyS000", O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		fprintf(stdout, "open tty error!\n");
		return -1;
	}
	tcgetattr(fd, &Opt);
	Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	Opt.c_oflag &= ~OPOST;
	Opt.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	cfsetispeed(&Opt, B9600);
	cfsetospeed(&Opt, B9600);
	tcsetattr(fd, TCSANOW, &Opt);
	tcflush(fd, TCIFLUSH);
	tcflush(fd, TCOFLUSH);
	write(fd, ctrlcode, sizeof(ctrlcode));
	usleep(100);
	while ((ret = read(fd, answercode, sizeof(answercode))) < 0)
	{
		if (ret < 0)
			fprintf(stdout, "read error");
	}
#if 1
	for (i = 0; i < 11; i++)
	{
		printf("%x\n", answercode[i]);
	}
#endif
	for (i = 2; i <= 5; i++)
	{
		poi |= answercode[i] << (4 * (5 - i));
	}
	printf("%x\n", poi);
	close(fd);
	return poi;
}
void getLRPoi()
{
	char LPoi_str[10], RPoi_str[10];

	if (-1 == GetProfileString("/home/tmp.conf", "leftpoi", LPoi_str))
	{
		LPoi = 0;
	}
	LPoi = atoi(LPoi_str);

	if (-1 == GetProfileString("/home/tmp.conf", "rightpoi", RPoi_str))
	{
		RPoi = 0;
	}
	RPoi = atoi(RPoi_str);

}

void getFLenformfile()
{
	char FLen_str[10] = {};
	if (-1 == GetProfileString("/home/tmp.conf", "flen", FLen_str))
	{
		FLen = 0;
	}
	FLen = atoi(FLen_str);
}


unsigned short getFLen()
{
	unsigned char answercode[11] = {};
	unsigned char ctrlcode[] = {0x87, 0x09, 0x04, 0x47, 0xFF};
	unsigned short poi = 0;
	int ret;
	int fd;
	int i;
	struct termios Opt;
	fd = open("/dev/ttyS000", O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		fprintf(stdout, "open tty error!\n");
		return -1;
	}
	tcgetattr(fd, &Opt);
	Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	Opt.c_oflag &= ~OPOST;
	Opt.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	cfsetispeed(&Opt, B9600);
	cfsetospeed(&Opt, B9600);
	tcsetattr(fd, TCSANOW, &Opt);
	tcflush(fd, TCIFLUSH);
	tcflush(fd, TCOFLUSH);
	write(fd, ctrlcode, sizeof(ctrlcode));
	usleep(100);
	while ((ret = read(fd, answercode, sizeof(answercode))) < 0)
	{
		if (ret < 0)
			fprintf(stdout, "read error");
	}
	for (i = 2; i <= 5; i++)
	{
		poi |= answercode[i] << (4 * (5 - i));
	}
	printf("FLen:%x\n", poi);
	close(fd);
	return poi;
}


size_t writeTTY(const unsigned char *buf, int count)
{
	int fd;
	struct termios options;
	size_t ret = 0;
	fd = open("/dev/ttyS000", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0)
	{
		perror("open tty error");
		return -1;
	}
	tcgetattr(fd, &options);
	options.c_lflag &= ~(ICANON | ECHO | ECHOE);
	options.c_oflag &= ~OPOST;
	cfsetospeed(&options, B9600);
	cfsetispeed(&options, B9600);
	tcsetattr(fd, TCSANOW, &options);
	tcflush(fd, TCOFLUSH);
	tcflush(fd, TCIFLUSH);
	ret = write(fd, buf, count);
	close(fd);
	return ret;
}
int SetPanTiltPoi(unsigned char YSpeed, unsigned char ZSpeed,
                  unsigned short YPoi, unsigned short ZPoi)
{
	PoiCtrlCode c;
	int ret;
	memset(&c, 0, sizeof(c));
	c.head = htonl(0x87010602);
	c.YSpeed = YSpeed;
	c.ZSpeed = ZSpeed;
	c.YPoi = htonl(NumToContlCode(YPoi));
	c.ZPoi = htonl(NumToContlCode(ZPoi));
	c.tail = 0xFF;
	ret = writeTTY(&c, sizeof(c));
	return ret;
}

int SetPanTiltFLen(short FLen)
{
	FLenCtrlCode f;
	int ret;
	f.head = htonl(0x87010447);
	f.FLen = htonl(NumToContlCode(FLen));
	f.tail = 0xFF;
	ret = writeTTY(&f, sizeof(f));
	return ret;
}


