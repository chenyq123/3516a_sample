#include <iostream>
#include <cstdio>
#include "detect_t.h"
#include "KVConfig.h"
#include "hi_comm_ive.h"
#include "mpi_ive.h"
#include "hi_ive.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/opencv.hpp"
#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
//#include "teacher.h"
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

extern HI_S32 SnapCnt;

const VPSS_GRP VpssGrp = 0;
const VPSS_CHN VpssChn = 0;
const VPSS_GRP SendVencGrp = 0;
const VPSS_CHN SendVencChn = 0;

const VENC_CHN SnapVencChn = 2;
const VENC_CHN SnapVencGrp = 0;

const VI_DEV ViDev = 0;
const VI_CHN ViChn = 0;
const VI_CHN ExtChn = 1;
const VI_CHN SnapExtChn = 2;

int X1 = 0, Y1 = 10, X2 = 72, Y2 = 100;
int upbody_x1,upbody_x2,upbody_y1,upbody_y2;

#define SNAPWIDTH 480
#define SNAPHEIGHT 272
#define ANALYZEWIDTH 480
#define ANALYZEHEIGHT 272

short LPoi = 0, RPoi = 720;
short FLen = 0;
//const unsigned char YSpeed = 0x08;
//const unsigned char ZSpeed = 0x08;
const unsigned char YSpeed = 0x10;
const unsigned char ZSpeed = 0x50;
KVConfig *cfg_ = NULL;
TeacherDetecting *detect_ = NULL;
unsigned short default_ZPoi;
unsigned short sitdown_ZPoi;


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
}FLenCtrlCode;
#pragma pack()

VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;
PIC_SIZE_E enSize = PIC_HD1080;
HI_BOOL isSnap = HI_FALSE, isSend = HI_TRUE;
HI_BOOL isThreadStop = HI_FALSE;
HI_BOOL isChangePic = HI_FALSE;
int analyzeStart = 1;
int cfg_changed = 0;
int is_sitdown = 0;
int sitdown_det = 0;
int sitdowned = 0;
short sendYPoi = 0;

int changeConfFile(char *KeyName, short Val);
void getLRPoifromfile();
unsigned short getHPoi();
unsigned short getFLen();
unsigned short getZPoi();
void detect_init();
size_t writeTTY(const unsigned char *buf, int count);
int SetPanTiltPoi(unsigned char YSpeed, unsigned char ZSpeed, unsigned short YPoi, unsigned short ZPoi);
int SetPanTiltFLen(short FLen);
void frame2rgb(VIDEO_FRAME_INFO_S *pFrameInfo, Mat *Img);
void getFLenfromfile();
void vector_to_json_t(std::vector < Rect > r, cv::Rect upbody_rect, bool is_upbody, bool is_rect, char *buf);
int get_roi(const char *pts, int *x1, int *y1, int *x2, int *y2);
void sendPoitoPtz();

unsigned int NumToContlCode(short num)
{
	unsigned int contlcode = 0;
	int i;
	for (i = 0; i < 4; i++)
		contlcode |= ( (unsigned int)num & ( 0xF << ( i * 4 ))) << ( i * 4 );
	return contlcode;
}


void getRectfromfile()
{
    get_roi(cfg_->get_value("calibration_data"), &X1, &Y1, &X2, &Y2);
    get_roi(cfg_->get_value("upbody_calibration_data"), &upbody_x1, &upbody_y1, &upbody_x2, &upbody_y2);
}
void getZPoifromfile()
{
    const char *str = cfg_->get_value("default_ZPoi");
    if(str != 0)
        default_ZPoi = atoi(str);
    else
        default_ZPoi = 0;

    str = cfg_->get_value("sitdown_ZPoi");
    if(str != 0)
        sitdown_ZPoi = atoi(str);
    else
        sitdown_ZPoi = 0;
}

void AnalyzePic()
{
	HI_U32 s32Ret;
    bool isrect;
	VIDEO_FRAME_INFO_S FrameInfoA;
    //vector<cv::Rect> first_r;
    //vector<cv::Rect> r;
    int first_ = 1;
	while (1)
	{
        if(detect_ != NULL)
        {
            delete detect_;
        }
        detect_ = new TeacherDetecting(cfg_);
        cfg_changed = 0;
        detect_init();
        printf("X1:%d,Y1:%d,X2:%d,Y2:%d\n",X1,Y1,X2,Y2);
		while (1)
		{
            if(cfg_changed == 1)
            {
                break;
            }

            if(analyzeStart == 0)
            {
                usleep(100);
                continue;
            }
            char str[1024]={};
			//获取图像帧
			s32Ret = HI_MPI_VI_GetFrame(ExtChn, &FrameInfoA, -1);
			if (HI_SUCCESS != s32Ret)
			{
				printf("HI_MPI_VI_GetFrameTimeOut faileded with err code %#x!\n", s32Ret);
                continue;
			}
            Mat Img(272, 480, CV_8UC3, 0);
            Img.create(272, 480, CV_8UC3);
            frame2rgb(&FrameInfoA, &Img);
			//cvtColor(Img,Img,CV_RGB2BGR);
            vector<cv::Rect> r;
            vector<cv::Rect> first_r;
            //if(first_ == 1)
            //{
            //    imwrite("/app/a.jpg",Img);
            //    first_ = 0;
            //}
            Mat masked_img_temp = Mat(Img, detect_->masked_rect);
            Mat masked_img;
            masked_img_temp.copyTo(masked_img);
            detect_->do_mask(masked_img);
            isrect = detect_->one_frame_luv(Img, masked_img, r, first_r);

            for(int i = 0; i < r.size(); i++)
            {
                cv::Rect box = detect_->masked_rect;
                r[i].x = r[i].x + box.x;
                r[i].y = r[i].y + box.y;
                r[i] &= cv::Rect(0, 0, Img.cols, Img.rows);
            }
            cv::Rect upbody;
            bool is_up_body = false;
            if(atoi(cfg_->get_value("t_upbody_detect", "0")) > 0)
            {
                Mat masked_upbody = Mat(Img, detect_->upbody_masked_rect);
                detect_->get_upbody(masked_upbody);
                if(detect_->up_update.upbody_rect.size() > 0)
                {
                    cv::Rect upbody_t = detect_->up_update.upbody_rect[0];
                    if(!(upbody_t.x + upbody_t.width / 2 > detect_->upbody_masked_rect.width - 15 || upbody_t.x + upbody_t.width / 2 < 15))
                    {
                        is_up_body = true;
                        cv::Rect box = detect_->upbody_masked_rect;
                        upbody = upbody_t;
                        upbody.x = upbody.x + box.x;
                        upbody.y = upbody.y + box.y;
                        upbody &= cv::Rect(0, 0, Img.cols, Img.rows);
                    }
                }
            }
            int num = r.size();
            //printf("num = %d\n",num);
            if(num == 1)
            {
                int detect_x = (r[0].x + r[0].width) / 2 + r[0].x;
                //printf("%d\n",r[0].x);
                short YPoi = (short)LPoi + ((short)RPoi - (short)LPoi) * (detect_x / 480.0);
                //printf("YPoi:%x\n",YPoi);
#if 0
                if(is_sitdown == 1 && sitdown_det == 1)
                {
                    if(detect_x < upbody_x1|| detect_x > upbody_x2)
                    {
                        is_sitdown = 0;
                        SetPanTiltPoi(YSpeed, ZSpeed, YPoi, default_ZPoi);
                    }
                    else
                    {
                        SetPanTiltPoi(YSpeed, ZSpeed, YPoi, sitdown_ZPoi);
                    }
                }
                else
                {
                    //int upbodynum = upbody.size();
                    if(upbody.width > 0 && upbody.height >0 && sitdown_det == 1)
                    {
                        //if(detect_x > upbody_x1 && detect_x < upbody_x2 && ((float)(upbody_y1 + (upbody.height + upbody_y1) / 2 - upbody.y) / (float)(upbody_y2 - upbody_y1)) > 0.5)
                        if(detect_x > upbody_x1 && detect_x < upbody_x2 && upbody.y > 45)
                        {
                            is_sitdown = 1;
                            SetPanTiltPoi(YSpeed, ZSpeed, YPoi, sitdown_ZPoi);
                        }
                        else
                        {
                            SetPanTiltPoi(YSpeed, ZSpeed, YPoi, default_ZPoi);
                        }
                    }
                    else
                    {
                        SetPanTiltPoi(YSpeed, ZSpeed, YPoi, default_ZPoi);
                    }
                }
            }
#endif
                if(sitdown_det == 1)
                {
                    if(upbody.width > 0 && upbody.height >0 && upbody.y > 45)
                    {
                        sitdowned  = 1;
                    }
                    else
                    {
                        sitdowned = 0;
                    }
                }
                else
                {
                    sitdowned = 0;
                }
                sendYPoi = YPoi;
            }
            else if(num > 1)
            {
                //SetPanTiltPoi(YSpeed, ZSpeed, 0, default_ZPoi);
                sendYPoi = 0;
                sitdowned = 0;
            }

            vector_to_json_t(r, upbody, is_up_body, isrect, str);
	        printf("%s\n",str);
			//分析图片
			HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfoA);
		}

	}
}

void accept_thread()
{
	char buff[100];
	int ret;
	int readfd, writefd;
	char str[25] = {};
	if (access("/home/myfifo", F_OK) == -1)
	{
		if ((mkfifo("/home/myfifo", 0666) < 0) && (errno != EEXIST))
		{
			printf("Can Not create fifo file!\n");
			exit(-1);
		}
	}
	while (!isThreadStop)
	{
		if ((readfd = open("/home/myfifo", O_RDONLY)) == -1)
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
				//analyzeStart = !analyzeStart;
                analyzeStart = analyzeStart == 0 ? 1 : 0;
				continue;
			}
			if (!strcmp(buff, "setcali"))
			{
                cfg_->reload();
                cfg_changed = 1;
				continue;
			}
			if (!strcmp(buff, "setleftpoi"))
			{
				//printf("setleftpoi\n");
				unsigned short leftpoi;
				leftpoi = getHPoi();
				//changeConfFile("leftpoi", leftpoi);
                cfg_->set_value("leftpoi", leftpoi);
                cfg_->save_as("/home/teacher_detect_trace.config");
				LPoi =  leftpoi;
				continue;
			}
			if (!strcmp(buff, "setsitzpoi"))
			{
				unsigned short sitzpoi;
				printf("setsitzpoi\n");
				sitzpoi = getZPoi();
                cfg_->set_value("sitdown_ZPoi", sitzpoi);
                cfg_->save_as("/home/teacher_detect_trace.config");
				sitdown_ZPoi = sitzpoi;
				continue;
			}
			if (!strcmp(buff, "setdefzpoi"))
			{
				unsigned short defzpoi;
				printf("setdefzpoi\n");
				defzpoi = getZPoi();
                cfg_->set_value("default_ZPoi", defzpoi);
                cfg_->save_as("/home/teacher_detect_trace.config");
				default_ZPoi = defzpoi;
				continue;
			}
			if (!strcmp(buff, "setrightpoi"))
			{
				//printf("setrightpoi\n");
				unsigned short rightpoi;
				rightpoi = getHPoi();
			//	changeConfFile("rightpoi", rightpoi);
                cfg_->set_value("rightpoi", rightpoi);
                cfg_->save_as("/home/teacher_detect_trace.config");
				RPoi = rightpoi;
				continue;
			}
			if (!strcmp(buff, "saveflen"))
			{
				//printf("setrightpoi\n");
				unsigned short FLength;
				FLength = getFLen();
				//changeConfFile("flen", FLength);
                cfg_->set_value("flen", FLength);
                cfg_->save_as("/home/teacher_detect_trace.config");
				FLen = FLength;
				continue;
			}
		}
		close(readfd);
		usleep(50);
	}
}

void SnapPic()
{
	while (!isThreadStop)
	{
		if (isSnap)
		{
			isSnap = HI_FALSE;
			printf("snap!!!!!\n");
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
		s32Ret = SAMPLE_COMM_VENC_Start(SendVencChn, enPayLoad,
		                                gs_enNorm, enSize, enRcMode, 0);
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
			return ;
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
		SAMPLE_COMM_VENC_Stop(SendVencChn);
		usleep(100);
	}
}

int main(int argc, char const *argv[])
{
	VPSS_GRP_ATTR_S stVpssGrpAttr;
	VPSS_CHN_ATTR_S stVpssChnAttr;
	VPSS_CHN_MODE_S stVpssChnMode;
	VI_EXT_CHN_ATTR_S stExtChnAttr;
	SAMPLE_RC_E enRcMode = SAMPLE_RC_CBR;
	HI_S32 s32Ret = HI_SUCCESS;
	int sockfd, SendSockfd;
	struct sockaddr_in SendServaddr;
	pthread_t sendvideotid, AnalyzePictid, snaptid, accepttid, sendPoitid;
	int ret;
	MPP_CHN_S stSrcChn;
	MPP_CHN_S stDestChn;
	SIZE_S stSize;
	SAMPLE_VI_CONFIG_S stViConfig;

	stViConfig.enViMode = SENSOR_TYPE;
	//stViConfig.enViMode = SONY_IMX122_DC_1080P_30FPS;
	stViConfig.enRotate = ROTATE_NONE;
	stViConfig.enNorm = VIDEO_ENCODING_MODE_AUTO;
	stViConfig.enViChnSet = VI_CHN_SET_NORMAL;
#if 0
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
	u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm,
	             enSize, SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
	stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
	stVbConf.astCommPool[0].u32BlkCnt = 10;

//	u32BlkSize = SAMPLE_COMM_SYS_CalcPicVbBlkSize(gs_enNorm,
//	             PIC_QVGA, SAMPLE_PIXEL_FORMAT, SAMPLE_SYS_ALIGN_WIDTH);
//	stVbConf.astCommPool[0].u32BlkSize = u32BlkSize;
//	stVbConf.astCommPool[0].u32BlkCnt = 10;

	//初始化系统
	printf("-----------------------V0.01-----------------------\n");
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
#endif
	stExtChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
	stExtChnAttr.s32BindChn = ViChn;
	stExtChnAttr.stDestSize.u32Width = ANALYZEWIDTH;
	stExtChnAttr.stDestSize.u32Height = ANALYZEHEIGHT;
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
	stExtChnAttr.stDestSize.u32Width = SNAPWIDTH;
	stExtChnAttr.stDestSize.u32Height = SNAPHEIGHT;
	stExtChnAttr.s32DstFrameRate = -1;
	stExtChnAttr.s32SrcFrameRate = -1;
	stExtChnAttr.enCompressMode = COMPRESS_MODE_NONE;
    HI_MPI_VI_DisableChn(SnapExtChn);
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
#if 0

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
#endif

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

	//stDestChn.enModId = HI_ID_GROUP;
	stDestChn.enModId = HI_ID_VENC;
	stDestChn.s32DevId = 0;
	stDestChn.s32ChnId = SnapVencChn;


	stSize.u32Width = 480;
	stSize.u32Height = 272;
	//printf("%d,%d\n",stSize.u32Width,stSize.u32Height);
VENC_START:
    printf("1\n");
	s32Ret = SAMPLE_COMM_VENC_SnapStart(SnapVencChn, &stSize);
	if (HI_SUCCESS != s32Ret)
	{
		printf("SAMPLE_COMM_VENC_SnapStart failed with err code %#x\n", s32Ret);
        if(s32Ret == 0xa0088004)
        {
            HI_MPI_VENC_DestroyChn(SnapVencChn);
	        HI_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
            goto VENC_START;
        }
	}

    printf("2\n");
	s32Ret = HI_MPI_SYS_Bind(&stSrcChn, &stDestChn);
	if (s32Ret != HI_SUCCESS)
	{
		SAMPLE_PRT("failed with %#x!\n", s32Ret);
		//return HI_FAILURE;
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
    cfg_ = new KVConfig("/home/teacher_detect_trace.config");
#if 1
	ret = pthread_create(&AnalyzePictid, 0, (void* (*)(void*))AnalyzePic, NULL);
	if (ret != 0)
	{
		perror("create AnalyzePic error");
		exit(0);
	}
#endif
	ret = pthread_create(&snaptid, 0, (void* (*)(void*))SnapPic, NULL);
	if (ret != 0)
	{
		perror("create SnapPic error");
		exit(0);
	}
	ret = pthread_create(&accepttid, 0, (void* (*)(void*))accept_thread, NULL);
	if (ret != 0)
	{
		perror("create accept_thread error");
		exit(0);
	}
    sleep(1);
	ret = pthread_create(&sendPoitid, 0, (void* (*)(void*))sendPoitoPtz, NULL);
	if (ret != 0)
	{
		perror("create sendPoitoPtz error");
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
#if 0
	for (i = 0; i < 11; i++)
	{
		printf("%x\n", answercode[i]);
	}
#endif
	for (i = 2; i <= 5; i++)
	{
		poi |= answercode[i] << (4 * (5 - i));
	}
	//printf("%x\n", poi);
	close(fd);
	return poi;
}

unsigned short getZPoi()
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
#if 0
	for (i = 0; i < 11; i++)
	{
		printf("%x\n", answercode[i]);
	}
#endif
	for (i = 2; i <= 5; i++)
	{
		poi |= answercode[i + 4] << (4 * (5 - i));
	}
	//printf("%x\n", poi);
	close(fd);
	return poi;
}

void getLRPoifromfile()
{
	//char LPoi_str[10], RPoi_str[10];
	const char *LPoi_str, *RPoi_str;

	//if (-1 == GetProfileString("/home/tmp.conf", "leftpoi", LPoi_str))
	LPoi_str = cfg_->get_value("leftpoi");
    if(LPoi_str == 0)
	{
		LPoi = 0;
	}
    else
        LPoi = atoi(LPoi_str);

	//if (-1 == GetProfileString("/home/tmp.conf", "rightpoi", RPoi_str))
	RPoi_str = cfg_->get_value("rigthpoi");
    if(RPoi_str == 0)
	{
		RPoi = 0;
	}
    else
	    RPoi = atoi(RPoi_str);

}

void getFLenfromfile()
{
	//char FLen_str[10] = {};
	const char *FLen_str;
	//if (-1 == GetProfileString("/home/tmp.conf", "flen", FLen_str))
	FLen_str = cfg_->get_value("flen");
    if(FLen_str == 0)
	{
		FLen = 0;
	}
    else
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
	//printf("FLen:%x\n", poi);
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
	ret = writeTTY((const unsigned char*)&c, sizeof(c));
	return ret;
}

int SetPanTiltFLen(short FLen)
{
    FLenCtrlCode f;
    int ret;
    f.head = htonl(0x87010447);
    f.FLen = htonl(NumToContlCode(FLen));
    f.tail = 0xFF;
    //printf("FLen=%x\n",f.FLen);
    ret = writeTTY((const unsigned char*)&f, sizeof(f));
    return ret;
}

void frame2rgb(VIDEO_FRAME_INFO_S *pFrameInfo, Mat *Img)
{
#if 0
    HI_S32 s32Ret;
    HI_U32 u32BlkSize, u32DstBlkSize;
    IVE_SRC_INFO_S stSrc;
    IVE_MEM_INFO_S stDst;
    IVE_CSC_CTRL_S stCscCtrl;
    HI_BOOL bInstant = HI_TRUE;
    HI_BOOL bFinish;
    IVE_HANDLE IveHandle;
    char *stDst_VirAddr = NULL;

    stSrc.enSrcFmt = IVE_SRC_FMT_SP420;
    stCscCtrl.enOutFmt = IVE_CSC_OUT_FMT_PACKAGE;
    stCscCtrl.enCscMode= IVE_CSC_MODE_PIC_BT709;

    u32DstBlkSize = pFrameInfo->stVFrame.u32Stride[0] * pFrameInfo->stVFrame.u32Height * 3;
    s32Ret = HI_MPI_SYS_MmzAlloc_Cached(&stDst.u32PhyAddr, (void**)&stDst_VirAddr, "user", HI_NULL, u32DstBlkSize);
    if(s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_SYS_MmzAlloc_Cached failed with err code %#x!\n", s32Ret);
    }
    HI_MPI_SYS_MmzFlushCache(stDst.u32PhyAddr, (void**)stDst_VirAddr, u32DstBlkSize);

    stSrc.stSrcMem.u32PhyAddr = pFrameInfo->stVFrame.u32PhyAddr[0];
    stSrc.stSrcMem.u32Stride = pFrameInfo->stVFrame.u32Stride[0];
    stSrc.u32Width = pFrameInfo->stVFrame.u32Width;
    stSrc.u32Height = pFrameInfo->stVFrame.u32Height;

    stDst.u32Stride = pFrameInfo->stVFrame.u32Stride[0];

    s32Ret = HI_MPI_IVE_CSC(&IveHandle, &stSrc, &stDst, &stCscCtrl, bInstant);
    if(s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_CSC failed with err code %#x\n",s32Ret);
        goto FRAME2RGB_END;
    }

    s32Ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
    if(s32Ret != HI_SUCCESS)
    {
        printf("HI_MPI_IVE_Query failed with err code %#x\n",s32Ret);
        goto FRAME2RGB_END;
    }
    memcpy((void*)(Img->data), (void*)stDst_VirAddr, u32DstBlkSize);
FRAME2RGB_END:
    HI_MPI_SYS_MmzFree(stDst.u32PhyAddr, stDst_VirAddr);
#endif
	HI_S32 s32Ret;
	HI_U32 u32BlkSize, u32DstBlkSize;
	IVE_SRC_IMAGE_S stSrc;
	IVE_DST_IMAGE_S stDst;
	IVE_CSC_CTRL_S stCscCtrl;
	HI_BOOL bInstant = HI_TRUE;
	HI_BOOL bFinish;
	IVE_HANDLE IveHandle;


	stSrc.enType = IVE_IMAGE_TYPE_YUV420SP;
	stDst.enType = IVE_IMAGE_TYPE_U8C3_PACKAGE;
	stCscCtrl.enMode = IVE_CSC_MODE_PIC_BT709_YUV2RGB;


	u32DstBlkSize = pFrameInfo->stVFrame.u32Stride[0] * pFrameInfo->stVFrame.u32Height * 3;
	s32Ret = HI_MPI_SYS_MmzAlloc_Cached(&stDst.u32PhyAddr[0], (void**)&stDst.pu8VirAddr[0], "user", HI_NULL, u32DstBlkSize);
	if (s32Ret != HI_SUCCESS)
	{
		printf("HI_MPI_SYS_MmzAlloc_Cached failed with err code %#x!\n", s32Ret);
	}
	HI_MPI_SYS_MmzFlushCache(stDst.u32PhyAddr[0], (void**)stDst.pu8VirAddr[0], u32DstBlkSize);

	stDst.pu8VirAddr[0] = (HI_U8*) HI_MPI_SYS_Mmap(stDst.u32PhyAddr[0], u32DstBlkSize);
	stDst.u16Stride[0] = pFrameInfo->stVFrame.u32Stride[0];
	stDst.u16Width = pFrameInfo->stVFrame.u32Width;
	stDst.u16Height = pFrameInfo->stVFrame.u32Height;

	stSrc.u32PhyAddr[0] = pFrameInfo->stVFrame.u32PhyAddr[0];
	stSrc.u32PhyAddr[1] = pFrameInfo->stVFrame.u32PhyAddr[1];
	stSrc.u32PhyAddr[2] = pFrameInfo->stVFrame.u32PhyAddr[2];

	stSrc.pu8VirAddr[0] = (HI_U8*)HI_MPI_SYS_Mmap(pFrameInfo->stVFrame.u32PhyAddr[0], u32DstBlkSize / 2);
	stSrc.pu8VirAddr[1] = stSrc.pu8VirAddr[0] + pFrameInfo->stVFrame.u32Stride[0] * pFrameInfo->stVFrame.u32Height;
	stSrc.pu8VirAddr[2] = stSrc.pu8VirAddr[1] + 1;


	stSrc.u16Stride[1] = pFrameInfo->stVFrame.u32Stride[0];
	stSrc.u16Stride[0] = pFrameInfo->stVFrame.u32Stride[0];
	stSrc.u16Stride[2] = 0;

	stSrc.u16Width = pFrameInfo->stVFrame.u32Width;
	stSrc.u16Height = pFrameInfo->stVFrame.u32Height;

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
}

void vector_to_json_t(std::vector < Rect > r, cv::Rect upbody_rect, bool is_upbody, bool is_rect, char *buf)
{
	int offset = 0;
	offset = sprintf(buf, "{\"stamp\":%d,", time(0));

	if (!is_rect)
	{
		offset += sprintf(buf + offset, "\"rect\":[ ],");
	}
	else
	{
		offset += sprintf(buf + offset, "\"rect\":[");

		for (int i = 0; i < r.size(); i++)
		{
			Rect t = r[i];
			if (i == 0)
				offset +=
				    sprintf(buf + offset,
				            "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
				            t.x, t.y, t.width, t.height);
			else
				offset +=
				    sprintf(buf + offset,
				            ",{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
				            t.x, t.y, t.width, t.height);
		}

		offset += sprintf(buf + offset, "],");
	}

	if (!is_upbody)
	{
		offset += sprintf(buf + offset, "\"up_rect\":{\"x\":0,\"y\":0,\"width\":0,\"height\":0}");
	}
	else
	{
		offset += sprintf(buf + offset, "\"up_rect\":");
		offset += sprintf(buf + offset, "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
		                  upbody_rect.x, upbody_rect.y, upbody_rect.width, upbody_rect.height);
	}

	strcat(buf, "}");
}

int get_roi(const char *pts, int *x1, int *y1, int *x2, int *y2)
{
    if(pts)
    {
        char *data = strdup(pts);
        char *p = strtok(data, ";");
        int i = 1;
        while(p)
        {
            int x, y;
            if(sscanf(p, "%d,%d", &x, &y) == 2)
            {
                if(i == 1)
                {
                    *x1 = x;
                    *y1 = y;
                }
                if(i == 3)
                {
                    *x2 = x;
                    *y2 = y;
                }
            }
            p = strtok(0, ";");
            i++;
        }
        free(data);
    }
    else
        return -1;
    return 1;
}

void detect_init()
{
	getLRPoifromfile();
    getFLenfromfile();
    getRectfromfile();
    getZPoifromfile();
    cfg_->reload();
    const char *str = cfg_->get_value("t_upbody_detect");
    sitdown_det = atoi(str);
}

void sendPoitoPtz()
{
    while(1)
    {
        if(analyzeStart == 0)
        {
            usleep(100);
            continue;
        }
        SetPanTiltPoi(YSpeed, ZSpeed, sendYPoi, sitdowned == 0 ? default_ZPoi : sitdown_ZPoi);
        usleep(200);
    }
}
