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
#include "libdetect_t.h"
#include "cJSON.h"
#include "math.h"
#include <termios.h>
using namespace cv;
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
#define STU_IMG_WIDTH 480
#define STU_IMG_HEIGHT 360
#define TEACHER_IMG_WIDTH 360
#define TEACHER_IMG_HEIGHT 360

short LPoi = 0, RPoi = 720;
short FLen = 0;
//const unsigned char YSpeed = 0x08;
//const unsigned char ZSpeed = 0x08;
const unsigned char YSpeed = 0x10;
const unsigned char ZSpeed = 0x50;
KVConfig *cfg_ = NULL;
TeacherDetecting *detect_ = NULL;
det_t *pdet = NULL;
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
int g_reset = 0;

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
//void vector_to_json_t(std::vector < Rect > r, cv::Rect upbody_rect, bool is_upbody, bool is_rect, char *buf);
int get_roi(const char *pts, int *x1, int *y1, int *x2, int *y2);
void sendPoitoPtz();
void getRectbyJSON(const char *json_str, vector<cv::Rect> &rects);
int getframe(Mat *Img, int ExtChn);
int getframe_resize(int width, int height, int ExtChn);
int getframe_init(int width, int height, int ExtChn);

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
    //bool isrect;
    //VIDEO_FRAME_INFO_S FrameInfoA;
    //vector<cv::Rect> first_r;
    //vector<cv::Rect> r;
    int img_w = 480, img_h = 360;
    int first_ = 1;
    while (1)
    {
        //if(detect_ != NULL)
        //{
        //    delete detect_;
        //}
        if(pdet != NULL)
        {
            det_close(pdet);
            pdet = NULL;
        }
        cfg_->reload();
        g_reset = 0;
        //detect_ = new TeacherDetecting(cfg_);
        //cfg_changed = 0;
        detect_init();
        printf("X1:%d,Y1:%d,X2:%d,Y2:%d\n",X1,Y1,X2,Y2);
        int model = atoi(cfg_->get_value("model"));
        if(model == 0)
        {
            pdet = det_open("teacher_detect_trace.config");
            img_w = TEACHER_IMG_WIDTH;
            img_h = TEACHER_IMG_HEIGHT;
            getframe_resize(img_w, img_h, VIU_EXT_CHN_START);
            sleep(1);
        }
        else if(model == 2)
        {
            pdet= det_open("student_detect_trace.config");
            img_w = STU_IMG_WIDTH;
            img_h = STU_IMG_HEIGHT;
            getframe_resize(img_w, img_h, VIU_EXT_CHN_START);
            sleep(1);
        }
        while (1)
        {
            //if(cfg_changed == 1)
            //{
            //    break;
            //}

            if(g_reset == 1)
            {
                break;
            }

            if(analyzeStart == 0)
            {
                usleep(100);
                continue;
            }
            Mat Img(img_h, img_w, CV_8UC3, 0);
            Img.create(img_h, img_w, CV_8UC3);
            getframe(&Img, VIU_EXT_CHN_START);
            const char *str = det_detect(pdet, Img);
            vector<cv::Rect> r;
            getRectbyJSON(str,r);
            int num = r.size();
            if(num == 1)
            {
                int detect_x = (r[0].x + r[0].width) / 2 + r[0].x;
                short YPoi = (short)LPoi + ((short)RPoi - (short)LPoi) * (detect_x / 480.0);
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
#if 1
                //if(sitdown_det == 1)
                //{
                //    if(upbody.width > 0 && upbody.height >0 && upbody.y > 45)
                //    {
                //        sitdowned  = 1;
                //    }
                //    else
                //    {
                //        sitdowned = 0;
                //    }
                //}
                //else
                //{
                //    sitdowned = 0;
                //}
                //sendYPoi = YPoi;
            }
            else if(num > 1)
            {
                //SetPanTiltPoi(YSpeed, ZSpeed, 0, default_ZPoi);
                sendYPoi = 0;
                sitdowned = 0;
            }

            //vector_to_json_t(r, upbody, is_up_body, isrect, str);
            //printf("%s\n",str);
            //分析图片
#endif
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
                //cfg_changed = 1;
                g_reset = 1;
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
            //  changeConfFile("rightpoi", rightpoi);
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

int main(int argc, char const *argv[])
{
    VI_EXT_CHN_ATTR_S stExtChnAttr;
    HI_S32 s32Ret = HI_SUCCESS;
    int sockfd, SendSockfd;
    struct sockaddr_in SendServaddr;
    pthread_t sendvideotid, AnalyzePictid, snaptid, accepttid, sendPoitid;
    int ret;
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;
    SIZE_S stSize;

#if 0
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
#endif
    getframe_init(480,270, VIU_EXT_CHN_START);

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

//void vector_to_json_t(std::vector < Rect > r, cv::Rect upbody_rect, bool is_upbody, bool is_rect, char *buf)
//{
//    int offset = 0;
//    offset = sprintf(buf, "{\"stamp\":%d,", time(0));
//
//    if (!is_rect)
//    {
//        offset += sprintf(buf + offset, "\"rect\":[ ],");
//    }
//    else
//    {
//        offset += sprintf(buf + offset, "\"rect\":[");
//
//        for (int i = 0; i < r.size(); i++)
//        {
//            Rect t = r[i];
//            if (i == 0)
//                offset +=
//                    sprintf(buf + offset,
//                            "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                            t.x, t.y, t.width, t.height);
//            else
//                offset +=
//                    sprintf(buf + offset,
//                            ",{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                            t.x, t.y, t.width, t.height);
//        }
//
//        offset += sprintf(buf + offset, "],");
//    }
//
//    if (!is_upbody)
//    {
//        offset += sprintf(buf + offset, "\"up_rect\":{\"x\":0,\"y\":0,\"width\":0,\"height\":0}");
//    }
//    else
//    {
//        offset += sprintf(buf + offset, "\"up_rect\":");
//        offset += sprintf(buf + offset, "{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d}",
//                          upbody_rect.x, upbody_rect.y, upbody_rect.width, upbody_rect.height);
//    }
//
//    strcat(buf, "}");
//}

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
    if(HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_SetExtChnAttr failed with err code %#x\n", s32Ret);
        return -1;
    }

    s32Ret = HI_MPI_VI_SetFrameDepth(ExtChn, 1);
    if(HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_SetFrameDepth failed with err code %#x\n", s32Ret);
        return -1;
    }

    s32Ret = HI_MPI_VI_EnableChn(ExtChn);
    if(HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_VI_EnableChn failed with err code %#x", s32Ret);
        return -1;
    }
    return 1;
}

int getframe_resize(int width, int height, int ExtChn)
{
    getframe_init(width, height, ExtChn);
}

/**
    获取一帧图片
    *Img：获取到的图片指针，格式为opencv的Mat
    ExtChn：从vi的ExtChn中获取图片
**/
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
    s32Ret = HI_MPI_SYS_MmzAlloc(&stDst.u32PhyAddr[0], (void**)&stDst.pu8VirAddr[0], "user", HI_NULL, u32DstBlkSize);
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

void getRectbyJSON(const char *json_str, vector<cv::Rect> &rects)
{
    cJSON *json = cJSON_Parse(json_str);
    cJSON *rects_json = cJSON_GetObjectItem(json, "rect");
    int rect_num = cJSON_GetArraySize(rects_json);
    for(int i = 0; i < rect_num; i++)
    {
        cv::Rect r;
        cJSON *rect_json = cJSON_GetArrayItem(rects_json, i - 1);
        r.x = cJSON_GetObjectItem(rect_json, "x")->valueint;
        r.y = cJSON_GetObjectItem(rect_json, "y")->valueint;
        r.width = cJSON_GetObjectItem(rect_json, "width")->valueint;
        r.height = cJSON_GetObjectItem(rect_json, "height")->valueint;
        rects.push_back(r);
    }
}
