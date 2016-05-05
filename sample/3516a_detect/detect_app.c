#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include "sample_comm.h"
#include "cJSON.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <pthread.h>
#include <netinet/tcp.h>
#include "conf_file.h"


#define WIDTH  720
#define HEIGHT 576

PIC_SIZE_E enSize = PIC_HD1080;
const VI_CHN ExtChn = VIU_EXT_CHN_START + 1;
const VI_CHN ViChn = 0;
VIDEO_NORM_E gs_enNorm = VIDEO_ENCODING_MODE_NTSC;
int X1 = 120, Y1 = 200, X2 = 220, Y2 = 250;
int connfd;
int socketfd;
int disconnect = 1;

int start_analysis = 0;

typedef struct RECT
{
    int x;
    int y;
    int width;
    int height;
}Rect;

typedef struct COORDARRAY coordarray;

typedef struct COORDARRAY
{
    int x;
    int y;
    coordarray *next;
}coordarray;

void AnalyzeCMD(char *text);
void sig_pipe(int signo);
void AnalyzePic();
void RecvCMD();
void sendPoi();
void getRect(cJSON *json);
void upgrade(int size);
int return_ok(int connectfd);
int return_failed(int connectfd);
int StringtoCoordArray(char *str, coordarray **array);
int StringtoCoord(char *str, int *x, int *y);
int freeCoordArray(coordarray **array);
int sendRectPoi(int stamp, Rect rect[], int num, Rect up_rect, int connfd);
int socketconnect(int socketfd);

int main(int argc, char const *argv[])
{
    VI_EXT_CHN_ATTR_S stExtChnAttr;
    HI_S32 s32Ret = HI_SUCCESS;
    pthread_t pictid, recvcmdtid;

    stExtChnAttr.enPixFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    stExtChnAttr.s32BindChn = ViChn;
    stExtChnAttr.stDestSize.u32Width = WIDTH;
    stExtChnAttr.stDestSize.u32Height = HEIGHT;
    stExtChnAttr.s32DstFrameRate = -1;
    stExtChnAttr.s32SrcFrameRate = -1;
    stExtChnAttr.enCompressMode = COMPRESS_MODE_NONE;

    HI_MPI_VI_DisableChn(ExtChn);

   // printf("---------------------v0.01-----------------------");
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
    signal(SIGPIPE, sig_pipe);
    pthread_create(&pictid, NULL, AnalyzePic, NULL);
    pthread_create(&recvcmdtid, NULL, RecvCMD, NULL);
    while(1)
    {
        sleep(10);
    }

}


void RecvCMD()
{
    struct sockaddr_in servaddr;
    char text[1024];
    int n;
    int ret;
    struct timeval timeout={10,0};
    int keepAlive = 1;
    int keepIdle = 60;
    int keepInterval = 5;
    int keepCount = 3;
    int reuse = 1;

    if((socketfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket error ");
        exit(0);
    }
#if 0
    setsockopt(socketfd, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepAlive,sizeof(keepAlive));
    setsockopt(socketfd, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepIdle,sizeof(keepIdle));
    setsockopt(socketfd, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepInterval,sizeof(keepInterval));
    setsockopt(socketfd, SOL_SOCKET, SO_KEEPALIVE, (void*)&keepCount,sizeof(keepCount));
#endif
    //setsockopt(socketfd, IPPROTO_TCP, TCP_NODELAY, (void*)&reuse, sizeof(reuse));
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(9001);
    if(bind(socketfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
    {
        perror("bind error");
        exit(0);
    }
    if(listen(socketfd, 10) == -1)
    {
        perror("listen error");
        exit(0);
    }
    while(1)
    {
        connfd = accept(socketfd, (struct sockaddr*)NULL, NULL);
        if(connfd == -1)
        {
            perror("accept error");
        }
        disconnect = 0;
        while(1)
        {
            if(disconnect == 1)
            {
                close(connfd);
                break;
            }
            usleep(20);
            n = recv(connfd, text, 1024, 0);
            if(n > 0)
            {
                AnalyzeCMD(text);
            }
            else
            {
                if(0 == socketconnect(socketfd))
                {
                    disconnect = 1;
                }
            }
        }
    }
}
void AnalyzePic()
{
    char *pImg;
    int size;
    char str1[20];
    char *str;
    char X1_str[4], X2_str[4], Y1_str[4], Y2_str[4];
    char x_str[4] = {};
    int x;
    HI_S32 s32Ret;
    int ret;
    VIDEO_FRAME_INFO_S FrameInfo;

    GetProfileString("/home/detect.conf", "X1", X1_str);
    GetProfileString("/home/detect.conf", "X2", X2_str);
    GetProfileString("/home/detect.conf", "Y1", Y1_str);
    GetProfileString("/home/detect.conf", "Y2", Y2_str);
    X1 = atoi(X1_str);
    Y1 = atoi(Y1_str);
    X2 = atoi(X2_str);
    Y2 = atoi(Y2_str);

    JSSetparam(X1,Y1,X2,Y2);
    while(1)
    {
        if(start_analysis == 0)
        {
            continue;
        }
        s32Ret = HI_MPI_VI_GetFrame(ExtChn, &FrameInfo, -1);
        if(HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_VI_GetFrame failed with err code %#x!\n",s32Ret);
        }
        size = FrameInfo.stVFrame.u32Stride[0] * FrameInfo.stVFrame.u32Height;
        pImg = (char*)HI_MPI_SYS_Mmap(FrameInfo.stVFrame.u32PhyAddr[0], size);
        str = JSMyTest(pImg, str1);
        if(str != NULL)
        {
#if 0
            cJSON *sendjson = cJSON_CreateObject();
            strncpy(x_str, str, strstr(str, ",") - str);
            x = atoi(x_str);
            cJSON_AddNumberToObject(sendjson, "x", x);
            cJSON_AddNumberToObject(sendjson, "y", 0);
            cJSON_AddNumberToObject(sendjson, "width", 1);
            cJSON_AddNumberToObject(sendjson, "height", 1);
            char *sendstr = cJSON_Print(sendjson);
            ret = send(connfd, sendstr, strlen(sendstr) + 1, 0);
            if(ret < 0)
            {
                start_analysis = 0;
            }
            cJSON_Delete(sendjson);
            free(sendstr);
#endif
            Rect rect,up_rect;
            int stamp = 12345;
            strncpy(x_str, str, strstr(str, ",") - str);
            x = atoi(x_str);
            //printf("x=%d\n",x);
            rect.x = x;
            rect.y = 1;
            rect.width = 1;
            rect.height = 1;
            up_rect.x = 0;
            up_rect.y = 0;
            up_rect.width = 0;
            up_rect.height = 0;
            int ret = sendRectPoi(stamp, &rect, 1, up_rect, connfd);
            if(ret < 0)
            {
                if(socketconnect(socketfd) == 0)
                {
                    disconnect = 1;
                    start_analysis = 0;
                }
            }

        }
        HI_MPI_VI_ReleaseFrame(ExtChn, &FrameInfo);
    }
}
void AnalyzeCMD(char *text)
{
    cJSON *json, *json_cmd, *json_filesize;
    json = cJSON_Parse(text);
    if(!json)
    {
        printf("Error before: [%s]\n",cJSON_GetErrorPtr());
    }
    else
    {
        json_cmd = cJSON_GetObjectItem(json, "cmd");
        if(json_cmd->type == cJSON_Number)
        {
            //printf("value:%d\n",json_cmd->valueint);
            switch(json_cmd->valueint)
            {
                case 1:
                    getRect(json);
                    //printf("X1:%d,X2:%d,Y1:%d,Y2:%d\n",X1,X2,Y1,Y2);
                    break;
                case 2:
                    sendPoi();
                    break;
                case 3:
                    json_filesize = cJSON_GetObjectItem(json, "size");
                    if(json_filesize != NULL)
                        upgrade(json_filesize->valueint);
                    else
                        send(connfd, "failed", 7, 0);
                    break;
                default:
                    break;
            }
        }
        cJSON_Delete(json);
    }

}
void getRect(cJSON *json)
{
    cJSON *json_calibration, *json_upbody;
    json_calibration = cJSON_GetObjectItem(json, "calibration_data");
    json_upbody = cJSON_GetObjectItem(json, "upbody_calibration_data");
    if(json_calibration->type == cJSON_String)
    {
        coordarray *array;
        coordarray *p;
        //printf("*array1:%x\n",array);;
        int ret = StringtoCoordArray(json_calibration->valuestring, &array);
        //printf("*array2:%x\n",array);;
        p = array;
        if(ret == -1)
        {
            printf("error\n");
            return_failed(connfd);
            return ;
        }
        else
        {
            X1 = p->x;
            Y1 = p->y;
           // printf("p->x:%d,p->y:%d\n",p->x, p->y);
            p = p->next;
            X2 = p->x;
            Y2 = p->y;
            changeConfFile("/home/detect.conf", "X1", X1);
            changeConfFile("/home/detect.conf", "X2", X2);
            changeConfFile("/home/detect.conf", "Y1", Y1);
            changeConfFile("/home/detect.conf", "Y2", Y2);
            JSSetparam(X1,Y1,X2,Y2);
            //freeCoordArray(&array);
            //return_ok(connfd);
        }
    }
    else
    {
        return_failed(connfd);
        return ;
    }

    if(json_upbody->type == cJSON_String)
    {
        int upbody_x,upbody_y;
        int ret = StringtoCoord(json_upbody->valuestring, &upbody_x, &upbody_y);
        if(ret == -1)
        {
            printf("error\n");
            return_failed(connfd);
            return ;
        }
        else
        {
            changeConfFile("/home/detect.conf", "upbody_x", upbody_x);
            changeConfFile("/home/detect.conf", "upbody_y", upbody_y);
            printf("ok\n");
            return_ok(connfd);
        }
    }
    else
    {
        return_failed(connfd);
        return ;
    }
}
/*
void getRect(cJSON *json)
{
    cJSON *json_valueX1, *json_valueY1, *json_valueW, *json_valueH;
    json_valueX1 = cJSON_GetObjectItem(json, "X1");
    if(json_valueX1->type == cJSON_Number)
    {
        if(json_valueX1->valueint > 0)
        {
            X1 = json_valueX1->valueint;
        }
    }

    json_valueY1 = cJSON_GetObjectItem(json, "Y1");
    if(json_valueY1->type == cJSON_Number)
    {
        if(json_valueY1->valueint > 0)
        {
            Y1 = json_valueY1->valueint;
        }
    }

    json_valueW = cJSON_GetObjectItem(json, "W");
    if(json_valueW->type == cJSON_Number)
    {
        if(json_valueW->valueint > 0)
        {
            X2 = json_valueW->valueint + X1;
        }
    }

    json_valueH = cJSON_GetObjectItem(json, "H");
    if(json_valueH->type == cJSON_Number)
    {
        if(json_valueH->valueint > 0)
        {
            Y2 = json_valueH->valueint + Y1;
        }
    }
    changeConfFile("/home/detect.conf", "X1", X1);
    changeConfFile("/home/detect.conf", "X2", X2);
    changeConfFile("/home/detect.conf", "Y1", Y1);
    changeConfFile("/home/detect.conf", "Y2", Y2);
    JSSetparam(X1,Y1,X2,Y2);
    return_ok(connfd);
}
*/
void sendPoi()
{
    start_analysis =1;
}

void sig_pipe(int signo)
{
    start_analysis = 0;
    disconnect = 1;
}

void upgrade(int size)
{
    start_analysis = 0;
    char buf[1024] = {};
    int n = 0;
    int sum = 0;
    int ret;
    FILE *save = fopen("/home/detect_upgrade","w+");
    if(save == NULL)
    {
        perror("fopen error");
        return ;
    }
    while(1)
    {
        n = recv(connfd, buf, sizeof(buf), 0);
        if(n < 0)
            perror("recv error");
        if(n == 0)
            break;
        fwrite(buf, n, 1, save);
        sum  += n;
    }
    fclose(save);
    if(sum == size)
    {
        ret = return_ok(connfd);
        system("chmod +x /home/detect_upgrade");
        system("mv  /home/detect_upgrade /home/detect_app");
        if(ret < 0)
            perror("send error");
        printf("success !\n");
        system("reboot");
    }
    else
    {
        system("rm /home/detect_upgrade");
        ret = return_failed(connfd);
        if(ret < 0)
            perror("send error");
        printf("failed!\n");
    }
    disconnect = 1;
}

int return_ok(int connectfd)
{
    cJSON *sendjson = cJSON_CreateObject();
    cJSON_AddStringToObject(sendjson, "result", "ok");
    char *sendstr = cJSON_Print(sendjson);
    int ret = send(connectfd, sendstr, strlen(sendstr) + 1, 0);
    cJSON_Delete(sendjson);
    return ret;
}


int return_failed(int connectfd)
{
    cJSON *sendjson = cJSON_CreateObject();
    cJSON_AddStringToObject(sendjson, "result", "failed");
    char *sendstr = cJSON_Print(sendjson);
    int ret = send(connectfd, sendstr, strlen(sendstr) + 1, 0);
    cJSON_Delete(sendjson);
    return ret;
}

int StringtoCoordArray(char *str, coordarray **array)
{
    int num = 0;
    char *find = NULL;
    int first = 1;
    coordarray *p;
    coordarray *temp;
    printf("%s,%d\n",str,strlen(str));
    //printf("*array:%x\n",*array);
    while(1)
    {
        char x_str[10] = {}, y_str[10] = {};
        find = strstr(str, ",");
        if(find == NULL)
            return -1;
        num = find - str;
        strncpy(x_str, str, num);
        str = str + num + 1;

        find = strstr(str, ";");
        if(find == NULL)
            return -1;
        num = find - str;
        strncpy(y_str, str, num);
        str = str + num + 1;

        if(first == 1)
        {
            *array = (coordarray*)malloc(sizeof(coordarray));
            (*array)->x = atoi(x_str);
            (*array)->y = atoi(y_str);
            //printf("x:%d,y:%d\n", (*array)->x, (*array)->y);
            (*array)->next = NULL;
            first = 0;
            p = *array;
        }
        else
        {
            temp = (coordarray*)malloc(sizeof(coordarray));
            temp->x = atoi(x_str);
            temp->y = atoi(y_str);
            //printf("x:%d,y:%d\n", atoi(x_str), atoi(y_str));
            temp->next = NULL;
            p->next = temp;
            p = p->next;
        }

        if(*str == '\0')
            break;
    }
    return 1;
}
int StringtoCoord(char *str, int *x, int *y)
{
    char x_str[10] = {}, y_str[10] = {};
    char *find = NULL;
    int num = 0;

    printf("%s\n",str);

    find = strstr(str, ",");
    if(find == NULL)
        return -1;
    num = find - str;

    strncpy(x_str, str, num);
    *x = atoi(x_str);

    str = str + num + 1;

    strcpy(y_str, str);
    *y = atoi(y_str);

    return 1;

}

int freeCoordArray(coordarray **array)
{
    coordarray *p = *array;
    coordarray *q;
    while(p != NULL)
    {
        q = p->next;
        free(p);
        p = q;
    }
    *array = NULL;
    return 1;
}

int sendRectPoi(int stamp, Rect rect[], int num, Rect up_rect, int connfd)
{
    cJSON *pRoot = cJSON_CreateObject();
    cJSON *pRectArray = cJSON_CreateArray();
    cJSON *pUp_rect = cJSON_CreateObject();
    cJSON *pRect[10];
    char *sendtext;
    int i;
    int ret;
    for(i = 0;i < num; i++)
    {
        pRect[i] = cJSON_CreateObject();
        cJSON_AddNumberToObject(pRect[i], "x", rect[i].x);
        cJSON_AddNumberToObject(pRect[i], "y", rect[i].y);
        cJSON_AddNumberToObject(pRect[i], "width", rect[i].width);
        cJSON_AddNumberToObject(pRect[i], "height", rect[i].height);
        cJSON_AddItemToArray(pRectArray, pRect[i]);
    }

    cJSON_AddNumberToObject(pUp_rect, "up_x", up_rect.x);
    cJSON_AddNumberToObject(pUp_rect, "up_y", up_rect.y);
    cJSON_AddNumberToObject(pUp_rect, "up_width", up_rect.width);
    cJSON_AddNumberToObject(pUp_rect, "up_height", up_rect.height);

    cJSON_AddNumberToObject(pRoot, "stamp", stamp);
    cJSON_AddItemToObject(pRoot, "rect", pRectArray);
    cJSON_AddItemToObject(pRoot, "up_rect", pUp_rect);

    sendtext = cJSON_Print(pRoot);
    ret = send(connfd, sendtext, strlen(sendtext) + 1, 0);
    //for(i = 0; i < num; i++)
    //{
    //    cJSON_Delete(pRect[i]);
    //}
    //cJSON_Delete(pUp_rect);
    //cJSON_Delete(pRectArray);
    free(sendtext);
    cJSON_Delete(pRoot);
    return ret;

}

int socketconnect(int socketfd)
{
    struct tcp_info info;
    int len = sizeof(info);
    getsockopt(socketfd, IPPROTO_TCP, TCP_INFO, &info, (socklen_t)&len);
    if((info.tcpi_state == TCP_ESTABLISHED))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
