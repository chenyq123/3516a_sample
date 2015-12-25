
#include "teacher.h"


static int  g_js[4];//每次取数组的坐标，然后过滤完后，出来的发出去的坐标


static int JSgFrameCounter;
static UINT8 JSFrameDif[2][TWIDTH*THEIGHT];//
static UINT8 JSFrameBKTemp[2*10][TWIDTH*100];//
static UINT8 JSFrameBK[2][TWIDTH*THEIGHT];

static UINT8 JSFrameTmp[TWIDTH*THEIGHT];//临时用，输出sobel结果，然后做比对

UINT8 *JSpFrameCur[2];//

TSetting set;

int JSProX[2*JSTARGETCOUNT][TWIDTH];
int JSProXDif[2*JSTARGETCOUNT][TWIDTH];
int JSProXSobel[2][TWIDTH];
int JSProXSobelLow[2][TWIDTH];
//int ProY[4][THEIGHT];
TPoint JSAnglePoint[2][2];
int JSbMyTestFirst=1;
//int thvalue=40;
int JSg_X[2]={0,0};
int JSg_X2[2]={0,0};
int JSgTargetCounterNow;//记录最新分析的坐标的 index


TBlob JSg_Blob[BLOBNUM];
int JSg_BlobCounter=0;
int JSbGetBackGround[2];
int JSg_GetBGCount[2][TWIDTH];
TBlob JSg_BlobTemp[BLOBNUM];
TBlob JSg_BlobTempLast[BLOBNUM];

TBlobShow JSg_BlobShow[2][BLOBNUM];
int JSg_BlobShowCount[2];

int JSg_BGStudyCount[2];


int JSy_up,JSy_down,JSx_left,JSx_right;
int JSX_Width[2];//2个视频合并后的有限区域的宽带
int JSwidthjs;
int JSMinTargetWidth;

int JSFirstFindObject;

int JSTargetOutWaitingCount;
int JSg_XLast[2];
int JSg_X2Last[2];
Bool JSb_FindTargetAndOutCor;//正在找到目标，有坐标，然后计算出来符合区域范围的
//Bool JSb_OutCorFinnal;//最后是否输出坐标
int JSGetRealXY_X[2];//临时输出
int JSg_BGMask[2][TWIDTH];//针对背景中一些亮暗较大的区域，经常会引起误导的检测区域，设置掩区
int JSg_MoreTargetBGResetCount;
int JSTtime=0;//教师离开后坐标保持停留时间

//int JSg_jsbj=0;//0 不取预存背景  1 预存背景


int JSg_NoChangeSplit[2][SPLITNUM];//每一个split是100,没有变化的临时计数器
int JSg_NoChangeSplitMax[2][SPLITNUM];//每一个split是100，累计完全没有变化的次数
int JSg_SplitHaveTarget[2][SPLITNUM];//每一个split是100,该区域是否有目标，根据背景差
int JSg_SplitHaveTargetDif[2][SPLITNUM];//每一个split是100,该区域是否有目标,根据前后幁差分来
int JSg_SplitGetBackGround[2][SPLITNUM];//每一个split是100,启动的时候 是 都没学好的（为了方便分析，第一帧先直接赋值给背景），学背景要快速，一旦学好了，背景更新  就要慢一些
//int JSg_SplitKeepBGTime[2][SPLITNUM];////每一个split是100,没目标时，更新背景的间隔时间(次数，8次为1秒） 计数器
int JSg_NoChangeSplitCounterWithTarget[2][SPLITNUM];	////每一个split是100,当背景差有目标时的累计计数器
int JSg_DifX1[2],JSg_DifX2[2];
int JSBlobCounter[2];

TPointResult JSX_Array[COORNUM];//生成的坐标结果序列，用于控制输出滤波，当场面很乱时，输出全景（无坐标），不要跳来跳去
int JSgFrameOneTwo;



void OsdRect2(unsigned char *pData,int hei,int wid,int x1,int x2,int y1,int y2,int ps)
{
	int i,j,k,m,n;
	//int y_up,y_down;//左上角坐标原点
	//int ps=255;
	memset(pData+y1*wid+x1,ps,x2-x1+1);
	memset(pData+y2*wid+x1,ps,x2-x1+1);

	for(m=y1;m<y2;m++)
	{
		*(pData+m*wid+x1)=ps;
		*(pData+m*wid+x2)=ps;
	}
}

void IMG_sobel
(
    UINT8 *in,   /* Input image data   */
    UINT8 *out,  /* Output image data  */
    short cols, short rows              /* Image dimensions   */
)
{
    int H, O, V, i;
    int i00, i01, i02;
    int i10,      i12;
    int i20, i21, i22;
    int w = cols;

    /* -------------------------------------------------------------------- */
    /*  Iterate over entire image as a single, continuous raster line.      */
    /* -------------------------------------------------------------------- */
    for (i = 0; i < cols*(rows-2) - 2; i++)
    {
        /* ---------------------------------------------------------------- */
        /*  Read in the required 3x3 region from the input.                 */
        /* ---------------------------------------------------------------- */
        i00=in[i    ]; i01=in[i    +1]; i02=in[i    +2];
        i10=in[i+  w];                  i12=in[i+  w+2];
        i20=in[i+2*w]; i21=in[i+2*w+1]; i22=in[i+2*w+2];

        /* ---------------------------------------------------------------- */
        /*  Apply horizontal and vertical filter masks.  The final filter   */
        /*  output is the sum of the absolute values of these filters.      */
        /* ---------------------------------------------------------------- */

        H = -   i00 - 2*i01 -   i02 +
            +   i20 + 2*i21 +   i22;

        V = -   i00         +   i02
            - 2*i10         + 2*i12
            -   i20         +   i22;

        O = abs(H) + abs(V);

        /* ---------------------------------------------------------------- */
        /*  Clamp to 8-bit range.  The output is always positive due to     */
        /*  the absolute value, so we only need to check for overflow.      */
        /* ---------------------------------------------------------------- */
        if (O > 255) O = 255;

        /* ---------------------------------------------------------------- */
        /*  Store it.                                                       */
        /* ---------------------------------------------------------------- */
        out[i + 1] = O;
    }
}

// 高斯平滑
// 1. pImageData   图像数据
// 2. nWidth       图像宽度
// 3. nHeight      图像高度
// 4. nWidthStep   图像行大小
void SmoothGauss(unsigned char *pImageData, int nWidth, int nHeight, int nWidthStep)
{
    int            i            = 0;
    int            j            = 0;
    int            nValue       = 0;
    unsigned char *pLine[3]     = { NULL, NULL, NULL };
    int            nTemplate[9] =
    {
        1, 2, 1,
        2, 4, 2,
        1, 2, 1
    };
    for (j = 1; j < nHeight - 1; j++)
    {
        pLine[0]  = pImageData + nWidthStep * (j - 1);
        pLine[1]  = pImageData + nWidthStep * j;
        pLine[2]  = pImageData + nWidthStep * (j + 1);
        for (i = 1; i < nWidth - 1; i++)
        {
            nValue =
                (pLine[0][i-1] * nTemplate[0] +
                 pLine[0][i]   * nTemplate[1] +
                 pLine[0][i+1] * nTemplate[2] +
                 pLine[1][i-1] * nTemplate[3] +
                 pLine[1][i]   * nTemplate[4] +
                 pLine[1][i+1] * nTemplate[5] +
                 pLine[2][i-1] * nTemplate[6] +
                 pLine[2][i]   * nTemplate[7] +
                 pLine[2][i+1] * nTemplate[8]) / 16;
            pLine[0][i-1] = (unsigned char) nValue;
        }
    }
    //return TRUE;
}

void UpdateBackGround(UINT8* pCur,UINT8* pBK,int xl,int xr,int yu,int yd,int fudu)
{
	int i,j,k;
	if(fudu==0)//all
	{
		for(j=yu;j<yd;j++)
			for(i=xl;i<xr;i++)
			{
				pBK[j*TWIDTH+i]=pCur[j*TWIDTH+i];
			}
	}
	else
	{
		for(j=yu;j<yd;j++)
			for(i=xl;i<xr;i++)
			{
				k=pCur[j*TWIDTH+i]-pBK[j*TWIDTH+i];
				if(k>0)
				{
					//if(k>fudu)

					pBK[j*TWIDTH+i]++;
				}
				else if(k<0)
				{
					pBK[j*TWIDTH+i]--;
				}
			}
	}
}

void JSClearBlobShow()
{
	JSg_BlobShowCount[0]=0;
	JSg_BlobShowCount[1]=0;
}

void JSAddBlobShow(int iFrame,int x1,int x2,int ps)
{
	if((iFrame<0)||(iFrame>1)) return;
	if(JSg_BlobShowCount[iFrame]>=BLOBNUM) return;

	JSg_BlobShow[iFrame][JSg_BlobShowCount[iFrame]].x1=x1;
	JSg_BlobShow[iFrame][JSg_BlobShowCount[iFrame]].x2=x2;
	JSg_BlobShow[iFrame][JSg_BlobShowCount[iFrame]++].ps=ps;
}

int JSGetLastCounter(int cc)
{
	if(cc==0)
		return JSTARGETCOUNT-1;
	else
		return cc-1;
}

void JSRunNextCounter()
{
	JSgTargetCounterNow++;
	if(JSgTargetCounterNow>=JSTARGETCOUNT)
		JSgTargetCounterNow=0;
}

void JSPutCoor()
{
	int i;
	for(i=0;i<COORNUM-1;i++)
	{
		JSX_Array[i]=JSX_Array[i+1];
	}
	JSX_Array[i].pointnum=JSg_BlobCounter;
	JSX_Array[i].x1[0]=JSg_X[0];
	JSX_Array[i].x1[1]=JSg_X[1];
	JSX_Array[i].x2[0]=JSg_X2[0];
	JSX_Array[i].x2[1]=JSg_X2[1];
}

void JSOutput()
{
	int i;
	int one,all,two,xsnum,zero;
	int xcount1,xcount2;
	static int jslast[2]={0,0};
	for(i=0;i<4;i++)
    {
    	g_js[i]=0;
    }
	//教师
    one=0;all=0;two=0;xsnum=0;zero=0;
    xcount1=0;xcount2=0;
    for(i=10;i<COORNUM;i++)
    {
    	if(JSX_Array[i].pointnum==1)
    	{
    		one++;
    		xcount1+=JSX_Array[i].x1[0];
    		//xcount2+=JSX_Array[i].x1[1];
    	}
    	else
    	{
    		if(JSX_Array[i].pointnum==2)
    			two++;

    		if(JSX_Array[i].pointnum==0)
    			zero++;

    		all++;
    	}
    }

    //教师分析
    if(one>all)//单个目标居多
    {
    	xcount1 = xcount1/one;
    	//xcount2 = xcount2/one;
    	g_js[0] = xcount1;//g_js[1]=xcount2;
    	if(jslast[0]>0)//上次一个人的坐标, ||jslast[1]>0
    	{
    		if(abs(xcount1-jslast[0])<1 )//如果差距幁2以内，不动 && abs(xcount2-jslast[1])<2
    		{
    			g_js[0] = jslast[0];//g_js[1]=jslast[1];
    		}
    	}
    }
    else
    {
    	//if(one<two)
    	{
    		g_js[0] = 0;
    		g_js[1] = 0;
    		g_js[2] = 200;
    		g_js[3] = 200;
    	}
    }

    //g_js[0]=JSX_Array[0].x1[0];
    //g_js[1]=JSX_Array[0].x1[1];

    jslast[0]=g_js[0];
    jslast[1]=g_js[1];
}

int JSCheckRealTargetBySobelLow(int FrameNo,int xleft,int xright,int x1,int x2)
{
	int i,w=10,result=0;
	int xx1=(x1-w)>xleft?(x1-w):xleft;
	int xx2=(x2+w)>(xright-1)?(xright-1):(x2+w);
	for(i=xx1;i<xx2;i++)
	{
		if(JSProXSobelLow[FrameNo][i]>2)//有纹理
		{
			result=1;break;
		}
	}
	return result;
}


void JSGetRectCor(int FrameNo)//FrameNo 0~1 uOrder 0~1
{
	JSy_up=JSAnglePoint[FrameNo][0].y;
	JSy_down=JSAnglePoint[FrameNo][1].y;
	JSx_left=JSAnglePoint[FrameNo][0].x;
	JSx_right=JSAnglePoint[FrameNo][1].x;
}


void JSClearSplitHaveTarget(int FrameNo)
{
	int j;
	for(j=0;j<SPLITNUM;j++)
		JSg_SplitHaveTarget[FrameNo][j]=0;
}

void JSSetSplitHaveTarget(int FrameNo,int xleft,int xright,int x1,int x2)
{
	//JSg_SplitHaveTarget
	int j;
	int icount1 = (x1-xleft)/SPLITWIDTH;
	int icount2 = (x2-xleft)/SPLITWIDTH;
	//static
	for(j=icount1;j<=icount2;j++)
	{
		JSg_SplitHaveTarget[FrameNo][j]=1;
	}
}

void JSClearSplitHaveTargetDif(int FrameNo)
{
	int j;
	for(j=0;j<SPLITNUM;j++)
		JSg_SplitHaveTargetDif[FrameNo][j]=0;
}

void JSSetSplitHaveTargetDif(int FrameNo,int xleft,int xright,int x1,int x2)
{
	int j;
	int icount1 = (x1-xleft)/SPLITWIDTH;
	int icount2 = (x2-xleft)/SPLITWIDTH;
	for(j=icount1;j<=icount2;j++)
	{
		JSg_SplitHaveTargetDif[FrameNo][j]=1;
	}
}

void JSInitPara()
{
	int j,k;

	//set.pjs1[0].x = 10; set.pjs1[0].y = 10;
	// set.pjs1[1].x = 100; set.pjs1[0].y = 20;

	JSAnglePoint[0][0].x=set.pjs1[0].x;JSAnglePoint[0][0].y=set.pjs1[0].y;
	JSAnglePoint[0][1].x=set.pjs1[1].x;JSAnglePoint[0][1].y=set.pjs1[1].y;
	//JSAnglePoint[1][0].x=set.pjs2[0].x;JSAnglePoint[1][0].y=set.pjs2[0].y-288;
	//JSAnglePoint[1][1].x=set.pjs2[1].x;JSAnglePoint[1][1].y=set.pjs2[1].y-288;

  	if(JSAnglePoint[0][0].y<0||JSAnglePoint[0][0].y>(THEIGHT-1)||JSAnglePoint[0][1].y<0||JSAnglePoint[0][1].y>(THEIGHT-1)||JSAnglePoint[0][0].y>JSAnglePoint[0][1].y)return;
  	//if(JSAnglePoint[1][0].y<0||JSAnglePoint[1][0].y>(THEIGHT-1)||JSAnglePoint[1][1].y<0||JSAnglePoint[1][1].y>(THEIGHT-1)||JSAnglePoint[1][0].y>JSAnglePoint[1][1].y)return;
//	if(JSAnglePoint[0][0].x<0||JSAnglePoint[0][0].x>(TWIDTH-1)||JSAnglePoint[0][1].x<0||JSAnglePoint[0][1].x>(TWIDTH-1)||JSAnglePoint[0][0].x>JSAnglePoint[0][1].x)return;
//	if(JSAnglePoint[1][0].x<0||JSAnglePoint[1][0].x>(TWIDTH-1)||JSAnglePoint[1][1].x<0||JSAnglePoint[1][1].x>(TWIDTH-1)||JSAnglePoint[1][0].x>JSAnglePoint[1][1].x)return;

	//if(JSy_up<0||JSy_up>(THEIGHT-1)||JSy_down<0||JSy_down>(THEIGHT-1)||JSy_up>JSy_down)continue;
	//if(JSx_left<0||JSx_left>(TWIDTH-1)||JSx_right<0||JSx_right>(TWIDTH-1)||JSx_left>JSx_right)continue;

	if(set.CamWidthjs<=0 )
		JSMinTargetWidth=20;
	else
		JSMinTargetWidth=set.CamWidthjs;

	if(set.widthjs<=0 )
		JSwidthjs=100;
	else
		JSwidthjs=set.widthjs;


	if(set.Ttimejs<=0 )
		JSTtime=10;
	else
		JSTtime=set.Ttimejs;


	JSbMyTestFirst=0;

	JSg_BlobCounter=0;
	JSgTargetCounterNow=0;
	JSFirstFindObject=0;
	JSg_MoreTargetBGResetCount=0;
	JSTargetOutWaitingCount=0;
	JSgFrameCounter=0;

	for(k=0;k<COORNUM;k++)
	{
		JSX_Array[k].pointnum=0;
		JSX_Array[k].x1[0]=0;
		JSX_Array[k].x1[1]=0;
		JSX_Array[k].x2[0]=0;
		JSX_Array[k].x2[1]=0;
	}

	//读取教师背景，用于上电时的跟踪，一旦背景学习完成后，则可不再使用
	//if(JSg_jsbj)
	//	LoadJSBK(JSFrameDif[0]);

	for(k=0;k<1;k++)
	{

		//memset(JSFrameDif[k],255,TWIDTH*THEIGHT);
		//memset(JSFrameBK[k],255,TWIDTH*THEIGHT);

		for(j=0;j<BLOBNUM;j++)
		{
			//JSg_Blob[j].h[k]=0;JSg_Blob[j].y[k]=0;
			JSg_Blob[j].w[k]=0;JSg_Blob[j].x[k]=0;JSg_Blob[j].id=0;
		}

		for(j=0;j<BLOBNUM;j++)
		{
			//JSg_BlobTempLast[j].h[k]=0;JSg_BlobTempLast[j].y[k]=0;
			JSg_BlobTempLast[j].w[k]=0;JSg_BlobTempLast[j].x[k]=0;JSg_BlobTempLast[j].id=0;
		}


		JSGetRectCor(k);
		JSX_Width[k]=JSx_right-JSx_left;

		JSg_BGStudyCount[k]=720;

		JSg_DifX1[k]=0;
		JSg_DifX2[k]=0;
		for(j=0;j<SPLITNUM;j++)
		{
			JSg_NoChangeSplit[k][j]=0;
			JSg_NoChangeSplitMax[k][j]=0;
			JSg_SplitHaveTarget[k][j]=0;
			JSg_SplitHaveTargetDif[k][j]=0;
			JSg_SplitGetBackGround[k][j]=0;
			//JSg_SplitKeepBGTime[k][j]=0;
			JSg_NoChangeSplitCounterWithTarget[k][j]=0;
		}
	}

	/*if(JSg_jsbj)
	{
		Uint32 len1,len2;
		JSGetRectCor(0);
		len1= (JSy_down-JSy_up)*TWIDTH;
		memcpy(JSFrameBK[0]+JSy_up*TWIDTH,JSFrameDif[0],len1);
		JSGetRectCor(1);
		len2= (JSy_down-JSy_up)*TWIDTH;
		memcpy(JSFrameBK[1]+JSy_up*TWIDTH,JSFrameDif[0]+len1,len2);
		//memcpy(JSFrameDif[0],p+TWIDTH*JSy_up,len1);
	}*/
}

void JSPreCtrl(int framenum)
{
	int i,j,k,m,n;
	int thvalue;

	UINT8 *pImageCur,*pImageBK,*pImageBKTemp;

	//JSRunNextCounter();

	//更新临时背景  用于对前后幁差计算

	for(k=0;k<1;k++)
	{
		//m=2*JSgTargetCounterNow+k;
		m=k;
		memset(JSProX[m],0,sizeof(int)*TWIDTH);
		memset(JSProXDif[m],0,sizeof(int)*TWIDTH);
		memset(JSProXSobel[k],0,sizeof(int)*TWIDTH);
		memset(JSProXSobelLow[k],0,sizeof(int)*TWIDTH);

		pImageCur=JSpFrameCur[k];
	    pImageBK=JSFrameBK[k];

	    thvalue=25;//30

	    JSGetRectCor(k);

	    //高速平滑
	    SmoothGauss(pImageCur+(JSy_up-1)*TWIDTH,TWIDTH,JSy_down-JSy_up+2,TWIDTH);

	    //背景差
		for(j=JSy_up;j<JSy_down;j++)
		{
			//n=j*TWIDTH+JSx_left;
			for(i=JSx_left;i<JSx_right;i++)
			{
				//if(abs(pImageCur[n]-pImageBKTemp[n++])>thvalue)
				if(abs(pImageCur[j*TWIDTH+i]-pImageBK[j*TWIDTH+i])>thvalue)
				{
					JSProX[m][i]++;//ProY[2*k][j]++;
				}
			}
		}
		//幁间差
		if(JSgFrameCounter==0)
	    	pImageBKTemp=JSFrameBKTemp[2*9+k];
	    else
	    	pImageBKTemp=JSFrameBKTemp[2*(JSgFrameCounter-1)+k];
		for(j=JSy_up;j<JSy_down;j++)
		{
			//n=j*TWIDTH+JSx_left;
			for(i=JSx_left;i<JSx_right;i++)
			{
				//if(abs(pImageCur[n]-pImageBKTemp[n++])>thvalue)
				if(abs(pImageCur[j*TWIDTH+i]-pImageBKTemp[(j-JSy_up)*TWIDTH+i])>20)
				{
					JSProXDif[m][i]++;//ProY[2*k][j]++;
				}
			}
		}


		IMG_sobel(pImageCur+JSy_up*TWIDTH,JSFrameTmp+JSy_up*TWIDTH,TWIDTH,JSy_down-JSy_up);//当前图像 寻找垂直边缘

		for(j=JSy_up;j<JSy_down-2;j++)//sobel之后，少了后面2行的
		{
			for(i=JSx_left;i<JSx_right;i++)
			{
				if(JSFrameTmp[j*TWIDTH+i]>50)//存在大边缘，
				{
					JSProXSobel[k][i]++;
				}
				if(JSFrameTmp[j*TWIDTH+i]>30)//存在微小边缘，用于辅助背景差（如果前景根本就没有边缘，则不认为是目标，马上当背景）
				{
					JSProXSobelLow[k][i]++;
				}
			}
		}

		//更新临时背景，即上一佂图像保存，用于计算佂间差
		pImageBKTemp=JSFrameBKTemp[2*JSgFrameCounter+k];
		for(j=JSy_up;j<JSy_down;j++)
			for(i=JSx_left;i<JSx_right;i++)
			{
				//pImageBKTemp2[j*TWIDTH+i]=pImageBKTemp[j*TWIDTH+i];
				pImageBKTemp[(j-JSy_up)*TWIDTH+i]=pImageCur[j*TWIDTH+i];
			}

		if(framenum==8)
		{
			UpdateBackGround(pImageCur,pImageBK,JSx_left,JSx_right,JSy_up,JSy_down,0);
		}

	}
	JSgFrameCounter++;
	if(JSgFrameCounter>9)JSgFrameCounter=0;//循环
}

void JSFindTarget()
{
	int i,j,k,w,id,p,pmax;
	int bFindNextBlob;
	UINT8 *pImageCur,*pImageBK,*pImageBKTemp;
	//int BlobCounter[4];
	int HeiValid=2;

	for(k=0;k<1;k++)
	{
		for(j=0;j<BLOBNUM;j++)
		{
			JSg_BlobTemp[j].x[k]=0;JSg_BlobTemp[j].w[k]=0;
		}
		pImageCur=JSpFrameCur[k];
	    pImageBK=JSFrameBK[k];

	    JSGetRectCor(k);

	    //背景差
	    HeiValid = (JSy_down-JSy_up)*3/10;//有效高度
	 	JSBlobCounter[k]=0;
	    for(i=JSx_left;i<JSx_right;i++)
	    {
	    	if(JSProX[k][i]>HeiValid)//2
	    	{
	    		JSg_BlobTemp[JSBlobCounter[k]].x[k]=i-JSx_left;
	    		for(i++;i<JSx_right;i++)
	    		{
	    			if(JSProX[k][i]<HeiValid)//1,
	    			{
	    				//2013-12-20增加判断右侧是否有割裂的框
	    				bFindNextBlob=0;
	    				pmax = (i+30)>JSx_right?JSx_right:(i+30);//教师30个像素的联通
	    				for(p=i+1;p<pmax;p++)
	    				{
	    					if(JSProX[k][p]>HeiValid)
	    					{
	    						bFindNextBlob=1;break;
	    					}
	    				}
	    				if(bFindNextBlob==0)//右边没找到
	    				{
		    				//g_BlobTemp已经是有效区域的x,w
		    				//尽一步确定该区域是否范围过小，有效宽度来判断(4)
		    				if(i-JSx_left-JSg_BlobTemp[JSBlobCounter[k]].x[k] > 4)//5
		    				{
		    					//从轻微差分里的确前景是有纹理的，才认为是找到目标了
		    					if(JSCheckRealTargetBySobelLow(k,JSx_left,JSx_right,JSg_BlobTemp[JSBlobCounter[k]].x[k],i)>0)
		    					{
		    						JSg_BlobTemp[JSBlobCounter[k]].w[k]=i-JSx_left-JSg_BlobTemp[JSBlobCounter[k]].x[k];
		    						JSBlobCounter[k]++;
		    					}
		    					else//更新背景
		    					{
		    						UpdateBackGround(pImageCur,pImageBK,JSg_BlobTemp[JSBlobCounter[k]].x[k],i,JSy_up,JSy_down,1);//更新背景	+1 增量趋势
		    					}
		    					break;//这次查找结束
		    				}
	    				}
	    				else
	    					i=p;
	    			}
	    		}
	    		if(i==JSx_right)//一直到右边都是有物体
	    		{
	    			JSg_BlobTemp[JSBlobCounter[k]].w[k]=JSx_right-JSx_left-JSg_BlobTemp[JSBlobCounter[k]].x[k];
	    			JSBlobCounter[k]++;
	    		}
	    	}
	    	if(JSBlobCounter[k]>=BLOBNUM)
	    		break;
	    }


		//清空 g_SplitHaveTarget
		JSClearSplitHaveTarget(k);

		if(JSBlobCounter[k]==0)//没找到一个BLOB
		{
			for(j=0;j<BLOBNUM;j++)
			{
				JSg_BlobTempLast[j].x[k]=-1;
				JSg_BlobTempLast[j].w[k]=-1;
			}
		}
		else
		{
			////初步判断有物体
			//FindLed(1);
			//BGUpdateCount[k]=0;

			//判断这些分析出来的 区间 ，是否跟上次 有变动，如果有，则可能 幁间差 还没发现的 ，把变化变量g_NoChangeSplit 赋值为0
			//FindAndSetNoChangeSplit(k,x_left,x_right,BlobCounter[k]);

			//清0，然后 赋值，用于下次比对
			for(j=0;j<BLOBNUM;j++)
			{
				JSg_BlobTempLast[j].x[k]=-1;
				JSg_BlobTempLast[j].w[k]=-1;
			}
			for(j=0;j<JSBlobCounter[k];j++)
			{
				int xx1=JSg_BlobTemp[j].x[k]+JSx_left;
				int xx2=JSg_BlobTemp[j].x[k]+JSx_left+JSg_BlobTemp[j].w[k];

				JSg_BlobTempLast[j].x[k]=JSg_BlobTemp[j].x[k];
				JSg_BlobTempLast[j].w[k]=JSg_BlobTemp[j].w[k];

				//计算该目标 在哪个 g_SplitHaveTarget
				JSSetSplitHaveTarget(k,JSx_left,JSx_right,xx1,xx2);

				//投影
				////OsdRect2(pImageCur,THEIGHT,TWIDTH,xx1,xx2,y_up,y_down,255);

				//OsdRect2(pImageCur,THEIGHT,TWIDTH,xx1,xx2,0,JSy_down-JSy_up,255);

				////OsdText(pImageCur,THEIGHT,TWIDTH,50,50,(xx1+xx2)/2);

			}


		}
	}

}

void JSCheckTarget()
{
	int i,j,k,n;

	//总体分析判断跟踪目标
	if(JSBlobCounter[0]==0)
	{
		JSg_BlobCounter=0;
	}
	else if(JSBlobCounter[0]==1)
	{
		JSg_Blob[0]=JSg_BlobTemp[0];
		JSg_BlobCounter=1;
	}
	else
	{
		JSg_Blob[0]=JSg_BlobTemp[0];
		JSg_Blob[1]=JSg_BlobTemp[1];
		JSg_BlobCounter=2;
	}


	/*for(j=0;j<BLOBNUM;j++)
	{
		JSg_BlobTempLast[j]=JSg_BlobTemp[j];
	}*/
}

void JSUpdateImage()
{
	int i,j,k,icount;
	int DisX1,DisX2;
	int ileft,iright;
	int iFudu,iPinlv;//背景更新的幅度与频率
	UINT8 *pImageCur,*pImageBK;
	for(k=0;k<1;k++)
	{
		pImageCur=JSpFrameCur[k];
	    pImageBK=JSFrameBK[k];
		JSGetRectCor(k);

		//根据幁差 计算运动物体的区间 最左边 和  最右边
		JSClearSplitHaveTargetDif(k);
	    DisX1=0;DisX2=0;
	    for(i=JSx_left;i<JSx_right;i++)
	    {
	    	if(JSProXDif[k][i]>1)//
	    	{
	    		DisX1 = i;
	    	}
	    }
	    for(i=JSx_right-1;i>=JSx_left;i--)
	    {
	    	if(JSProXDif[k][i]>1)//
	    	{
	    		DisX2 = i;
	    	}
	    }
	    if(DisX1>0 && DisX2>0)//本视频的运动区域
	    {
	    	JSg_DifX1[k] = (DisX1-5)<JSx_left?JSx_left:(DisX1-5);
	    	JSg_DifX2[k] = (DisX2+5)>(JSx_right-1)?(JSx_right-1):(DisX2+5);
	    	JSSetSplitHaveTargetDif(k,JSx_left,JSx_right,JSg_DifX1[k],JSg_DifX2[k]);
	    }

		//幁差 没有 变化的次数 统计，用来控制 背景的更新
	    //一开始 先赋值背景 但是背景并不稳定 g_SplitGetBackGround[k][j]=false
	    //如果一直都没变动  才会把 g_SplitGetBackGround[k][j]=true

		// 如果 其它地方有运动，没运动的区域 快速变背景
		// 如果 其它地方没有运动， 静态的目标  5分钟后才变背景
		icount = (JSx_right-JSx_left)/SPLITWIDTH;
		for(j=0;j<=icount;j++)
		{
			JSg_NoChangeSplit[k][j]++;
			JSg_NoChangeSplitMax[k][j]++;
			ileft = JSx_left+SPLITWIDTH*j;
			iright = (JSx_left+SPLITWIDTH*(j+1))>JSx_right?JSx_right:(JSx_left+SPLITWIDTH*(j+1));
			for(i=ileft; i<iright; i++)
	    	{
	    		if(JSProXDif[k][i]>1)//幁差
	    		{
	    			JSg_NoChangeSplit[k][j]=0;
	    			JSg_NoChangeSplitMax[k][j]=0;
	    			break;
	    		}
	    	}

	    	//iFudu =0 是最慢的变动；=1 是 直接赋值；=2
	    	//iPinLv ,更新的次数阀值
	    	if(JSg_SplitGetBackGround[k][j]==0)//还没学好背景的，更新的幅度 要大一些（看看频率是否需要快一些）
	    	{
	    		iFudu=2;iPinlv=20;
	    	}
	    	else
	    	{
	    		iFudu=0;iPinlv=20;
	    	}

	    	//g_SplitKeepBGTime[k][j];//暂时不用

	    	if(JSg_NoChangeSplit[k][j]>500 && JSg_SplitGetBackGround[k][j]==0)//还没学好背景的//更新背景 次数 100次相当于 5秒
			{
				UpdateBackGround(pImageCur,pImageBK,ileft,iright,JSy_up,JSy_down,0);//直接更新背景
				JSg_NoChangeSplit[k][j]=0;
			}

			if(JSg_NoChangeSplit[k][j]>500 && JSg_SplitGetBackGround[k][j]==1)////已经学好背景的
			{
				UpdateBackGround(pImageCur,pImageBK,ileft,iright,JSy_up,JSy_down,1);//更新背景	+1 增量趋势
				JSg_NoChangeSplit[k][j]=10;//如果一直没变化，就加快更新背景频率
			}

			//没学好背景时，要求较长时间 没变化了，才认为是 学好了
			if(JSg_SplitGetBackGround[k][j]==0 && JSg_NoChangeSplitMax[k][j]>1000)
			{
				JSg_SplitGetBackGround[k][j]=1;//认为学好了
			}

			//静止目标转背景，长时间（20秒）一点都不动，但是在目标范围（背景差判断的）内 ，当前幁 直接赋值 给 背景
			/*if(g_NoChangeSplitMax[k][j]>2000 && g_SplitHaveTarget[k][j]>0)
			{
				UpdateBackGround(pImageCur,pImageBK,ileft,iright,y_up,y_down,0);//直接更新背景
				g_NoChangeSplitMax[k][j]=0;
			}*/


			//静止目标 一直有目标 而且是 没有幁差的,长时间 才 强制恢复 背景
			if(JSg_SplitHaveTarget[k][j]>0 && JSg_NoChangeSplit[k][j]>0)
			{
				JSg_NoChangeSplitCounterWithTarget[k][j]++;
			}
			else
			{
				JSg_NoChangeSplitCounterWithTarget[k][j]=0;
			}
			if(JSg_NoChangeSplitCounterWithTarget[k][j]>1500)//累计超过1500次约3分钟
			{
				UpdateBackGround(pImageCur,pImageBK,ileft,iright,JSy_up,JSy_down,0);//直接更新背景
				JSg_NoChangeSplitCounterWithTarget[k][j]=0;
			}
			/////


			//show status
			//OsdText(pImageCur,THEIGHT,TWIDTH,ileft,0,JSg_SplitGetBackGround[k][j]);	//顶部
			//OsdText(pImageCur,THEIGHT,TWIDTH,ileft+55,0,JSg_NoChangeSplitMax[k][j]);	//中线下面
		}


	}

	////如果该视频区域  有运动的区域，其它无运动的区域 要加速变背景
	icount=0;
	for(k=0;k<1;k++)
	{
		for(j=0;j<SPLITNUM;j++)
		{
			if(JSg_SplitHaveTargetDif[k][j]>0 && JSg_SplitHaveTarget[k][j]>0)
				icount++;
		}
	}
	if(icount>0)//有这样的区域，既找到背景差，又有幁差
	{
		for(k=0;k<1;k++)
		{
			for(j=0;j<SPLITNUM;j++)
			{
				if(JSg_SplitHaveTargetDif[k][j]==0 && JSg_SplitHaveTarget[k][j]>0)//幁差为0，但是背景差不为0的区域
				{
					if(JSg_SplitGetBackGround[k][j]>0)//学好背景的了，更新要更快一些
					{
						JSg_NoChangeSplit[k][j]+=100;
						JSg_NoChangeSplitMax[k][j]+=100;
					}
					else//还没学好背景 的
					{
						JSg_NoChangeSplit[k][j]+=20;
						JSg_NoChangeSplitMax[k][j]+=20;
					}
				}

			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////将来考虑   g_BlobTempLast 背景未学习完之前  运动区域 移动后，原来的 区域 根据纹理 来填补（如果跟非目标区域 是顺的，没有纹理，则可以马上填补，
	// 主要针对那些 错把人 作为背景 而且本身背景是理想没有纹理 的情况）
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//分析目标边缘区域，前景 没有边缘，而 背景是有边缘，
	// 获取前景目标 的 边缘，如果 无边缘，则 加快 背景更新；如果有边缘 则 延长 背景更新，这是 辅助手段
	//for(j=0;j<)

}

void JSGetRealXY(int x1)
{
	JSGetRealXY_X[0]=100*x1/JSX_Width[0];
	JSGetRealXY_X[1]=1;
}

void JSRecognize()
{
	int x1,x2,x3=0,x4=0;
	UINT8 *pImageCur=JSpFrameCur[0];
	//if(JSbNotGetBG[0]==1 && JSbNotGetBG[1]==1)
	if(JSg_BlobCounter>0)
	{
		JSGetRectCor(0);
		x1=JSg_Blob[0].x[0]+JSg_Blob[0].w[0]/2;//-JSx_left
		if(JSg_BlobCounter>1)
			x3=JSg_Blob[1].x[0]+JSg_Blob[1].w[0]/2;	//-JSx_left

		JSGetRealXY(x1);//x2
		//OsdText(pImageCur,THEIGHT,TWIDTH,100,200,JSX_Width[0]);
		//
		//OsdText(pImageCur,THEIGHT,TWIDTH,200,200,x1);

		JSg_X[0]=JSGetRealXY_X[0];JSg_X[1]=JSGetRealXY_X[1];

		if(JSg_BlobCounter>1)
		{
			JSGetRealXY(x3);//x2
			JSg_X2[0]=JSGetRealXY_X[0];JSg_X2[1]= JSGetRealXY_X[1];

			if((JSg_X[0]==0)&&(JSg_X2[0]>0))
			{
				JSg_X[0]=JSg_X2[0];JSg_X[1]=JSg_X2[1];
				JSg_X2[0]=0;JSg_X2[1]=0;
			}

		}

		JSFirstFindObject=1;

		/*if(((JSNotGetBGCor[0][1]-JSNotGetBGCor[0][0])>(JSONEWITTH+JSMinTargetWidth))||((JSNotGetBGCor[1][1]-JSNotGetBGCor[1][0])>(JSONEWITTH+JSMinTargetWidth)))
		{
			JSg_X2[0]=1;JSg_X2[1]=1;
		}*/
	}
}

Bool JSCheckTargetOutWaiting()
{
	if(JSFirstFindObject==0)
		return FALSE;
	if(JSb_FindTargetAndOutCor)
	{
		JSTargetOutWaitingCount=0;
		JSg_XLast[0]=JSg_X[0];
		JSg_XLast[1]=JSg_X[1];
		JSg_X2Last[0]=JSg_X2[0];
		JSg_X2Last[1]=JSg_X2[1];
		return TRUE;
	}
	else
	{
		JSTargetOutWaitingCount++;
		if(JSTargetOutWaitingCount<10*1)//JSTtime
		{
			JSg_X[0]=JSg_XLast[0];
			JSg_X[1]=JSg_XLast[1];
			JSg_X2[0]=JSg_X2Last[0];
			JSg_X2[1]=JSg_X2Last[1];
			return TRUE;
		}
		else
			return FALSE;
	}
}

int JSFindNearBlobNo(TBlob ArrayBlob[],int BlobNum,TBlob oldBlob,int Col)
{
	int j,min,old,find,res;
	old=oldBlob.x[Col]+oldBlob.w[Col]/2;
	min=ArrayBlob[0].x[Col]+ArrayBlob[0].w[Col]/2;res=0;
	for(j=1;j<BlobNum;j++)
	{
		find=ArrayBlob[j].x[Col]+ArrayBlob[j].w[Col]/2;
		if(abs(old-find)<abs(old-min))
		{
			min=find;
			res=j;
		}
		else
			break;
	}
	return res;
}

void JSSetparam(int x1,int y1,int x2,int y2)
{
	set.pjs1[0].x = x1; set.pjs1[0].y = y1;
	set.pjs1[1].x = x2; set.pjs1[1].y = y2;
	JSbMyTestFirst = 1;
}



char* JSMyTest(UINT8 *pFrame,char *str)
{
	static int JSVideoNo=1;
//	static float JSfdelta=0.03;
	JSg_X[0]=0,JSg_X[1]=0;JSg_X2[0]=0;JSg_X2[1]=0;JSb_FindTargetAndOutCor=FALSE;
	JSpFrameCur[0]=pFrame;
	JSpFrameCur[1]=pFrame+TWIDTH*THEIGHT;
	if(JSbMyTestFirst)
	{
        JSbMyTestFirst = 0;
		JSInitPara();
		JSg_X[0]=0,JSg_X[1]=0;
		JSVideoNo=0;
		return NULL;
	}

	//OsdRect2(pFrame,THEIGHT,TWIDTH,set.pjs1[0].x,set.pjs1[1].x,set.pjs1[0].y,set.pjs1[1].y,255);

	JSVideoNo++;
	JSPreCtrl(JSVideoNo);	//背景差，幁差

	if(JSVideoNo<30)
        return NULL;

	JSVideoNo=100;

	JSFindTarget(); //根据投影 找blob 存到 JSg_BlobTemp

	JSCheckTarget(); //分析blob 的 逻辑关系，判断输出目标

    //return "3333";

	JSUpdateImage(); //判断和控制 背景更新

	//JSShowWhiteRec();

	JSRecognize();//计算角度

	//if(JSCheckTargetOutWaiting())//符合要求和延时要求
	{
	}
	//else
	{
	//	JSg_X[0]=0;JSg_X[1]=0;JSg_X2[0]=0;JSg_X2[1]=0;
	}

	if((JSVideoNo<2000) && (JSg_BlobCounter>1)){
		JSg_BlobCounter=1;
		if((JSg_X[0]>0) && (JSg_X2[0]>0)){// 比较一下 哪个跟上一个 接近的，只保留 接近的那个坐标
			if(abs(g_js[0]-JSg_X[0])>abs(g_js[0]-JSg_X2[0])){
				JSg_X[0]=JSg_X2[0];
			}
		}
		JSg_X2[0]=0;JSg_X2[1]=0;
	}

	JSPutCoor();
	JSOutput();//g_js[0],g_js[1]
    //sprintf(str,"%d,%d,%d,%d",JSg_X[0],JSg_X[1],JSg_X2[0],JSg_X2[1]);
	sprintf(str,"%d,%d",g_js[0],JSg_BlobCounter);
    return str;
}

