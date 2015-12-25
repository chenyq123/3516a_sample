
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "MyCore.h"


#define UINT8 unsigned char
#define TWIDTH 720
#define THEIGHT 576
#define TWIDTHUV 360
#define BLOBNUM 4
#define COORNUM 20

///////总长度 最大为 720
#define SPLITNUM 10
#define SPLITWIDTH 100

#define JSTARGETCOUNT 5

#define JSONEWITTH 50  //50

char* JSMyTest(UINT8 *pFrame,char* str);
void JSSetparam(int x1,int y1,int x2,int y2);
