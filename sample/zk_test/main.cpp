#include "studentDetectOP.h"
#include <iostream>
#include <unistd.h>
#include "StudentTrack.h"

#include "opencv2/opencv.hpp"
#include "opencv2/legacy/legacy.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

long t1, t2, t3, t4, t5, t6, t7, t8, t9;
long flowcount;

int main()
{
    CStudentTrack sdop;
    Mat img;
    sdop.setduration(30);
    sdop.setdebug(true);
    sdop.start();
    while(true)
    {
        sdop.process(img);
        //bGetFrame = false;
        t1 = sdop.t1;
        t2 = sdop.t2;
        t3 = sdop.t3;
        t4 = sdop.t4;
        t5 = sdop.t5;
        t6 = sdop.t6;
        t7 = sdop.t7;
        t8 = sdop.t8;

        t9 = GetTickCount() - sdop.tt;

        usleep(2);
    }
    return 0;
}
