#include <stdio.h>
#include "math.h"
unsigned int float_to_s25q7(float num)
{
    unsigned int a = 0;
    a |= (int)num << 7;
    a |= (int)((fabsf(num) - (int)fabsf(num)) * 128 ) & 0x7F;
    printf("%x\n", a);
    return a;
}
float s25q7_to_float(unsigned int num)
{
    float a = (int)num >> 7;
    if(a <  0)
        a -= ((int)(num & 0x7F)) / 128.0;
    else
        a += ((int)(num & 0x7F)) / 128.0;
//    printf("%d\n",((int)(num & 0x7F)));
}
float s9q7_to_float(unsigned short num)
{
    float a = (short)num >> 7;
    if(a < 0)
        a -= ((short)(num & 0x7F)) / 128.0;
    else
        a += ((short)(num & 0x7F)) / 128.0;
    printf("%f\n",a);
    printf("%f\n",((int)(num & 0x7F))/128.0);
}
int main()
{
    float a = -100.635;
    s9q7_to_float((unsigned short)float_to_s25q7(a));
    return 0;
}
