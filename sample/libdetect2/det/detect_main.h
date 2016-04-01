#ifndef DETECT_MAIN_H__
#define DETECT_MAIN_H__

typedef struct zifImage
{
	//PixelFormat fmt_type;
	int width;
	int height;
	unsigned char *data[4];
	int stride[4];

	double stamp;

	void *internal_ptr;
} zifImage;

void det_close(void *ins);
const char *det_detect(void *det, zifImage *img);
void *det_open(const char *cfg_name);
void det_set_flipped_mod(void *det, int enabled);
void det_set_param(void *det, int thres_dis, int thres_area, double factor_0, double factor_05, double notused);
void det_source_stats(void *det, int total, int lost, int pending, int cached);

#endif
