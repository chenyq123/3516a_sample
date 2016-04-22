#pragma once

#include <opencv2/opencv.hpp>
#include <time.h>
#include <string>
#include <stdarg.h>
#include <stdio.h>

inline unsigned long GetTickCount()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/// 两点之间的距离
inline double distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

inline std::string pos2str(const cv::Rect &rc)
{
    char info[128];
    snprintf(info, sizeof(info), "[%d,%d  %d,%d]", rc.x, rc.y, rc.width, rc.height);
    return std::string(info);
}

extern std::string _log_fname;

inline void log_file(const char *fmt, ...)
{
    va_list args;
    char buf[1024];

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    FILE *fp = fopen(_log_fname.c_str(), "at");
    if (fp) {
        time_t now = time(0);
        struct tm *ptm = localtime(&now);
        fprintf(fp, "%02d:%02d:%02d.%03d: %s",
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec, GetTickCount() % 1000, buf);
        fclose(fp);
    }
}

inline void log_init_file(const char *fname = 0)
{
    if (fname)
        _log_fname = fname;

    FILE *fp = fopen(_log_fname.c_str(), "w");
    if (fp) {
        fprintf(fp, "------ log begin ---------\n");
        fclose(fp);
    }
}

inline void log_init_dummy(const char *fname = 0)
{
}

inline void log_dummy(const char *fmt, ...)
{
}
