#pragma once

#include <opencv2/opencv.hpp>
#include <deque>
#include "KVConfig.h"
#include "utils.h"
#include "hi_opencv.h"

/// ∂‡÷°÷°≤Ó ...
class DiffMotion
{
    int cnt_;
    std::deque<cv::Mat> diffs_;
    int diff_threshold_;
    cv::Mat ker_erode_, ker_dilate_;

public:
    explicit DiffMotion(KVConfig *cfg)
    {
        int es = atoi(cfg->get_value("dm_erode_size", "5"));
        int ds = atoi(cfg->get_value("dm_dilate_size", "15"));

        ker_erode_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(es, es));
        ker_dilate_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ds, ds));

        cnt_ = atoi(cfg->get_value("dm_cnt", "3"));
        diff_threshold_ = atoi(cfg->get_value("dm_threshold", "28"));

        //log_file("DiffMotion config:\n");
        //log_file("\tdm_erode_size: %d, dm_dilate_size:%d\n", es, ds);
        //log_file("\tdm_cnt: %d\n", cnt_);
        //log_file("\tdm_threshold: %d\n", diff_threshold_);
        //log_file("\n");
    }

    ~DiffMotion()
    {
    }

    std::vector<cv::Rect> get_motions(const cv::Mat &prev_gray, const cv::Mat &next_gray)
    {
        cv::Mat diff;
        //
        //cv::absdiff(prev_gray, next_gray, diff);
        hi_absdiff(prev_gray, next_gray, diff);
        //
        //cv::threshold(diff, diff, diff_threshold_, 255, cv::THRESH_BINARY);
        hi_threshold(diff, diff, diff_threshold_, 255, cv::THRESH_BINARY);

        diffs_.push_back(diff);
        while ((int)diffs_.size() > cnt_) {
            diffs_.pop_front();
        }

        return sum_motions();
    }

    void reset()
    {
        diffs_.clear();
    }



private:
    std::vector<cv::Rect> sum_motions() const
    {
        printf("diffs.size:%d, cnt_:%d\n",diffs_.size(), cnt_);
        assert(diffs_.size() == cnt_);
        assert(cnt_ >= 1);

        cv::Mat sum = diffs_[0];
        for (size_t i = 1; i < diffs_.size(); i++) {
            cv::bitwise_or(sum, diffs_[i], sum);
        }

        //
        //cv::erode(sum, sum, ker_erode_);
        hi_erode(sum, sum, ker_erode_);
        //
        //cv::dilate(sum, sum, ker_dilate_);
        hi_dilate(sum, sum, ker_dilate_);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(sum, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        std::vector<cv::Rect> rcs;
        for (size_t i = 0; i < contours.size(); i++) {
            rcs.push_back(cv::boundingRect(contours[i]));
        }

        return rcs;
    }
};
