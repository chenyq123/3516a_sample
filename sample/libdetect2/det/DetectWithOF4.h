#pragma once
#include "detect.h"
#include <string.h>
#include "hi_ive.h"
#include "hi_comm_ive.h"
#include "mpi_ive.h"
#include "hi_comm_sys.h"
#include "mpi_sys.h"

/** 使用稀疏光流，持续跟踪目标，当连续 N 秒没有较大移动时，则认为本次跟踪结束

 */
class DetectWithOF4 : public Detect
{
	typedef std::vector<cv::Point> CONTOUR;	// 轮廓
	typedef std::vector<cv::Point2f> FEATURES;	// 特征点

	/** 描述一个对象的连续活动

			对象的创建：
				根据帧差，当出现较大帧差时，腐蚀膨胀后，得到轮廓，然后为每个轮廓创建一个 MotionObject，在MotionObject范围内找“特征点”开始跟踪；

			对象的跟踪：
				计算稀疏光流，跟踪，聚合主要方向的特征点，删除分离的特征点；及时补充特征点

			对象的销毁：
				当特征点数目少于阈值，或特征点的移动速度小于阈值后，认为目标已经停止，则销毁；
	 */
	class MotionObject
	{
		DetectWithOF4 *parent_;
		std::vector<CONTOUR> contours_hist_;	//
		std::vector<cv::Point2f> mean_pt_hist_;	// 运动历史，就是中心点的位置？？
		int max_features_, min_features_;
		FEATURES curr_features_;
		bool moving_;
		cv::Mat prev_;
		double stamp_update_;

	public:
		MotionObject(DetectWithOF4 *parent, const cv::Rect &brc, const cv::Mat &curr_gray) : parent_(parent)
		{
			max_features_ = 300;
			min_features_ = 10;

			CONTOUR region;
			region.push_back(brc.tl());
			region.push_back(cv::Point(brc.x, brc.y+brc.height));
			region.push_back(brc.br());
			region.push_back(cv::Point(brc.x+brc.width, brc.y));

			curr_features_ = update_features(max_features_, curr_features_, region, curr_gray);
			moving_ = curr_features_.size() > min_features_;
			prev_ = curr_gray;

			if (moving_) {
				contours_hist_.push_back(region);
				mean_pt_hist_.push_back(mean_pt(curr_features_));
			}
		}

		bool is_moving() const { return moving_; }

		void track(const cv::Mat &gray)
		{
			FEATURES next_pts;
			cv::Mat status, err;
#if 1
            HI_S32 ret;
            IVE_HANDLE IveHandle;
            IVE_SRC_IMAGE_S stSrcPre;
            IVE_SRC_IMAGE_S stSrcCur;
            IVE_SRC_MEM_INFO_S stPoint;
            IVE_SRC_MEM_INFO_S stMv;
            IVE_LK_OPTICAL_FLOW_CTRL_S stLKOptiFlowCtrl;
            HI_BOOL bInstant = HI_TRUE;
            HI_BOOL bFinish;
            static int _isfirst = 1;

            stSrcPre.u16Width = prev_.cols;
            stSrcPre.u16Height = prev_.rows;
            stSrcPre.u16Stride[0] = prev_.cols;
            stSrcPre.enType = IVE_IMAGE_TYPE_U8C1;
            ret = HI_MPI_SYS_MmzAlloc_Cached(&stSrcPre.u32PhyAddr[0], (void**)&stSrcPre.pu8VirAddr[0], "user", HI_NULL, prev_.rows * prev_.cols);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
                //goto TRACK_END1;
            }
            memcpy(stSrcPre.pu8VirAddr[0], prev_.data, prev_.rows * prev_.cols);

            stSrcCur.u16Width = gray.cols;
            stSrcCur.u16Height = gray.rows;
            stSrcCur.u16Stride[0] = gray.cols;
            stSrcCur.enType = IVE_IMAGE_TYPE_U8C1;
            ret = HI_MPI_SYS_MmzAlloc_Cached(&stSrcCur.u32PhyAddr[0], (void**)&stSrcCur.pu8VirAddr[0], "user", HI_NULL, gray.rows * gray.cols);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
                //goto TRACK_END2;
            }
            memcpy(stSrcCur.pu8VirAddr[0], gray.data, gray.rows * gray.cols);

            stLKOptiFlowCtrl.u16CornerNum = curr_features_.size();
            stLKOptiFlowCtrl.u0q8MinEigThr = 30;
            stLKOptiFlowCtrl.u8IterCount = 10;
            stLKOptiFlowCtrl.u0q8Epsilon = 2;

            ret = HI_MPI_SYS_MmzAlloc_Cached(&stPoint.u32PhyAddr, (void**)&stPoint.pu8VirAddr, "user", HI_NULL, stLKOptiFlowCtrl.u16CornerNum * sizeof(IVE_POINT_S25Q7_S));
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
               // goto TRACK_END3;
            }
            stPoint.u32Size = stLKOptiFlowCtrl.u16CornerNum * sizeof(IVE_POINT_S25Q7_S);
            for(size_t i = 0; i < stLKOptiFlowCtrl.u16CornerNum; i++)
            {
                ((IVE_POINT_S25Q7_S *)(stPoint.pu8VirAddr))[i].s25q7X = float_to_s25q7(curr_features_[i].x);
                ((IVE_POINT_S25Q7_S *)(stPoint.pu8VirAddr))[i].s25q7Y = float_to_s25q7(curr_features_[i].y);
            }
           // if(_isfirst == 1)
            //{
                ret = HI_MPI_SYS_MmzAlloc_Cached(&stMv.u32PhyAddr, (void**)&stMv.pu8VirAddr, "user", HI_NULL, stLKOptiFlowCtrl.u16CornerNum * sizeof(IVE_MV_S9Q7_S));
                if(ret != HI_SUCCESS)
                {
                    printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
                }
                stMv.u32Size = stLKOptiFlowCtrl.u16CornerNum * sizeof(IVE_MV_S9Q7_S);
                //_isfirst = 0;
           // }
#endif
            //printf("PLK begin\n");
#if 1
            ret = HI_MPI_IVE_LKOpticalFlow(&IveHandle, &stSrcPre, &stSrcCur, &stPoint, &stMv, &stLKOptiFlowCtrl, bInstant);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_IVE_LKOpticalFlow failed with error code %#x\n",ret);
            }

            ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_IVE_Query failed with error code %#x\n",ret);
            }
#endif
			//cv::calcOpticalFlowPyrLK(prev_, gray, curr_features_, next_pts, status, err);
            //printf("PLK over\n");

            for(size_t i = 0; i < stLKOptiFlowCtrl.u16CornerNum; i++)
            {
                cv::Point2f poi;
                status.push_back((char)((IVE_MV_S9Q7_S *)(stMv.pu8VirAddr))[i].s32Status);
                poi.x = s25q7_to_float(((IVE_POINT_S25Q7_S *)(stPoint.pu8VirAddr))[i].s25q7X) + s9q7_to_float(((IVE_MV_S9Q7_S *)(stMv.pu8VirAddr))[i].s9q7Dx);
                poi.y = s25q7_to_float(((IVE_POINT_S25Q7_S *)(stPoint.pu8VirAddr))[i].s25q7Y) + s9q7_to_float(((IVE_MV_S9Q7_S *)(stMv.pu8VirAddr))[i].s9q7Dy);
                next_pts.push_back(poi);
            }

			cv::Mat s = status.reshape(1, 1), e = status.reshape(1, 1);
			unsigned char *ps = s.ptr<unsigned char>(0), *pe = s.ptr<unsigned char>(0);

			FEATURES features;
			for (size_t i = 0; i < status.cols; i++) {
				if (ps[i] == 1) {
					features.push_back(next_pts[i]);
				}
			}

			if (features.size() < min_features_) {
				moving_ = false;
				return;
			}

			remove_stray(features);	//
			if (features.size() < min_features_) {
				curr_features_ = update_features(max_features_, features, get_contour(features), gray);
			}
			else {
				curr_features_ = features;
			}

			prev_ = gray;	//
TRACK_END1:
            ret = HI_MPI_SYS_MmzFree(stMv.u32PhyAddr, stSrcPre.pu8VirAddr);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }
TRACK_END2:
            ret = HI_MPI_SYS_MmzFree(stPoint.u32PhyAddr, stPoint.pu8VirAddr);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }

TRACK_END3:
            ret = HI_MPI_SYS_MmzFree(stSrcCur.u32PhyAddr[0], stSrcCur.pu8VirAddr[0]);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }

TRACK_END4:
            ret = HI_MPI_SYS_MmzFree(stSrcPre.u32PhyAddr[0], stSrcPre.pu8VirAddr[0]);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }

		}

		cv::Rect get_curr_brc() const
		{
			return cv::boundingRect(curr_features_);
		}

		void draw_hist(cv::Mat &img) const
		{
			if (mean_pt_hist_.size() > 0) {
				cv::circle(img, mean_pt_hist_[0], 2, cv::Scalar(0, 0, 255));
			}

			for (size_t i = 1; i < mean_pt_hist_.size(); i++) {
				cv::line(img, mean_pt_hist_[i-1], mean_pt_hist_[i], cv::Scalar(0, 0, 255));
			}

			for (size_t i = 0; i < curr_features_.size(); i++) {

			}
		}

	private:
        //
        unsigned int float_to_s25q7(float num)
        {
            unsigned int a = 0;
            a |= (int)num << 7;
            a |= (int)((fabsf(num) - (int)fabsf(num)) * 128 ) & 0x7F;
            return a;
        }
        float s25q7_to_float(unsigned int num)
        {
            float a = (int)num >> 7;
            if(a <  0)
                a -= ((int)(num & 0x7F)) / 128.0;
            else
                a += ((int)(num & 0x7F)) / 128.0;
            return a;
        }
        float s9q7_to_float(unsigned short num)
        {
            float a = (short)num >> 7;
            if(a < 0)
                a -= ((short)(num & 0x7F)) / 128.0;
            else
                a += ((short)(num & 0x7F)) / 128.0;
            return a;
        }

		// 返回 features 的轮廓
		CONTOUR get_contour(const FEATURES &pts)
		{
			CONTOUR tmp, c;
			for (size_t i = 0; i < pts.size(); i++) {
				tmp.push_back(pts[i]);
			}

			cv::convexHull(tmp, c);
			return c;
		}

		// 平均点的位置
		cv::Point2f mean_pt(const FEATURES &pts) const
		{
			double sx = 0.0, sy = 0.0;
			for (size_t i = 0; i < pts.size(); i++) {
				sx += pts[i].x, sy += pts[i].y;
			}
			cv::Rect all(0, 0, parent_->video_width_, parent_->video_height_);
			cv::Point2f m = cv::Point2f(sx/pts.size(), sy/pts.size());
			if (m.x < 0) m.x = 0.1;
			if (m.y < 0) m.y = 0.1;
			if (m.x >= parent_->video_width_-0.1) m.x = parent_->video_width_-1;
			if (m.y >= parent_->video_height_-0.1) m.y = parent_->video_height_-1;
			return m;	// 平均位置
		}

		// 到平均点的距离
		double mean_dis(const cv::Point2f &mean_pt, const FEATURES &pts) const
		{
			double sd = 0.0;
			for (size_t i = 0; i < pts.size(); i++) {
				sd += _distance(mean_pt, pts[i]);
			}
			return sd / pts.size();
		}

		// 删除离群点
		void remove_stray(FEATURES &pts)
		{
			if (pts.size() < 3) {
				return;
			}

			/** 首先计算所有点的中心，计算所有点到中心的距离，删除超过平均距离的点
			 */
			cv::Point2f mean = mean_pt(pts);
			double mdis = mean_dis(mean, pts);

			for (int i = pts.size()-1; i >= 0; i--) { // 从后往前删除
				if (_distance(pts[i], mean) > 2 * mdis) {
					pts.erase(pts.begin() + i);
				}
			}
		}

		// 更新
		FEATURES update_features(int max, const FEATURES &old, const CONTOUR &region, const cv::Mat &curr_gray)
		{
			FEATURES features;
			cv::Rect brc = cv::boundingRect(region);
#if 1
            IVE_HANDLE IveHandle;
            IVE_SRC_IMAGE_S stSrc;
            IVE_DST_IMAGE_S stCandiCorner;
            IVE_DST_MEM_INFO_S stCorner;
            IVE_ST_CANDI_CORNER_CTRL_S stStCandiCornerCtrl;
            IVE_ST_CORNER_CTRL_S stStCornerCtrl;
            HI_BOOL bInstant = HI_TRUE;
            HI_BOOL bFinish;
            SYS_VIRMEM_INFO_S stMemInfo;
            HI_U32 u32BlkSize;
            HI_S32 ret;
            int relwidth,relheight;
            cv::Mat temp;

            temp = curr_gray(brc);
            relwidth = temp.cols < 64 ? 64 : temp.cols;
            relheight = temp.rows < 64 ? 64 : temp.rows;
            if(relwidth % 16 != 0)
                relwidth += 16 - relwidth % 16;
            if(relheight %2 != 0)
                relheight += 1;
            //printf("width:%d,height:%d\n",relwidth,relheight);

            ret = HI_MPI_SYS_MmzAlloc_Cached(&stSrc.u32PhyAddr[0], (void**)&stSrc.pu8VirAddr[0], "user", HI_NULL, relwidth * relheight);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
            }

            memcpy(stSrc.pu8VirAddr[0], temp.data, temp.rows * temp.cols);

            stSrc.enType = IVE_IMAGE_TYPE_U8C1;
            //stSrc.pu8VirAddr[0] = temp.data;
            stSrc.u16Stride[0] = relwidth;
            stSrc.u16Width = relwidth;
            stSrc.u16Height = relheight;
            //printf("rows:%d,cols:%d\n",temp.cols,temp.rows);
            //printf("rows1:%d,cols1:%d\n",curr_gray(brc).cols,curr_gray(brc).rows);
#if 0
            ret = HI_MPI_SYS_GetVirMemInfo(stSrc.pu8VirAddr[0], &stMemInfo);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_GetVirMemInfo failed with error code:%#x\n",ret);
                return features;
            }
            printf("Get mem success!\n");
            stSrc.u32PhyAddr[0] = stMemInfo.u32PhyAddr;
#endif
            stStCandiCornerCtrl.u0q8QualityLevel = 13;
            u32BlkSize = 4 * stSrc.u16Stride[0] * stSrc.u16Height + sizeof(IVE_ST_MAX_EIG_S);

            ret = HI_MPI_SYS_MmzAlloc_Cached(&stStCandiCornerCtrl.stMem.u32PhyAddr,
                    (void**)&stStCandiCornerCtrl.stMem.pu8VirAddr, "user", HI_NULL, u32BlkSize);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
            }

            HI_MPI_SYS_MmzFlushCache(stStCandiCornerCtrl.stMem.u32PhyAddr, (void*)stStCandiCornerCtrl.stMem.pu8VirAddr, u32BlkSize);
            stStCandiCornerCtrl.stMem.u32Size = u32BlkSize;

            stCandiCorner.u16Width = stSrc.u16Width;
            stCandiCorner.u16Height = stSrc.u16Height;
            stCandiCorner.u16Stride[0] = stSrc.u16Stride[0];
            stCandiCorner.enType = IVE_IMAGE_TYPE_U8C1;

            ret = HI_MPI_SYS_MmzAlloc_Cached(&stCandiCorner.u32PhyAddr[0], (void**)&stCandiCorner.pu8VirAddr[0], "user", HI_NULL, relwidth * relheight);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
            }
            HI_MPI_SYS_MmzFlushCache(stCandiCorner.u32PhyAddr[0], (void*)stCandiCorner.pu8VirAddr[0], relwidth * relheight);

            memset(stCandiCorner.pu8VirAddr[0], 0, relwidth * relheight);

            ret = HI_MPI_IVE_STCandiCorner(&IveHandle, &stSrc, &stCandiCorner, &stStCandiCornerCtrl, bInstant);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_IVE_STCandiCorner failed with error code %#x\n",ret);
            }

            ret = HI_MPI_IVE_Query(IveHandle, &bFinish, HI_TRUE);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_IVE_Query failed with error code %#x\n",ret);
            }

            //stStCornerCtrl.u16MaxCornerNum = max - old.size();
            stStCornerCtrl.u16MaxCornerNum = 200;
            stStCornerCtrl.u16MinDist = 3;

            ret = HI_MPI_SYS_MmzAlloc_Cached(&stCorner.u32PhyAddr, (void**)&stCorner.pu8VirAddr, "user", HI_NULL, sizeof(IVE_ST_CORNER_INFO_S));
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzAlloc_Cached failed with error code %#x\n",ret);
            }
            HI_MPI_SYS_MmzFlushCache(stCorner.u32PhyAddr, (void*)stCorner.pu8VirAddr, sizeof(IVE_ST_CORNER_INFO_S));
            stCorner.u32Size = sizeof(IVE_ST_CORNER_INFO_S);

            ret = HI_MPI_IVE_STCorner(&stCandiCorner, &stCorner, &stStCornerCtrl);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_IVE_STCandiCorner failed with error code %#x\n",ret);
            }
            cv::Point2f poi;
            //printf("stCorner.u16CornerNum:%d\n",(*((IVE_ST_CORNER_INFO_S *)&stCorner)).u16CornerNum);
            //printf("stCorner.u16CornerNum :%u\n",((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->u16CornerNum);
            for(size_t i = 0; i < ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->u16CornerNum; i++)
            {
                //poi.x = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->astCorner[i].u16X;
                //poi.y = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->astCorner[i].u16Y;
                unsigned short x = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->astCorner[i].u16X;
                unsigned short y = ((IVE_ST_CORNER_INFO_S *)stCorner.pu8VirAddr)->astCorner[i].u16Y;
                if(x != temp.cols && y != temp.rows)
                {
                    poi.x = x;
                    poi.y = y;
                    features.push_back(poi);
                }
            }
#endif
            //printf("max - old.size():%d\n",max - old.size());
			//cv::goodFeaturesToTrack(curr_gray(brc), features, max - old.size(), 0.05, 1.0);
            //printf("features.size:%d\n",features.size());
			for (size_t i = 0; i < features.size(); i++) {
				features[i].x += brc.x;
				features[i].y += brc.y;
			}

			// 合并 old
			for (size_t i = 0; i < old.size(); i++) {
				if (!has_same_feature_pt(features, old[i])) {
					features.push_back(old[i]);
				}
			}
#if 1
            ret = HI_MPI_SYS_MmzFree(stStCandiCornerCtrl.stMem.u32PhyAddr,
                    stStCandiCornerCtrl.stMem.pu8VirAddr);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }

            ret = HI_MPI_SYS_MmzFree(stCorner.u32PhyAddr, stCorner.pu8VirAddr);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }

            ret = HI_MPI_SYS_MmzFree(stSrc.u32PhyAddr[0], stSrc.pu8VirAddr[0]);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }

            ret = HI_MPI_SYS_MmzFree(stCandiCorner.u32PhyAddr[0], stCandiCorner.pu8VirAddr[0]);
            if(ret != HI_SUCCESS)
            {
                printf("HI_MPI_SYS_MmzFree failed with error code %#x\n",ret);
            }
#endif
			return features;
		}

		bool has_same_feature_pt(const FEATURES &fs, const cv::Point2f &pt)
		{
			for (size_t i = 0; i < fs.size(); i++) {
				if (distance(pt, fs[i]) < 1.0) {
					return true;
				}
			}
			return false;
		}

		inline double distance(const cv::Point2f &p0, const cv::Point2f &p1) const
		{
			return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
		}
	};
	friend class MotionObject;
	std::vector<MotionObject> motion_objects_;


	/** 描述一个目标，目标由一个或者多个 MotionObject 构成，如一个人，可能有躯干和四肢的组合运动得到

			创建：
				????
	 */
	class Target
	{
	public:
		Target()
		{
		}
	};

	double threshold_diff_;	// 缺省 25
	cv::Mat ker_erode_, ker_dilate_;

public:
	DetectWithOF4(KVConfig *cfg);
	~DetectWithOF4(void);

private:
	virtual std::vector<cv::Rect> detect0(size_t st_cnt, cv::Mat &origin, cv::Mat &gray_prev, cv::Mat &gray_curr, cv::vector<int> &dirs);
	std::vector<cv::Rect> get_diff_rects(const cv::Mat &prev, const cv::Mat &curr)	// 根据帧差，腐蚀膨胀，然后找轮廓，返回轮廓外接矩形
	{
		cv::Mat diff;
		cv::absdiff(prev, curr, diff);
		cv::threshold(diff, diff, threshold_diff_, 255.0, cv::THRESH_BINARY);

		cv::erode(diff, diff, ker_erode_);
		cv::dilate(diff, diff, ker_dilate_);

		//cv::imshow("ed", diff);

		std::vector<CONTOUR> contours;
		cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

		std::vector<cv::Rect> rcs;
		for (size_t i = 0; i < contours.size(); i++) {
			rcs.push_back(cv::boundingRect(contours[i]));
		}

		return rcs;
	}
};

