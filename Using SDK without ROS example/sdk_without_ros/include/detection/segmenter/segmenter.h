/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 1.6.5
 * Date: 2018.2
 *
 * DESCRIPTION
 *
 * Robosense cluster module.
 *
 */
#ifndef ROBOSENSE_DETECTION_SEGMENTER_H
#define ROBOSENSE_DETECTION_SEGMENTER_H

#include "common/data_type/robo_types.h"
#include <opencv2/opencv.hpp>
//#include "detection/segmenter/segFeature.h"

namespace Robosense{
    namespace Detection{

//    struct CellFeature{
//
//      bool has_gap;
//      Point2f max_gap;
//      float max_height;
//
//      float lower_size_z;
//      float upper_size_z;
//
//      int lower_size_num;
//      int upper_size_num;
//    };

        template <typename PointT>
        class Segmenter{
        public:
            typedef pcl::PointCloud<PointT> PointCloud;
            typedef typename PointCloud::Ptr PointCloudPtr;
            typedef typename PointCloud::ConstPtr PointCloudConstPtr;

            Segmenter(const RoboUsrConfig &usr_config);
            ~Segmenter(){}

            /**
             * @brief cluster
             * @param[in] in_cloud_ptr input cloud for clustering
             * @param[out] objects output objects
             */
            void segmenter(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height, std::vector<PointCloudPtr>& out_objects);

            void segmenter2(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height, std::vector<PointCloudPtr>& out_objects);

            void segmenter3(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height, std::vector<PointCloudPtr>& out_objects);

            /**
             * @brief set cluster params
             * @param[in] range,grid_size,clutser_z,growth_z,growth_ref_dis,dilate_size params set for clustering
             */
            void setSegmenterParams(const Range2D& range = Range2D(-50.,50.,-50.,50.),
                                    const float& grid_size = 0.2f, const float& clutser_z = 2.f,
                                    const float& growth_z = 4.4f, const float& growth_ref_dis = 10.f,
                                    const int& dilate_size = 1);

            /**
             * @brief set cluster restrict params
             * @param[in] restrict_pts params set for clustering restrict size
             */
            void setSegmenterRestrictPts(const int& restrict_pts = 3);

            void setUserDefineSegmentXYScan(Eigen::Vector2f (*pf)(const PointT&) = NULL);
            void setUserDefineSegmentGrowthZScan(float (*pf)(const PointT&) = NULL);
            void setUserDefineSegmentGrowthXYScanScan(Eigen::Vector2f (*pf)(const PointT&) = NULL);

        protected:

            bool segmentRefiner(const std::vector<PointCloudPtr>& raw_segments, const float& lidar_h,
                                std::vector<PointCloudPtr>& refine_segments, const float& gap = 0.5f);

//            void calcCellFeature(PointCloudConstPtr cloud, CellFeature& feature, const float& refer_vert_gap, const float& refer_div_h,
//                                 const float &estimate_lidar_height, const float& gap = 0.5f);

            void setConditionMat();
            Eigen::Vector2f (*computeSegmentXYScan_)(const PointT& pt) = NULL;
            Eigen::Vector2f computeSegmentXYScan(const PointT& pt);

            float (*computeSegmentGrowthZScan_)(const PointT& pt) = NULL;
            float computeSegmentGrowthZScan(const PointT& pt);

            Eigen::Vector2f (*computeSegmentGrowthXYScan_)(const PointT& pt) = NULL;
            Eigen::Vector2f computeSegmentGrowthXYScan(const PointT& pt);

            inline void cluster(const cv::Mat& bin_mat,cv::Mat& label_mat);

            inline void growth(const cv::Mat& label_mat, const cv::Mat& gap_mat,
                               const std::vector<cv::Mat>& growth_bin_mat,
                               std::vector<cv::Mat>& growth_label_mat);

            inline void growth2(const cv::Mat& label_mat, const cv::Mat& gap_mat,
                                const cv::Mat& project_mat,
                             const std::vector<cv::Mat>& growth_bin_mat,
                             std::vector<cv::Mat>& growth_label_mat);

            inline bool isNanPt(const PointT& pt);
            bool isReasonableRange2D(const Range2D& range);
            Range2D range_;
            float grid_size_,growth_z_,growth_step_,cluster_z_;
            int rows_,cols_,growth_slice_,dilate_size_;
            std::set<std::pair<int,int> > merge_;
            int restrict_min_pts_;

            cv::Mat scan_x_mat_,scan_y_mat_,scan_growth_z_mat_,scan_growth_x_mat_,scan_growth_y_mat_;

        private:

            RoboUsrConfig usr_config_;
            float hori_res_, vert_res_;
            float hori_grown_limit_, vert_grown_limit_;
            float division_;
            int total_obj_num_;

        };
    }
}

#endif //ROBOSENSE_DETECTION_SEGMENTER_H
