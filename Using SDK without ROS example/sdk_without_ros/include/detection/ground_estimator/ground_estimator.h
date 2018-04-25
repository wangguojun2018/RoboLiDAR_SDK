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
 * Robosense ground remove module.
 *
 */
#ifndef ROBOSENSE_DETECTION_GNDEST_H
#define ROBOSENSE_DETECTION_GNDEST_H

#include "common/data_type/robo_types.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace Robosense{
    namespace Detection{
        template <typename PointT>
        class GroundEstimator{
        public:
            typedef pcl::PointCloud<PointT> PointCloud;
            typedef typename PointCloud::Ptr PointCloudPtr;
            typedef typename PointCloud::ConstPtr PointCloudConstPtr;
            typedef pcl::SampleConsensusModelPlane<PointT> SampleConsensusModelPlane;
            typedef typename SampleConsensusModelPlane::Ptr SampleConsensusModelPlanePtr;
            typedef pcl::RandomSampleConsensus<PointT> RandomSampleConsensus;

            GroundEstimator();
            ~GroundEstimator(){}

            /**
             * @brief ground remove use linefit method
             * @param[in] in_cloud_ptr input cloud for ground remove
             * @param[out] objects output objects
             */
            void groundEstimator(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height, PointCloudPtr obstacle_cloud_ptr);
            /**
             * @brief get ground points
             * @param[in,out] ground_cloud_ptr ground point cloud of ground
             */
            void getGroundPts(PointCloudPtr ground_cloud_ptr);

            void setGndEstConstrainParams(const float& height_thre = 0.2,const float& angle_thre = 5.);
            void setGndEstParams(const Range2D& range = Range2D(-50.f, 50, -25.f, 25.f),
                                     const float& angle_step = 0.5, const float& range_step = 0.2);
            void setGndConstrainParams(const float& thre_ = 0.2, const float& abs_thre = 0.5f);

        protected:

            inline int ptx2grid(const float& val);
            inline int pty2grid(const float& val);
            inline int rowcol2grid(const int& row,const int& col);
            inline float comRange(const PointT& pt);
            inline float comBeta(const PointT& pt);
            inline float pt2plane(const PointT& pt,const Eigen::VectorXf& plane_model);
            inline float pt2pt(const PointT& pt1, const PointT& pt2);
            inline float locateInPlane(const PointT& pt,const Eigen::VectorXf& plane_model);
            inline bool isNanPt(const PointT& pt);
            bool isReasonableRange2D(const Range2D& range);
            float max_radius_,rad_step_,range_step_;
            Range2D range_;
            int rows_,cols_;
            float height_thre_,rad_thre_;
            float ground_plane_thre_;
            float abs_thre_;
            PointCloudPtr ground_cloud_ptr_;
        };
    }
}
#endif //ROBOSENSE_DETECTION_GNDEST_H
