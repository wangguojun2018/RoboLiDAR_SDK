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
 * Robosense detection module.
 *
 */
#ifndef ROBOSENSE_DETECTION_DETECTOR_H
#define ROBOSENSE_DETECTION_DETECTOR_H

#include "detection/ground_estimator/ground_estimator.h"
#include "detection/segmenter/segmenter.h"


namespace Robosense{
    namespace Detection{
        template <typename PointT>
        class Detector:public GroundEstimator<PointT>,public Segmenter<PointT>{
        public:
            typedef pcl::PointCloud<PointT> PointCloud;
            typedef typename PointCloud::Ptr PointCloudPtr;
            typedef typename PointCloud::ConstPtr PointCloudConstPtr;

            Detector(const RoboUsrConfig &usr_config);
            ~Detector(){}

            /**
             * @brief object detection
             * @param[in] in_cloud_ptr input cloud for detection
             * @param[out] clusters,obstacle_cloud_ptr output of detection
             */
            void detector(const PointCloudConstPtr in_cloud_ptr, const float &estimate_lidar_height,
                              std::vector<PointCloudPtr>& clusters, PointCloudPtr obstacle_cloud_ptr);

            /**
             * @brief get ground points
             * @param[in,out] ground_cloud_ptr the result ground cloud of this function
             */
            void getGroundPts(PointCloudPtr ground_cloud_ptr);

        };
    }
}

#endif //ROBOSENSE_DETECTION_DETECTOR_H
