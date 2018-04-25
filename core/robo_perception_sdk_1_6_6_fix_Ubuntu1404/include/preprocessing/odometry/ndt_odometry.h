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
 * Date: 2017.11
 *
 * DESCRIPTION
 *
 * Robosense local odometry estimation module.
 *
 */

#ifndef ROBOSENSE_NDT_ODOMETRY_H
#define ROBOSENSE_NDT_ODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl/registration/ndt.h>
#include <queue>
#include "common/data_type/robo_types.h"

namespace Robosense{
    class NdtOdometry{
    public:
        NdtOdometry();
        ~NdtOdometry(){}

        bool insertCloud(PointCloudConstPtr in_cloud_ptr);

        void getAlignLocalMap(PointCloudPtr map_cloud_ptr);
        bool getAlignPreFrame(const int& idx, PointCloudPtr align_cloud_ptr);
        bool getPreGlobalPose(const int& idx,Eigen::Matrix4f& global_pose);

        bool setAlignNum(const int& num = 5);
        bool setMinShift(const float& shift = 0.1);
        bool setVoxelSize(const float& size = 2.);

    protected:
        int pre_align_num_;
        float voxel_leaf_size_;
        std::queue<PointCloud > pre_global_cloud_;
        std::queue<PointCloud > pre_frame_global_cloud_;
        std::queue<Eigen::Matrix4f> pre_frame_matrix_;
        Eigen::Matrix4f pre_global_pose_,cur_global_pose_;
        PointCloudPtr map_cloud_ptr_;
        pcl::NormalDistributionsTransform<Point, Point> ndt_;
        bool is_map_update_;
        float min_shift_;
        int ndt_max_iter_;
        float ndt_res_;
        double ndt_step_size_,ndt_trans_eps_;
    private:
    };
}

#endif //ROBOSENSE_NDT_ODOMETRY_H
