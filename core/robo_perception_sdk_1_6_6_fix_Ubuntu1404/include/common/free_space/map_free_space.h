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
 * Version: 1.0.0
 * Date: 2018.3
 *
 * DESCRIPTION
 *
 * Robosense freespace module, for free space detection
 *
 */

#ifndef ROBOSENSE_MAP_FREE_SPACE_H
#define ROBOSENSE_MAP_FREE_SPACE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "common/data_type/robo_types.h"

namespace Robosense{
    /** @brief Class for free space filter
      * @ingroup free_space
      */
    template <typename PointT>
    class MapFreeSpace{
    public:

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        MapFreeSpace();

        /** @brief Set map label for map_free_space
          * @param[in] map_label_path the input label path for map_free_space
          * @ingroup free_space
          */
        bool setMapLabel(const std::string& map_label_path);

        /** @brief filter roi clusters in free space
          * @param[in] in_clusters in clusters for free space filter
          * @param[in] global_trans_mat transfor mat for in clusters
          * @param[out] roi_clusters filter clusters
          * @ingroup free_space
          */
        bool mapFreeSpace(PointCloudConstPtr in_cloud_ptr,
                          const Eigen::Matrix4f& global_trans_mat, PointCloudPtr roi_cloud_ptr, int mode = 0);


        /** @brief filter indices in free space
          * @param[in] in_cloud_ptr in point cloud for free space filter
          * @param[in] global_trans_mat transfor mat for in point clouds
          * @param[out] free_space_indices filter the indices in free space
          * @ingroup free_space
          */
        bool mapFreeSpace(PointCloudConstPtr in_cloud_ptr,
                          const Eigen::Matrix4f& global_trans_mat,std::vector<int>& free_space_indices, int mode = 0);
    protected:
        std::vector<std::vector<cv::Mat> > map_label_;
        Range2D range_;
        float label_img_size_,label_grid_size_;
    private:
    };
}

#endif //ROBOSENSE_MAP_FREE_SPACE_H

/**
 * @brief free_space module
 * @defgroup free_space
 */
