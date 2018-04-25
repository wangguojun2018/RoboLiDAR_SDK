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
 * Version: 2.0
 * Date: 2017.11
 *
 * DESCRIPTION
 *
 * Robosense classification module, for common processing of object classification.
 *
 */

#ifndef ROBOSENCE_CLASSIFY_COMMON_H
#define ROBOSENCE_CLASSIFY_COMMON_H

#include "common/data_type/robo_types.h"
#include "common/box/boxer.h"
#include "common/geometry/geo_base.h"

namespace Robosense
{

template <typename PointT>
class ClassifyUtil
{
public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;


  ClassifyUtil();

  /**
   * @brief post-refiner for classification
   * @param in_percept
   */
  void enhanceClassifier(RoboPerceptron<PointT> &in_percept, const float &estimate_lidar_height);

/**
 * @brief extract segment pointcloud based on bounding box
 * @param[in] in_cloud_ptr: the whole input pointcloud
 * @param[in] box:the bounding box of the segment
 * @param[out] out_cloud_ptr: the pointcloud of the segment
 */
  bool extract_seg_points_crop_box(PointCloudConstPtr in_cloud_ptr,
                                   const BoundingBox &box, PointCloudPtr out_cloud_ptr);


};



}
#endif