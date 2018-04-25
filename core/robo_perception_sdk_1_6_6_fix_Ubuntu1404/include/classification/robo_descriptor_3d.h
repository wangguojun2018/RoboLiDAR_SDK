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
 * Version: 3.0
 * Date: 2018.02
 *
 * DESCRIPTION
 *
 * Robosense classification module, for object recognition.
 *
 */
#ifndef ROBO_DESCRIPTOR_3D_H
#define ROBO_DESCRIPTOR_3D_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include "common/box/boxer.h"
#include "common/data_type/robo_types.h"

namespace Robosense {

  /**
   * @brief The robosense 3d descriptor module
   * @ingroup postprocessing
   */

class RoboDescriptor3D
{
public:

/**
 * @brief Construct descriptor object
 * @param[in] lidar_config: configure parameter of lidar by user
 */
  RoboDescriptor3D(const float &estimate_lidar_height = 1.9f);

  ~RoboDescriptor3D(){}

  /**
   * @brief compute the 3d descriptor of input for predict
   * @param[in] in_cloud_ptr: input object to be predict
   */
  void computeFeature3D_predict(const NormalRoboCluster& in_cloud_ptr);

  /**
   * @brief compute the 3d descriptor of input for train classifier
   * @param[in] in_cloud_ptr: input object for train
   */
  void computeFeature3D_train(const NormalPointCloudPtr& in_cloud_ptr);

  /**
   * @brief get the feature vector
   * @param[out] feature: output feature vector
   */
  void getFeature(std::vector<float>& feature);

private:
  std::vector<float> normal_his_;
  std::vector<float> normal_dev_;
  std::vector<float> inertia_tensor_;
  std::vector<float> n_inertia_tensor_;
  std::vector<float> cov_mat_;
  std::vector<float> n_cov_mat_;
  std::vector<float> cov_eigenvalue_;
  std::vector<float> n_cov_eigenvalue_;
  std::vector<float> mid_cov_;
  std::vector<float> n_mid_cov_;
  std::vector<float> local_pose_;
  std::vector<float> range_z_;
  std::vector<float> pointnum_;
  std::vector<float> slice_mat_;
  std::vector<float> intensity_statistic_;

private:
  void normalHistorgram(NormalPointCloudConstPtr in_cloud_ptr);
  void normalDevDescriptor(NormalPointCloudConstPtr in_cloud_ptr);
  void calcuLocalPoseNew(const BoundingBox &box);
  void calcuInertiaTensor(NormalPointCloudConstPtr cloud);
  void calcuCovMat(NormalPointCloudConstPtr cloud);
  void calcuCovEigenvalue(const std::vector<float>& cov_mat,std::vector<float>& feature);
  void calcuMidCov(NormalPointCloudPtr cloud);
  void calcuIntensityStatistic(NormalPointCloudPtr cloud);

  bool isInvalidPoint(const pcl::PointXYZINormal &pts);
  bool isInvalidNormalPoint(const pcl::PointXYZINormal &normal);

  float estimate_lidar_height_;
};

}

#endif // ROBO_DESCRIPTOR_3D_H
