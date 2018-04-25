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
 * Robosense box wrappering module.
 *
 */

#ifndef ROBOSENSE_BOX_WRAPER_H
#define ROBOSENSE_BOX_WRAPER_H

#include "common/box/boxer.h"
#include "common/data_type/robo_types.h"

namespace Robosense
{

template<typename PointT>
class BoxWrapper
{

public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  BoxWrapper(const RoboUsrConfig& args);

  /**
   * @brief wrapper the detection results
   * @param[in] segment_vec input cluster
   * @param[out] cluster_vec output result
   * @return flag to indicate the process is succeed or failed
  */
  bool wrapper(const std::vector<PointCloudPtr> &segment_vec, std::vector<RoboPerceptron<PointT> > &cluster_vec);

  void getOriCluster(std::vector<RoboPerceptron<PointT> > &cluster_ori_vec);

  void freshLidarHeight(const float &estimate_lidar_height);

  void feedPriorBoxes(const std::vector<BoundingBox> &prior_boxes);

protected:

  bool wrapBox(const std::vector<PointCloudPtr> &segment_vec, std::vector<RoboPerceptron<PointT> > &cluster_vec);

  bool mergeInnerBox(const std::vector<RoboPerceptron<PointT> > &in_clusters, std::vector<RoboPerceptron<PointT> > &out_clusters);

  bool mergeInferBox(const std::vector<RoboPerceptron<PointT> > &in_clusters, std::vector<RoboPerceptron<PointT> > &out_clusters);

  bool mergeBox(const std::vector<RoboPerceptron<PointT> > &in_clusters, std::vector<RoboPerceptron<PointT> > &out_clusters);

  bool mergePriorBox(const std::vector<RoboPerceptron<PointT> > &in_clusters, std::vector<RoboPerceptron<PointT> > &out_clusters);

  bool mergePriorBox(const std::vector<PointCloudPtr> &segment_vec, std::vector<PointCloudPtr> &segment_vec_out);

  bool segmentFilter(const std::vector<RoboPerceptron<PointT> > &in_clusters, std::vector<RoboPerceptron<PointT> > &out_clusters);

  BoundingBoxCalculator<PointT> *box_calculator_;
  std::vector<RoboPerceptron<PointT> > cluster_ori_vec_;

  RoboUsrConfig args_;

  float estimate_lidar_height_;

  std::vector<BoundingBox> prior_boxes_;

  static int OBJ_NUM;

};
}

#endif //ROBOSENSE_BOX_WRAPER_H
