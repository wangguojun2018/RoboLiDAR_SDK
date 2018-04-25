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
 * Robosense planefit module.
 *
 */

#ifndef ROBOSENSE_COMMON_GEOMETRY_GROUND_PLANEFIT_H
#define ROBOSENSE_COMMON_GEOMETRY_GROUND_PLANEFIT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "common/data_type/robo_types.h"

namespace Robosense{

template <typename PointT>
class GroundPlanefit
{

public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  typedef pcl::SampleConsensusModelPlane<PointT> SampleConsensusModelPlane;
  typedef typename SampleConsensusModelPlane::Ptr SampleConsensusModelPlanePtr;
  typedef pcl::RandomSampleConsensus<PointT> RandomSampleConsensus;

  GroundPlanefit(const float lidar_height = 1.9f);
  ~GroundPlanefit(){}

  void setGroundPlanefitParams(const Range2D& range = Range2D(-16.,16.,-10.,10.));
  bool groundPlanefit(const PointCloudConstPtr in_cloud_ptr, Eigen::VectorXf& plane_model, float &estimate_lidar_height);
  void setConstrainParams(const float& plane_thre = 0.2, const float& abs_thre = 0.6f, const float& norm_thre = 0.9);

protected:

  inline float pt2range(const PointT& pt);
  inline float pt2beta(const PointT& pt);
  inline int ptx2grid(const float& val);
  inline int pty2grid(const float& val);
  inline int rowcol2grid(const int& row,int& col);
  inline int pptx2grid(const float& val);
  inline int ppty2grid(const float& val);
  inline int prowcol2grid(const int& row,int& col);
  Range2D range_;
  float grid_size_,prange_step_,prad_step_,max_range_;
  float plane_thre_,norm_thre_;
  int rows_,cols_,prows_,pcols_;
  float abs_thre_;
private:
  float lidar_height_;
};
}


#endif //ROBOSENSE_COMMON_GEOMETRY_GROUND_PLANEFIT_H
