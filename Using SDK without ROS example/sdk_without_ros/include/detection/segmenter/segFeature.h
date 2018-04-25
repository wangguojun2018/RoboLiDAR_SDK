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

#ifndef ROBOSENSE_DETECTION_SEGFEATURE
#define ROBOSENSE_DETECTION_SEGFEATURE

#include "common/data_type/robo_types.h"
#include <opencv2/opencv.hpp>
#include <stack>

namespace Robosense
{
namespace Detection
{

template <typename PointT>
class SegFeature
{
public:
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;


  SegFeature(const float &refer_vert_gap, const float &estimate_lidar_height = 2.f, const float &refer_div_height = 2.f);

  SegFeature(const SegFeature<PointT>& segf);

  SegFeature& operator=(const SegFeature<PointT>& segf);

  void exec(PointCloudConstPtr cloud);

  std::pair<float, float> max_gap_; //gap 的 起始点和终点高度

  std::vector<std::pair<float, float> > abnormal_gaps_;

  float max_height_;

  float lower_size_height_;
  float upper_size_height_;

  float refer_div_height_;
  float estimate_lidar_height_;
  float refer_vert_gap_;

  bool is_null_;

  PointCloudPtr upper_;
  PointCloudPtr lower_;


  typedef boost::shared_ptr<SegFeature<PointT> > Ptr;
  typedef boost::shared_ptr<const SegFeature<PointT> > ConstPtr;

};


}
}
#endif //ROBOSENSE_DETECTION_SEGFEATURE
