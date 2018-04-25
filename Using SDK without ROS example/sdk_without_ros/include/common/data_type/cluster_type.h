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
 * Date: 2018.3
 *
 * DESCRIPTION
 *
 * Robosense built-in data structs type —— clustered segment.
 *
 */

#ifndef ROBOSENSE_CLUSTER_TYPE_H
#define ROBOSENSE_CLUSTER_TYPE_H

#include "common/data_type/basic_types.h"
#include "common/data_type/box_type.h"

namespace Robosense
{
/**
 * @brief cluster container struct
 */
template <typename PointT>
class EIGEN_ALIGN16 RoboCluster //segmentation module IO structure
{
public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  RoboCluster();

  RoboCluster(const RoboCluster<PointT> &pc);

  RoboCluster &operator=(const RoboCluster<PointT> &pc);

  int object_id; /**< object ID, which will be set after clustering, then the id will be used as a commonly global identity for
											every object in the procedure following such as tracking and classification and their match for fusion*/

  BoundingBox box;/**< original bounding-box for a segmented point cluster*/

  BoundingBox semantic_box;/**< refined bounding-box for an recognized object*/

  Point3f barycenter; /**< gravity center of pointcloud */

  bool is_infered; /**< true if infered success */

  PointCloudPtr pointCloud; /**< pointcloud for a segmented cluster*/


  typedef boost::shared_ptr<RoboCluster<PointT> > Ptr;
  typedef boost::shared_ptr<const RoboCluster<PointT> > ConstPtr;

};


};

#endif //ROBOSENSE_CLUSTER_TYPE_H
