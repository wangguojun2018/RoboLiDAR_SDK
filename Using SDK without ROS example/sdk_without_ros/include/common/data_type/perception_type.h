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
 * Robosense built-in data structs type —— percepted object, ensemble of segmentation, detection, classification and tracking etc..
 *
 */

#ifndef ROBOSENSE_PERCEPTION_TYPE_H
#define ROBOSENSE_PERCEPTION_TYPE_H

#include "common/data_type/basic_types.h"
#include "common/data_type/cluster_type.h"
#include "common/data_type/target_type.h"
#include "common/data_type/obj_type.h"

namespace Robosense
{

/**
 * @brief tracker info container struct
 */
template <typename PointT>
class EIGEN_ALIGN16 RoboPerceptron
{
public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  RoboPerceptron();

  RoboPerceptron(const RoboPerceptron<PointT> &per);

  RoboPerceptron &operator=(const RoboPerceptron<PointT> &per);

  RoboTarget tracker; /**<tracker*/
  RoboObject labeler; /**<classifier*/
  RoboCluster<PointT> cluster;   /**<cluster for the tracked object.*/

  bool is_segmented;/**<if the Target is segmented*/
  bool is_tracked;/**<if the Target is tracked*/
  bool is_classified;/**<if the Target is classified*/
  bool is_real;/**<if the Target is real or virtual (such as predicted object)*/
  bool is_background; /**< if is background, the flag will be set true. */

  BoundingBox infer_box;

  typedef boost::shared_ptr<RoboPerceptron<PointT> > Ptr;
  typedef boost::shared_ptr<const RoboPerceptron<PointT> > ConstPtr;

};


}


#endif //ROBOSENSE_PERCEPTION_TYPE_H
