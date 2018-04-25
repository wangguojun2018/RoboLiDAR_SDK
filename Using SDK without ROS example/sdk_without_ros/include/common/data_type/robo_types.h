
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
 * Robosense built-in data structs type definition module.
 *
 */

#ifndef ROBOSENSE_TYPES_H
#define ROBOSENSE_TYPES_H

#include "common/data_type/basic_types.h"
#include "common/data_type/box_type.h"
#include "common/data_type/cluster_type.h"
#include "common/data_type/obj_type.h"
#include "common/data_type/target_type.h"
#include "common/data_type/perception_type.h"
#include "common/data_type/configuration_type.h"

namespace Robosense
{

typedef RoboCluster<pcl::PointXYZINormal> NormalRoboCluster;
typedef RoboPerceptron<pcl::PointXYZINormal> NormalRoboPerceptron;

typedef RoboCluster<pcl::PointXYZI> PtsRoboCluster;
typedef RoboPerceptron<pcl::PointXYZI> PtsRoboPerceptron;


template<typename T>
std::string num2str(T num, int precision)
{
  std::stringstream ss;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(precision);
  std::string st;
  ss << num;
  ss >> st;

  return st;
}



}



#endif //ROBOSENSE_TYPES_H
