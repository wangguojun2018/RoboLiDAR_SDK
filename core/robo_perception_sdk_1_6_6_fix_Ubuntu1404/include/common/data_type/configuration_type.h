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
 * Robosense built-in data structs type —— user defined configuration params container.
 *
 */

#ifndef ROBOSENSE_CONFIGURATION_TYPE_H
#define ROBOSENSE_CONFIGURATION_TYPE_H

#include "common/data_type/basic_types.h"

namespace Robosense
{

/**
 * @brief tracker info container struct
 */
class EIGEN_ALIGN16 RoboUsrConfig
{

public:

  RoboUsrConfig();
  RoboUsrConfig(const RoboUsrConfig &pa);
  RoboUsrConfig &operator=(const RoboUsrConfig &pa);

  void load(const std::string &filename);
  void save(const std::string &filename);

  //lidar
  LidarType lidar_type;

  bool use_tracking;
  bool use_classification;

  //user defined args
  ObjectLimit obj_limit;

  //detection args
  bool is_pre_merge;
  bool is_merge;
  bool is_geo_filter;
  bool is_auto_align;
  bool is_recover_to_vehicle;
  bool use_data_enhance;

  Range2D ground_range;
  Range2D detect_range;
  Range2D ignore_range;

  int min_pts_num;

  //tracking args
  bool track_seq_evaluate;
  bool track_global_refine;
  float track_range;
  int predict_frame_num;

  //classify
  int class_mode;
  bool with_enhance;
  std::string classify0_model_path;
  std::string classify1_model_path;

  //roi filter
  bool use_roi;
  std::string roi_filter_map;

};


class EIGEN_ALIGN16 RoboLidarConfig {

public:
  RoboLidarConfig();
  RoboLidarConfig(const RoboLidarConfig &pa);
  RoboLidarConfig &operator=(const RoboLidarConfig &pa);

  Pose lidar_mount;

  void load(const std::string &filename);
};





}

#endif //ROBOSENSE_CONFIGURATION_TYPE_H
