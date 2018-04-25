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
 * Robosense sdk proxy module, integrated entrance for all functional modules including detection, tracking and classification, etc.
 *
 */

#ifndef ROBOSENSE_ALL_H
#define ROBOSENSE_ALL_H


#include "preprocessing/preprocessing.h"
#include "detection/detector.h"
#include "tracking/tracker.h"
#include "classification/classifier0.h"
#include "classification/classifier1.h"
#include "common/wrapper/box_wrapper.h"
#include "common/geometry/transformer.h"
#include "classification/classifier2.h"


namespace Robosense
{

/**
 * @brief integrated entrance for sdk with map
 * @ingroup robosense
 */
class RobosenseALL
{

public:

  RobosenseALL(const std::string &lidar_config_path, const std::string &usr_config_path);

  ~RobosenseALL();

  /**
   * @brief all processing entrance, including ground-detection, segmentation, tracking, classification
   */
  void mainPerceptionProcess(PointCloudConstPtr in_cloud_ptr, const float &obd_velocity=0, const float &yaw_of_car=0,
                             Eigen::Matrix4f v2g_mat = Eigen::Matrix4f::Identity());

  /**
   * @brief get ground points
   * @param ground_cloud_ptr
   */
  void getGroundPoints(NormalPointCloudPtr ground_cloud_ptr);

  /**
   * @brief get obstacle object points
   * @param object_cloud_ptr
   */
  void getObjectPoints(NormalPointCloudPtr object_cloud_ptr);

  /**
   * @brief get original pointcloud tranformed in vehicle coordinate
   * @param vehicle_cloud_ptr
   */
  void getCloudInVehicle(NormalPointCloudPtr vehicle_cloud_ptr);


  /**
   * @brief return the segmentation results after refine.
   * @return
   */
  std::vector<NormalRoboPerceptron> &getObjectPeceptResults();

  /**
   * @brief return the original segmentation results.
   * @return
   */
  std::vector<NormalRoboPerceptron> &getObjectOriPeceptResults();

  /**
   * @brief return the collected last result in lidar coordinate for users
   * @return
   */
  std::vector<PerceptOutput> &getLastResultsForUser();


private:

  PreProcess *preProcesser_;

  Transformer* transformer_;

  Robosense::Detection::Detector<NormalPoint>  *detector_;

  Tracking<NormalPoint> *tracker_;

  Classifier0 *classifier0_;
  Classifier1 *classifier1_;
  Classifier2 *classifier2_;

  BoxWrapper<NormalPoint> *box_wrapper_;

  NormalPointCloudPtr norm_cloud_ptr_;
  NormalPointCloudPtr object_cloud_ptr_;
  NormalPointCloudPtr ground_cloud_ptr_;
  NormalPointCloudPtr vehicle_cloud_ptr_;

  std::vector<NormalRoboPerceptron> percept_vec_, percept_vec_filtered_;
  std::vector<PerceptOutput> output_list_;

  RoboUsrConfig args_;
  RoboLidarConfig lidar_config_;

  std::vector<BoundingBox> tracker_boxes_;

};


}

#endif //ROBOSENSE_ALL_H
