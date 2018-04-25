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
 * Robosense geometry tranformation module.
 *
 */

#ifndef ROBOSENSE_TRANSFORMER_H
#define ROBOSENSE_TRANSFORMER_H

#include "common/data_type/robo_types.h"
#include "common/geometry/geo_base.h"
#include "common/free_space/map_free_space.h"

#define WITH_SORTED 0

namespace Robosense{


class Transformer{

public:

  /**
   * @brief involving the common geometry transformation operations together
   * @param lidar_config
   * @param args
   */
  Transformer(const RoboLidarConfig& lidar_config, const RoboUsrConfig& args);

  /**
   * @brief ensemble process for pointcloud
   * @param in_cloud_ptr
   * @param out_cloud_ptr
   * @return
   */
  bool preprocess(PointCloudConstPtr in_cloud_ptr, NormalPointCloudPtr out_cloud_ptr);

  /**
   * @brief calibration the original pointcloud according the lidar mounting configuration params
   * @param in_cloud_ptr
   * @param out_cloud_ptr
   * @return
   */
  bool initCalibration(PointCloudConstPtr in_cloud_ptr, PointCloudPtr out_cloud_ptr);

  /**
   * @brief transport pointcloud from lidar coord to vehicle coord
   * @param in_cloud_ptr
   * @param out_cloud_ptr
   * @return
   */
  bool transFromLidar2Vehicle(NormalPointCloudConstPtr in_cloud_ptr, NormalPointCloudPtr out_cloud_ptr);

  /**
   * @brief transport pointcloud from vehicle coord to lidar coord
   * @param in_cloud_ptr
   * @param out_cloud_ptr
   * @return
   */
  bool transFromVehicle2Lidar(NormalPointCloudConstPtr in_cloud_ptr, NormalPointCloudPtr out_cloud_ptr);

  /**
   * @brief add normal information
   * @param pcloud_in
   * @param pcloud_out
   * @param with_true_normal
   * @return
   */
  bool addNormAndCurve(PointCloudConstPtr pcloud_in, NormalPointCloudPtr pcloud_out, bool with_true_normal = true);

  /**
  * @brief automatically adjust the pointcloud, make it parallel to horizon plane
  * @param in_cloud_ptr input cloud
  * @param align_cloud_ptr refined output
  * @return procesedure result, true means success, false means calibration failed.
  */
  bool autoAlign(PointCloudConstPtr in_cloud_ptr, PointCloudPtr align_cloud_ptr, bool use_auto_align = true);

  /**
   * @brief recovery perception result into original or vehicle coord
   * @param percept_list
   * @return
   */
  bool recoverPerceptResult(std::vector<RoboPerceptron<NormalPoint> > &percept_list);

  /**
   * @brief recovery processed pointcloud into original or vehicle coord
   * @param in_cloud_ptr
   * @param recover_cloud_ptr
   * @return
   */
  bool recoverPointcloud(NormalPointCloudConstPtr in_cloud_ptr, NormalPointCloudPtr recover_cloud_ptr);


  /**
   * @brief clip input cloud by range parameters
   * @param in_cloud original input cloud
   * @param out_cloud cliped output
   * @return procesedure result, true means success, false means calibration failed.
   */
  bool applyRange(NormalPointCloudConstPtr in_cloud, NormalPointCloudPtr out_cloud, bool use_range = false);

  /**
   * @brief roi region filter for cloud
   * @param in_cloud original input cloud
   * @param out_cloud filtered output
   * @return procesedure result, true means success, false means calibration failed.
   */
  bool applyRoi(NormalPointCloudConstPtr in_cloud, NormalPointCloudPtr out_cloud, bool use_roi = false);

  /**
   * @brief set transform matrix between vehicle and global
   * @param vehicle2global_mat
   */
  void setVehicle2GlobalMat(const Eigen::Matrix4f &vehicle2global_mat){
    vehicle2global_mat_ = vehicle2global_mat;
  }

  void getVehicleCloud(NormalPointCloudPtr vehicle_cloud_ptr) const {*vehicle_cloud_ptr = *vehicle_cloud_ptr_;}

  float getLidarHightEstimate() const { return estimate_lidar_height_; }

private:

  bool isInValidRange(const NormalPoint &p);

  MapFreeSpace<NormalPoint>* map_free_space_;

  Range2D detect_range_;
  Range2D ignore_range_;

  RoboUsrConfig args_;
  RoboLidarConfig lidar_config_;

  Eigen::Matrix4f vehicle2global_mat_;
  Eigen::Matrix4f auto_rot_mat_, auto_rot_mat_inv_;
  Eigen::Matrix4f lidar2vehicle_trans_mat_, vehicle2lidar_trans_mat_;
  Eigen::Matrix4f init_trans_mat_;

  NormalPointCloudPtr vehicle_cloud_ptr_;

  float estimate_lidar_height_;

};





}







#endif //ROBOSENSE_TRANSFORMER_H
