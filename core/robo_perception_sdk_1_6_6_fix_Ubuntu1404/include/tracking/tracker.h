
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
 * Robosense tracker module, for fast multi-objects tracking.
 *
 */

#ifndef ROBOSENSE_TRACKER_H
#define ROBOSENSE_TRACKER_H

#include <opencv2/video/tracking.hpp>
#include "common/data_type/robo_types.h"
#include <boost/circular_buffer.hpp>

namespace Robosense
{
/**
 * @brief Kalman tracker Class, construct a tracker with kalman Filter.
 * @ingroup postprocessing
 */

template <typename PointT>
class TargetKalman
{
public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  TargetKalman(const KalmanParms &kalman, int history_buffer_size = 10); //=1 means no history info stored for default set.

  TargetKalman(const TargetKalman &target_kalman);/**<copy constructor*/

  TargetKalman &operator=(const TargetKalman &target_kalman);

  boost::circular_buffer<RoboPerceptron<PointT> > target_buffer_;//save the tracker history for certain length frames, the length setting is 5 now.
  int vanish_time_; // target vanish time counter, counts for delete the vanish target, default is 10 frames, or 1 second for 10 Hz.
  cv::KalmanFilter KF_; //kalman filter for tracking.

  BoundingBox tracker_box_;//record the best bbox of for the object tracked in a tracker.
};


/**
 * @brief Tracking Class, the main class for tracking multi-objects.
 * @ingroup postprocessing
 */
template <typename PointT>
class Tracking
{

public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  /**
   * @brief Constructer.
   * @param[in] use_seq_eva use sequenctial analysis or not, if not, please set false to save calculation time.
   * @param[in] use_global_info use global information like OBD/IMU generated global velocity to refine the detection or not,
   * you'd better use it when OBD/IMU information is provided or used in static scene.
   * @param[in] track_range defines the range of tracking between adjacent frames, the bigger, the higher speed can be tracked
   * @param[in] frame_during_time time between two adjacent frames, is up to lidar working frequency.
   * @param[in] predict_frame_num lost tracker will be predicted for following several frames untill new object are linked again, if not, will be discard;
   * @param[in] history_buffer_size buffer size for historical tracking info retrieval;
   * @param[in] init_track_restrict the restrictions for the init association for a new tracker
   * @param[in] track_prepare_frame the number of pre-tracking evaluation frames for robust tracking
   * @param[in] eva_win_size the number of sequential frames for evaluate the robustness for a tracker.
   */
  Tracking(bool use_seq_eva = false, bool use_global_info = false, float track_range = 3, float frame_during_time = 0.1,
           int predict_frame_num = 5, int history_buffer_size = 10, Point2f init_track_restrict = Point2f(1.f, 0.5f),
           int track_prepare_frame = 3, int eva_win_size = 8);

  /**
   * @brief Setting some user-specific configuration params. Note that the params should be set at init by Constructor, but here you can reset them.
   * unreasonable value will be ignored, and built-in restricted value will be used instead, notice that the restricted value is not equal to the default value.
   */
  void setTrackingParams(bool use_seq_eva = false, bool use_global_info = false, float track_range = 3, float frame_during_time = 0.1,
                         int predict_frame_num = 5, int history_buffer_size = 10, Point2f init_track_restrict = Point2f(1.f, 0.5f),
                         int track_prepare_frame = 3, int eva_win_size = 8);

  /**
   * @brief
   * @param[in, out] percept_vec perception lists after segment process as input/output for tracking.
   * @param[in] obd_velocity global velocity of vehicle self
   * @param[in] estimate_lidar_height lidar_height from the ground estimate by the preprocessing.
   * @param[in] yaw_of_car global direction in x-y plane (yaw) of vehicle self
   */
  void trackMainProcess(std::vector<RoboPerceptron<PointT> > &percept_vec, const float &estimate_lidar_height,
                                  const float &obd_velocity=0, const float &yaw_of_car=0);

  /**
   * @brief reset kalman params for tracking
   */
  void resetKalman(const Point4f &process_noise_conv = Point4f(0.1, 0.1, 0.2, 0.2),
                   const Point4f &measure_noise_conv = Point4f(0.3, 0.3, 0.6, 0.6));

  /**
   * @brief set params for tracking estimate
   * @param params
   */
  void setEstimateParams(const Point4f &params = Point4f(0.4,0.3,0.2,0.1));

  /**
   * @brief get historical track info, which can be used to improve like classification task.
   *
   */
  std::vector<TargetKalman<PointT> > getHistoricalTrackInfos();

  void getBeliefTrackerBoxes(std::vector<BoundingBox> &tracker_boxes);

private:

  void trackTarget(std::vector<RoboPerceptron<PointT> > &head_target, const float &estimate_lidar_height,
                   const float &obd_velocity, const float &yaw_of_car);

  Point2f calcGlobalVelocity(const RoboPerceptron<PointT >& target,
                                               const float &obd_velocity, const float &yaw_of_car);

  float calcShapeScore(const BoundingBox &last, const BoundingBox &current);

  Point3f calcMeasuredVelocity(const RoboPerceptron<PointT> &object, const RoboPerceptron<PointT> &tracklet,
                               const float &estimate_lidar_height, int &link_idx);


  void estimateRobustness(RoboPerceptron<PointT> &object, TargetKalman<PointT> &tracker);

  void refiner(RoboPerceptron<PointT> &object, TargetKalman<PointT> &tracker, float yaw);

  std::vector<TargetKalman<PointT> > tracker_list_;

  float track_range_;

  Point2f init_track_restrict_;

  int track_prepare_frame_count_;
  float frame_during_time_;
  int predict_frame_num_;
  int history_buffer_size_;

  Point4f kalman_process_noise_conv_;
  Point4f kalman_measure_noise_conv_;

  Point4f estimate_praram_;

  int TOTAL_TARGET_NUM;
  int FRAME_COUNT;

  static float asso_score_min_;
  static float valid_velocity_max_;
  static float valid_velocity_min_;
  static float valid_acceleration_max_;
  static float valid_asso_range_max_;
  static float predict_box_width_max_;
  static float predict_box_length_min_;
  static float predict_box_length_max_;
  static float predict_box_height_max_;

  float yaw_old_, delta_yaw_;

  bool use_seq_eva_;
  bool use_global_info_;

  int eva_win_size_;

  std::vector<BoundingBox> prior_boxes_;
};
}//end Robosense namespace

#endif

