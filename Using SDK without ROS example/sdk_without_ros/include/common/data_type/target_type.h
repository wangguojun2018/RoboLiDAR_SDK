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
 * Robosense built-in data structs type —— tracked target.
 *
 */

#ifndef ROBOSENSE_TARGET_TYPE_H
#define ROBOSENSE_TARGET_TYPE_H

#include "common/data_type/basic_types.h"

namespace Robosense
{

/**
 * @brief tracker info container struct
 */
class EIGEN_ALIGN16 RoboTarget
{
public:
  RoboTarget();

  RoboTarget(const RoboTarget &tar);

  RoboTarget &operator=(const RoboTarget &tar);

  int tracker_id; /**<tracking ID, which assigns an object to a certain tracker.*/
  int state; /**<state for an object in tracking procedure, have 3 state: 0 means new-appear, 1 means under tracking, 2 means disappearing.*/

  Point2f position; /**<tracking object position in xy-plane, relative to lidar */

  Point2f velocity; /**<tracking object velocity in xy-plane, relative to lidar.*/
  Point2f acceleration; /**<relative tracking object acceleration in xy-plane.*/

  Point2f velocity_abs;/**<global tracking object velocity in xy-plane.*/
  Point2f acceleration_abs; /**<global tracking object acceleration in xy-plane.*/

  Point2f traj;/**< trajectory for measuring center of object box.*/

  float angle_velocity; /**< angle velocity in radian*/

  unsigned long life_time; /**<current tracker total life time, including visible and invisible tracks (calculated by live frame number)*/
  unsigned long visible_life_time; /**<current tracker life time only considering visible tracks (calculated by live frame number)*/

  float possibility;//**< the possibility measurement that the current target is associated to a existing tracker */

  float sequence_robustness; /**< robustness analyzed by a historical sequential tracker frames, the smaller, the better.*/

  //#debug param

  int link_mode;


  typedef boost::shared_ptr<RoboTarget> Ptr;
  typedef boost::shared_ptr<const RoboTarget> ConstPtr;

};

}



#endif //ROBOSENSE_TARGET_TYPE_H
