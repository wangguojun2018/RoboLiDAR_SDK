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
 * Robosense built-in data structs type —— bounding box.
 *
 */

#ifndef ROBOSENSE_BOX_TYPE_H
#define ROBOSENSE_BOX_TYPE_H

#include "common/data_type/basic_types.h"

namespace Robosense
{

/**
 * @brief tracker info container struct
 */
class EIGEN_ALIGN16 BoundingBox
{

public:

  BoundingBox();

  BoundingBox(const BoundingBox &box);

  BoundingBox &operator=(const BoundingBox &box);

  bool operator==(const BoundingBox &box) const;
  bool operator!=(const BoundingBox &box) const;

  bool isNull();

  Point3f size;/**< x:length, y:width, z:height, notice that length >= width*/
  Point3f center; /**<box geometry center*/
//  Point3f anchor;/**< the nearest corner point of box to the lidar*/
  float angle; /**< yaw angle of box direction*/
  Point3f heading; /**< box direction, parallel to longest edge*/

  typedef boost::shared_ptr<BoundingBox> Ptr;
  typedef boost::shared_ptr<const BoundingBox> ConstPtr;

};


}
#endif //ROBOSENSE_BOX_TYPE_H
