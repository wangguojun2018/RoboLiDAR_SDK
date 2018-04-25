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
 * Robosense built-in data structs type —— classified object.
 *
 */

#ifndef ROBOSENSE_LABEL_TYPE_H
#define ROBOSENSE_LABEL_TYPE_H

#include "common/data_type/basic_types.h"

namespace Robosense
{

/**
 * @brief tracker info container struct
 */
class EIGEN_ALIGN16 RoboObject
{

public:

  RoboObject();

  RoboObject(const RoboObject &tl2);

  RoboObject &operator=(const RoboObject &tl2);

  int label; /**<object label after classification, a certain number represents a class, such as car, pedestrian, bike/motorcycle, truck and so on.*/
  float confidence; /**<probability of classification*/
  std::vector<float> prob_vec;/**<probability for classification*/

  typedef boost::shared_ptr<RoboObject> Ptr;
  typedef boost::shared_ptr<const RoboObject> ConstPtr;

};


}


#endif //ROBOSENSE_LABEL_TYPE_H
