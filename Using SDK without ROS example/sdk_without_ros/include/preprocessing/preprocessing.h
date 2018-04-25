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
 * Version: 1.0.0
 * Date: 2017.10
 *
 * DESCRIPTION
 *
 * Robosense preprocessing module, including calibration according mounting args etc.
 *
 */


#ifndef ROBOSENSE_PREPROCESS_H
#define ROBOSENSE_PREPROCESS_H

#include "common/data_type/robo_types.h"
#include "common/geometry/geo_base.h"

namespace Robosense
{
  /**
   * @brief preprocessing module
   * @ingroup preprocessing
   */
  class PreProcess{

  public:
    PreProcess();
    ~PreProcess();

    bool preProcess(PointCloudConstPtr pcloud_in, NormalPointCloudPtr pcloud_out);

  private:

  };
}


#endif //ROBOSENSE_PREPROCESS_H


/**
 * @brief preprocessing module
 * @defgroup preprocessing
 */