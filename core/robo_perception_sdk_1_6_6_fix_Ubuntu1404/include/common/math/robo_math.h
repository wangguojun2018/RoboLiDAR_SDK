
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
 * Date: 2018.3
 *
 * DESCRIPTION
 *
 * Robosense common math module, including common used algebra, trigonometric, etc.
 *
 */

#ifndef ROBOSENSE_MATH_H
#define ROBOSENSE_MATH_H
#include "common/data_type/robo_types.h"

namespace Robosense
{
/**
 * @brief calculate a gaussian-like weight, not equal to gaussian distribution
 * @param[in] val input value need evaluate.
 * @param[in] sigma sigma for gaussian
 * @param[in] mean mean for gaussian
 * @return
 */
float gaussianWeight(float val, float sigma, float mean = 0.f);

/**
 * @brief calculate a gaussian value from gaussian distribution
 * @param[in] val input value need evaluate.
 * @param[in] sigma sigma for gaussian
 * @param[in] mean mean for gaussian
 * @return
 */
float gaussianProb(float val, float sigma, float mean = 0.f);

/**
 * calculate dot product
 * */
float vecDot2D(const Point2f& a, const Point2f& b);
float vecDot3D(const Point3f& a, const Point3f& b);
float vecDot4D(const Point4f& a, const Point4f& b);

/**
 * @brief calculate error between angles, in radian.
 */
float calcAngleErr2D(const float &a, const float &b);

float calcAngleSum2D(const float &a, const float &b);

float normalizeAngle(float &a);

template <typename T>
int sign(const T& val)
{
  return val>=0?1:-1;
}


bool inPolygon(int nvert, float *vertx, float *verty, float testx, float testy);

}



#endif