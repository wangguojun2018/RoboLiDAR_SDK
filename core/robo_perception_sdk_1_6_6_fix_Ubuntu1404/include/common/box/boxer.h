
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
 * Robosense box module, for object bounding box calculation.
 *
 */

#ifndef ROBOSENSE_BOXER_H
#define ROBOSENSE_BOXER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "common/data_type/robo_types.h"

namespace Robosense
{

/**
 * @brief find the nearest corner of a bounding box according lidar coordinate.
 * @param[in] box input bounding box
 * @return
 */
Point3f findNearestBoxCorner(const BoundingBox &box);


bool findNearest2Corners(const BoundingBox &box, Point3f &first, Point3f &second);


/**
 * @brief calculate box area
 * @param[in] box input bounding box
 * @return box area
 */
float getBoxArea(const BoundingBox &box);

/**
 * @brief calculate box volume
 * @param[in] box input bounding box
 * @return box volume
 */
float getBoxVolume(const BoundingBox &box);

/**
 * @brief rectify box for those nearly horizontal to axis.
 * @param[in] box input original box
 * @param[in] ang_thd the tilt angel thd within which need to rectify
 * @param[in] len_thd the length thd of box length within which need to rectify
 * @return rectified box
 */
BoundingBox rectifyBox(const BoundingBox &box, float ang_thd, float len_thd);

/**
 * @brief rotate bounding box
 * only work for xy plane
 * @param[in] box input bounding box
 * @param[in] angle input rotate angle in radian
 * @return rotated bounding box
 */
BoundingBox rotateBox2D(const BoundingBox &box, float angle);

/**
 * @brief rotate bounding box with a given center point
 * @param[in] box input bounding box
 * @param[in] angle input rotate angle in radian
 * @param[in] rot_center rotate center
 * @return
 */
BoundingBox rotateBox2DwithCenter(const BoundingBox &box, float angle, Point3f rot_center);

/**
 * @brief scale bounding box
 * @param[in] box input bounding box
 * @param[in] scale input scale value
 * @return scaled bounding box
 */
BoundingBox scaleBox(const BoundingBox &box, float scale);

/**
 * @brief scale bounding box in xyz respectively
 * @param[in] box input bounding box
 * @param[in] scale input scale value
 * @param[in] align_to_anchor if aligned to original bounding box anchor
 * @return scaled bounding box
 */
BoundingBox scaleBox3D(const BoundingBox &box, Point3f scale, bool align_to_anchor = true);

/**
* @brief translate bounding box
* @param[in] box input bounding box
* @param[in] t move vector
* @return translated bounding box
*/
BoundingBox transBox(const BoundingBox &box, Point3f t);


/**
 * @brief resize box
 * @param[in] box input bounding box
 * @param[in] new_size input new size value
 * @param[in] align_to_anchor if aligned to original bounding box anchor
 * @return resized bounding box
 */
BoundingBox resizeBox(const BoundingBox &box, Point3f new_size, bool align_to_anchor = true);

/**
 * @brief get corners from a box that give the center, size, heading or yaw angle
 * @return
 */
bool getCornersFromBox2D(const BoundingBox &box, std::vector<Point3f> &out_corners);


/**
 * @brief semantic box infer
 * @param[in] box input original geometry bbox
 * @param[in] obj_limit
 * @param[in] estimate_lidar_height
 * @param[out] infer_box output semantic infered-box
 * @return infering success or not
 */
bool boxInferByGeo(const BoundingBox &box, const ObjectLimit &obj_limit, const float &estimate_lidar_height, BoundingBox &infer_box);

bool boxInferByGeo2(const BoundingBox &box, const ObjectLimit &obj_limit, const float &estimate_lidar_height, BoundingBox &infer_box);

bool boxInferByGeo3(const BoundingBox &box, const ObjectLimit &obj_limit, const float &estimate_lidar_height, BoundingBox &infer_box);

bool boxInferByGeo4(const BoundingBox &box, const ObjectLimit &obj_limit, const float &estimate_lidar_height, BoundingBox &infer_box);

/**
 * @brief box infer by classification
 */
void boxInferByLabel(std::vector<NormalRoboPerceptron>& perceptrons);


/**
 * @brief build a box when given corners.
 * @param[in] box_corners
 * @return generated box
 */
BoundingBox buildBoxFromCorners(const std::vector<Point3f> &box_corners);


/**
  * @brief calculate temp size for a given direction
  */
template <typename PointT>
BoundingBox calcBoundingBoxAccordingDir(typename pcl::PointCloud<PointT>::ConstPtr segment, const Point3f &direction);

template <typename PointT>
BoundingBox calcBoundingBoxAccordingDir(typename pcl::PointCloud<PointT>::ConstPtr segment, const float &yaw);


template <typename PointT>
void calcSizeCenterAccordingDir(typename pcl::PointCloud<PointT>::ConstPtr segment, const Point3f &direction, Point3f &size, Point3f &center);


bool withinBox2D(const BoundingBox& box, const Point3f &p);
bool withinBox3D(const BoundingBox& box, const Point3f &p);


//BoundingBox mirrorBox2D(const BoundingBox &box, const Point3f &a, const Point3f &b);



/**
 * @brief calculate bounding-box for pointcloud cluster
 */
template <typename PointT>
class BoundingBoxCalculator {

public:

  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  /**
   * @brief constructor
   * @param[in] grid_size grid size used to estimate the box edge
   * @param[in] theta angular unit used to estimate the main direction of box
   * @param[in] min_num_thd points num less than it will use the simple version to generate a axis-aligned bbox
   * @param[in] max_seg_num points num greater than it will use downsampling to generate bbox
   */
  BoundingBoxCalculator(float grid_size = 0.1f, float theta = M_PI / 90.f, int min_num_thd = 20, int max_seg_num = 300);

  /**
   * @brief reset the parameters
   * @param[in] grid_size grid size used to estimate the box edge
   * @param[in] theta angular unit used to estimate the main direction of box
   * @param[in] min_num_thd points num less than it will use the simple version to generate a axis-aligned bbox
   * @param[in] max_seg_num points num greater than it will use downsampling to generate bbox
   */
  void setBoxCalcParams(float grid_size = 0.1f, float theta = M_PI / 90.f, int min_num_thd = 20, int max_seg_num = 300);

  /**
   * @brief calculate bounding box by coordinate aligned, fast
   */

  BoundingBox calcSimpleBox(PointCloudConstPtr segment);

  /**
   * @brief calculate bounding box by object aligned, slow but accurate
   */
  BoundingBox calcBoundingBox(PointCloudConstPtr segment);


private:

  Point3f getSegmentCenter(PointCloudConstPtr segment);

  float grid_size_, grid_size_inv_;
  float theta_;
  int min_num_thd_;
  int max_seg_num_;
  std::vector<float> tabCos_, tabSin_;
};


/**
 * @todo TODO: some new fundamentally supporting functions will be added in the future ...
 */

}

#endif