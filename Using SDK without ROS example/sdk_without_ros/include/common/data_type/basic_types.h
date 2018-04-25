
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
 * Robosense built-in data structs type definition module.
 *
 */

#ifndef ROBOSENSE_BASIC_TYPES_H
#define ROBOSENSE_BASIC_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define COUT(X) std::cout << X << std::endl
#define COUTG(X) std::cout << "\033[1;32m " << X << "\033[0m" << std::endl
#define COUTR(X) std::cout << "\033[1;31m " << X << "\033[0m" << std::endl

namespace Robosense
{

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

typedef pcl::PointXYZINormal NormalPoint;
typedef pcl::PointCloud<NormalPoint> NormalPointCloud;
typedef pcl::PointCloud<NormalPoint>::Ptr NormalPointCloudPtr;
typedef pcl::PointCloud<NormalPoint>::ConstPtr NormalPointCloudConstPtr;


#define UNKNOW 0
#define PED 1
#define BIC 2
#define CAR 3
#define TRUCK_BUS 4

#define PI_OVER_180 (0.0174532925)


//--------------------- robosense build-in fundamental structs ---------------------------//
/**
 * @brief basic data struct
 */
struct EIGEN_ALIGN16 Point2f  // fundamental 2D data structure
{
  Point2f();

  Point2f(const float &tx, const float &ty);

  Point2f(const Point2f &p);

  Point2f &operator=(const Point2f &p);

  Point2f &operator+=(const Point2f &p);

  Point2f &operator-=(const Point2f &p);

  Point2f &operator*=(const float &t);

  Point2f &operator*=(const Point2f &p);

  Point2f operator+(const Point2f &b) const;

  Point2f operator-(const Point2f &b) const;

  Point2f& operator-();

  Point2f operator-() const;

  Point2f operator*(const float &t) const;

  Point2f operator*(const Point2f &p) const;

  float getLength() const;

  Point2f& normalize();

  float x, y;
};

/**
 * @brief basic data struct
 */
struct EIGEN_ALIGN16 Point3f // fundamental 3D data structure
{
  Point3f();

  Point3f(const float &tx, const float &ty, const float &tz);

  Point3f(const Point3f &p);

  Point3f &operator=(const Point3f &p);

  Point3f &operator+=(const Point3f &p);

  Point3f &operator-=(const Point3f &p);

  Point3f &operator*=(const float &t);

  Point3f &operator*=(const Point3f &p);

  Point3f operator+(const Point3f &b) const;

  Point3f operator-(const Point3f &b) const;

  Point3f& operator-();

  Point3f operator-() const;

  Point3f operator*(const float &t) const;

  Point3f operator*(const Point3f &p) const;

  float getLength() const;

  Point3f& normalize();

  float x, y, z;
};

/**
 * @brief basic data struct
 */
struct EIGEN_ALIGN16 Point4f // fundamental 4D data structure
{
  Point4f();

  Point4f(const float &tx, const float &ty, const float &tz, const float &tw);

  Point4f(const Point4f &p);

  Point4f &operator=(const Point4f &p);

  Point4f &operator+=(const Point4f &p);

  Point4f &operator-=(const Point4f &p);

  Point4f &operator*=(const float &t);

  Point4f &operator*=(const Point4f &p);

  Point4f operator+(const Point4f &b) const;

  Point4f operator-(const Point4f &b) const;

  Point4f& operator-();

  Point4f operator-() const;

  Point4f operator*(const float &t) const;

  Point4f operator*(const Point4f &p) const;

  float getLength() const;

  Point4f& normalize();

  float x, y, z, w;
};


/**
 * @brief Range definitions for 2D
 */
struct EIGEN_ALIGN16 Range2D
{
  Range2D();
  Range2D(float x_min, float x_max, float y_min, float y_max);
  Range2D(const Range2D &r);
  Range2D &operator=(const Range2D &r);

  float xmin, xmax, ymin, ymax;
};

/**
 * @brief Range definitions for 3D
 */
struct EIGEN_ALIGN16 Range3D
{
  Range3D();
  Range3D(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
  Range3D(const Range3D &r);
  Range3D &operator=(const Range3D &r);

  float xmin, xmax, ymin, ymax, zmin, zmax;
};


/**
 * @brief kalman filter struct, depicts the parameters need by kalman filter.
 * @ingroup tracker
 */
struct EIGEN_ALIGN16 KalmanParms
{
  Point4f init_state; /**<init state for kalman*/
  float cycle_period;/**<time period for update*/
  Point4f process_noise_conv;/**<process noise*/
  Point4f measure_noise_conv;/**<measurement noise*/
};


/**
 * @brief pose
 */
struct EIGEN_ALIGN16 Pose
{
  Eigen::Vector3f pos; /**< translation, with meter unit*/
  Eigen::Vector3f angle; /**< rotation, with radian unit*/
};


struct EIGEN_ALIGN16 LidarType
{
  std::string name;/**< lidar type name*/
  float hori_resolution;/**< lidar angle resolution between points in same line, with degree unit*/
  float vert_resolution;/**< lidar angle resolution between lines, with degree unit*/
  int frequency;/**< lidar working frequency*/
};


/**
  * @brief ObjectLimit parameters container, in order to filter out background and static objects so that the following tracking
 * and classification can focus on the foreground dynamic objects, the lidar assumed to be mounted horizontally.
*/
struct EIGEN_ALIGN16 ObjectLimit
{
  float obj_max_height; /**<height threshold for an object's absolute height from the ground*/
  float obj_size_height; /**<height threshold for an object size to be detected*/
  float obj_size_length; /**<length threshold for an object size to be detected*/
  float obj_size_width; /**<width threshold for an object size to be detected*/
  float obj_length_width_ratio; /**<the ratio of height and length threshold for an object to be detected*/
  float obj_max_area; /**< possible vechile-like object area restriction*/
};


/**
 * @brief struct container for perception information output for usrers
 */
struct EIGEN_ALIGN16 PerceptOutput
{
  Point3f location; /**<position of bbox center, measured in lidar or vehicle coordinate system, according to user's setting*/
  Point3f direction; /**<direction of bbox, vector format, measured by the length edge direction, in lidar or vehicle coordinate system, according to user's setting*/
  float yaw; /**<direction of bbox with roll pitch raw depictions, for simplicity, just yaw is usefull, coincident with "direction"*/
  Point3f size; /**<size of bbox, length, width, height of box, length is always >= width*/
  Point3f nearest_point; /**<position of nearest corner of bbox, measured in lidar or vehicle coordinate system, according to user's setting*/

  int tracker_id; /**<tracker id for objects, same object in sequtial frames share the same id*/
  float track_prob; /**<tracking association probability, confidence level (0~1), the higher, the better.*/

  Point2f velocity; /**<speed of obstacles, measured in local coordinate system*/
  Point2f acceleration; /**<acceleration of obstacles, measured in local coordinate system*/

  Point2f velocity_abs; /**<speed of obstacles, measured in global coordinate system*/
  Point2f acceleration_abs; /**<acceleration of obstacles, measured in global coordinate system*/
  float angle_velocity; /**< angle velocity in radian*/

  float life; /**<total current tracker life time, including visible and invisible ones, with unit seconds*/
  float visible_life; /**<current tracker life time only considering visible tracks, with unit seconds*/
  float robustness; /**< robustness analyzed by a historical sequential tracker frames, the smaller, the better.*/

  int label; /**<type of obstacles, like pedestrain, bike, car, truck*/
  float label_confidence; /**< confidence of classification*/

  bool is_background; /**< if is background, the flag will be set true. */
};




}

#endif //ROBOSENSE_BASIC_TYPES_H
