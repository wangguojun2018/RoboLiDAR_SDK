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
 * Date: 2017.11
 *
 * DESCRIPTION
 *
 * Robosense pointcloud normal calculation module.
 *
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "common/data_type/robo_types.h"

#ifndef ROBOSENSE_GEOBASE_H
#define ROBOSENSE_GEOBASE_H


namespace Robosense
{

    /**
     * @brief calculate distance between points
     * @param start
     * @param end
     * @return
     */
    float computeDisOfPoints(Point start, Point end);
    float computeDisOfPoints(Point3f start, Point3f end);

    /**
     * @brief calculate distance between a point and a line
     * @param start
     * @param end
     * @param test_point
     * @return
     */
    float computeDis2Line(Point start, Point end, Point test_point);
    float computeDis2Line(Point3f start, Point3f end, Point3f test_point);

    /**
     * @brief estimate normal
     * @param cloud_in
     * @param cloud_out
     * @return
     */
    bool computeNormal2(PointCloudConstPtr cloud_in, NormalPointCloudPtr cloud_out);
    bool computeNormal3(PointCloudConstPtr cloud_in, NormalPointCloudPtr cloud_out);

/**
 * @brief estimate normal, another complicate version
 * @param cloud_in
 * @param cloud_out
 * @param radius
 * @param search_step
 * @param angel_step
 * @return
 */
    bool computeNormal(PointCloudConstPtr cloud_in, NormalPointCloudPtr cloud_out,
                       float radius = 0.5f, float search_step = 0.1, float angel_step = 0.18);

/**
 * @brief estimate normal for grided points
 * @param cloud_in
 * @param cloud_out
 * @param step
 * @return
 */
    bool computeGridNormal(PointCloudConstPtr cloud_in, NormalPointCloudPtr cloud_out,
                           int step = 1);

/**
 * @brief estimate curvature
 * @param in_cloud_ptr
 * @return
 */
    bool estimateCurveFromNormal(NormalPointCloudPtr in_cloud_ptr);

/**
 * @brief calculate barycenter of pointcloud
 * @tparam PointT
 * @param cloud
 * @return
 */
template <typename PointT>
Point3f calcCloudBarycenter(typename pcl::PointCloud<PointT>::Ptr cloud);


/**
 * geometry transformes
 * */

    Eigen::Matrix4f getTransformMat(const std::vector<float>& in_vec);
    Eigen::Matrix4f calcRotationMatrix(const Eigen::Vector3f &before_vector, const Eigen::Vector3f &after_vector);
    Eigen::Matrix4f calcRotationMatrix(const float &angle, const Eigen::Vector3f &axis_vector);
    Eigen::Matrix4f calcRotationMatrix(const Eigen::Vector3f &angle);
    Eigen::Matrix4f calcTransformMatrix(const Pose &pose);
    Eigen::Matrix4f calcTransformMatrix(const Eigen::Vector3f &t);
    Eigen::Matrix4f calcTransformMatrix(const Eigen::Vector3f &before_vector, const Eigen::Vector3f &after_vector, const Eigen::Vector3f &trans);

    bool transormBox(const BoundingBox& in_box, BoundingBox &trans_box, const Eigen::Matrix4f &trans_mat);


}  // robosense
#endif //ROBOSENSE_GEOBASE_H
