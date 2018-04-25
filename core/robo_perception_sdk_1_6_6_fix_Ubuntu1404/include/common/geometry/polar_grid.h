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
 * Robosense polar grid module.
 *
 */

#ifndef ROBOSENSE_COMMON_GEOMETRY_POLAR_GRID_H
#define ROBOSENSE_COMMON_GEOMETRY_POLAR_GRID_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "common/data_type/robo_types.h"

namespace Robosense{
    class PolarGrid{
    public:
        PolarGrid();
        ~PolarGrid(){}

        /**
         * @brief build a polar grid
         * @param[in] in_cloud_ptr input cloud
         * @param[out] grids out grids
         * @return flag to indicate the process is succeed or failed
        */
        bool polarGrid(PointCloudConstPtr in_cloud_ptr,
                       std::vector<std::vector<std::vector<Point> > >& grids);

        /**
         * @brief get polar grid index for a point
         * @param[in] in_point input point
         * @param[out] out_index out index
         * @return flag to indicate the process is succeed or failed
        */
        bool getIndex(const Point& in_point,cv::Point2i& out_index);

        /**
         * @brief get points out range of the polar grids
         * @param[out] leave_cloud_ptr out cloud
        */
        void getLeavePts(PointCloudPtr leave_cloud_ptr);

        /**
         * @brief get polar grid size
         * @param[out] size out size
        */
        void getGridPtsSize(cv::Point2i& size);

        /**
         * @brief set polar grid
         * @param[in] range,min_angle,min_range_size input polar grid relative params
         * @return flag to indicate the process is succeed or failed
        */
        bool setGrid(const Range3D& range = Range3D(-50.,50.,-50.,50.,-3.,3.),
                     const float& min_angle = 0.5,const float& min_range_size = 0.25);
    protected:
        float computeBeta(const Point& pt);
        Range3D range_;
        float grid_range_size_;
        int MSegs_,NBins_;
        float grid_min_rad_;
        PointCloudPtr leave_cloud_ptr_;
    private:
    };
}

#endif //ROBOSENSE_COMMON_GEOMETRY_POLAR_GRID_H
