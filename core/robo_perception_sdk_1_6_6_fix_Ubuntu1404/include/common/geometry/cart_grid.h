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
 * Robosense grid module.
 *
 */

#ifndef ROBOSENSE_COMMON_GEOMETRY_CART_GRID_H
#define ROBOSENSE_COMMON_GEOMETRY_CART_GRID_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include "common/data_type/robo_types.h"

namespace Robosense{
    class CartGrid{
    public:
        CartGrid();
        ~CartGrid(){}

        /**
        * @brief grid build grid for cloud
        * @param[in] in_cloud_ptr input cloud
        * @param[out] grids output grids
        * @return flag to indicate the process succeed or failed
        */
        bool cartGrid(PointCloudConstPtr in_cloud_ptr,
                      std::vector<std::vector<std::vector<Point> > >& grids);

        /**
        * @brief get grid index for a point
        * @param[in] in_pt input point
        * @param[out] out_index output index
        * @return flag to indicate the process succeed or failed
        */
        bool getIndex(const Point& in_pt,cv::Point2i& out_index);

        /**
        * @brief get cloud out of grids
        * @param[out] leave_cloud_ptr output cloud out range of grids
        */
        void getLeavePts(PointCloudPtr leave_cloud_ptr);

        /**
        * @brief get grid size
        * @param[out] size grid size
        */
        void getGridPtsSize(cv::Point2i& size);

        /**
        * @brief set grid size
        * @param[in] range,size grid relative params
        */
        bool setGrid(const Range3D& range = Range3D(-50.,50.,-50.,50.,-3.,0.),const float& size = 0.5);

        void setUserDefineCartHT(float (*pf)(const Point& ) = NULL);
    protected:
        void setConditionMat();

        float (*computeCartHeightThre_)(const Point& point_pt) = NULL;
        float computeCartHeightThre(const Point& in_pt);
        cv::Mat height_thre_mat_;

        int grid_length_,grid_width_;
        PointCloudPtr leave_cloud_ptr_;
        Range3D range_;
        float grid_size_;
    private:
    };
}

#endif //ROBOSENSE_COMMON_GEOMETRY_CART_GRID_H
