/**
 *  Robosense display tool, service for Robosense perception debug.
 *  This is in processing, some more tools will be added in the future.
 */

#ifndef ROBOSENSE_DRAWRVIZ_H
#define ROBOSENSE_DRAWRVIZ_H

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <time.h>
#include <pcl_ros/point_cloud.h>
#include "common/data_type/robo_types.h"
#include "common/box/boxer.h"

namespace Robosense
{

template<typename PointT>
class DrawRviz
{
public:

	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	DrawRviz();

	void show_car_self(const ros::Publisher &pub_car_info, const pcl::PCLHeader &_header, Eigen::VectorXf car_pos,
										 const std::string &frame_id);

	void show_cluster(const ros::Publisher &pub_cluster_cloud, const pcl::PCLHeader &_header,
										const std::vector<RoboPerceptron<PointT> > &clusters);

	void show_percept(const ros::Publisher &pub_percept_info, const pcl::PCLHeader &_header,
										const std::vector<RoboPerceptron<PointT> > &perception_info, float yaw);

	void draw_box(const BoundingBox &box, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
								float scale = 1.0);

	void draw_cube(const BoundingBox &box, const int &marker_id, visualization_msgs::Marker &marker, float alpha,
								 float scale = 1.0);

	void draw_text(const Point3f &pos, const std::string &info, const int &marker_id, visualization_msgs::Marker &marker,
								 float alpha = 1.0);

	void draw_track_arrow(const RoboPerceptron<PointT> &per, const int &marker_id, visualization_msgs::Marker &marker,
												float yaw, float alpha = 1.0);


	std_msgs::Header convertPCLHeader2ROSHeader(const pcl::PCLHeader &_header);

	void generateColors(std::vector<Point3f> &colors, int num);
};

}

#endif //ROBOSENSE_DRAWRVIZ_H