/**
 *  Robosense display tool, service for Robosense perception debug.
 *  This is in processing, some more tools will be added in the future.
 */

#ifndef PROJECT_DRAWRVIZ_H
#define PROJECT_DRAWRVIZ_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include "communication_type.h"

struct BoxInfo
{
	cv::Point3f size;
	cv::Point3f center;
	cv::Point3f corners[8];
	float angle;
};

class DrawRviz
{
public:
	DrawRviz();
  ~DrawRviz();


	void show_labelmsg_info(const ros::Publisher& pub_percept_info, const pcl::PCLHeader& _header,
														 const std::vector<Robosense::PerceptResultMsg>& msgs);

  void show_labelmsg_info_box(const ros::Publisher& pub_percept_info, const pcl::PCLHeader& _header,
														 const std::vector<Robosense::PerceptResultMsg>& msgs);

	void show_cluster_msg_info(const ros::Publisher& pub_cluster_msg_info,const pcl::PCLHeader& _header,const std::vector<Robosense::PerceptResultMsg>& msgs);

	void show_track_msg_info(const ros::Publisher& pub_track_msg_info, const pcl::PCLHeader& _header, const std::vector<Robosense::PerceptResultMsg>& msgs,const float yaw);

	private:
  void draw_cube(const BoxInfo& box, const int& marker_id, visualization_msgs::Marker& marker, cv::Point3f color,float scale = 1.0);
	void draw_box(const BoxInfo& box, const int& marker_id, visualization_msgs::Marker& marker, float scale = 1.0);
	void draw_text(const cv::Point3f& pos, const std::string& info, const int& marker_id, visualization_msgs::Marker& marker);
	void draw_box_arrow(const BoxInfo& box, const int& marker_id, visualization_msgs::Marker& marker);
	void draw_track_msg_arrow(const Robosense::PerceptResultMsg& per, const int& marker_id, visualization_msgs::Marker& marker,const float& yaw);

	std_msgs::Header convertPCLHeader2ROSHeader(const pcl::PCLHeader& _header);

	void generateColors(std::vector<cv::Point3f>& colors, int num);
	void convertBoxMsg(const Robosense::boxMsg& BoxMsg,BoxInfo& box);
	BoxInfo rotateBox2D(const BoxInfo &box, float angle);

};



template<typename T>
std::string num2str(T num, int precision)
{
	std::stringstream ss;
	ss.setf(std::ios::fixed, std::ios::floatfield);
	ss.precision(precision);
	std::string st;
	ss << num;
	ss >> st;

	return st;
}


#endif //PROJECT_DRAWRVIZ_H