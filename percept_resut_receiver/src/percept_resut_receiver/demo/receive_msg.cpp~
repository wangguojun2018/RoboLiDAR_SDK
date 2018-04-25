
/**
 * Example of Robosense SDK usage.
 */
#include "communication.h"
#include "robo_draw_rviz.h"
#include <ros/package.h>
#include <std_msgs/Float32.h>

#define SAVE_PERCEPTION_RESULT_TO_FILE 0


Robosense::Communication *communicator;
DrawRviz *dr;
ros::Publisher pub_label_msg_info;
ros::Publisher pub_label_msg_box_info;
ros::Publisher pub_cluster_msg_info;
ros::Publisher pub_track_msg_info;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~"); //parameter node

  pub_label_msg_info = node.advertise<visualization_msgs::MarkerArray>("label_msg_info",10);
  pub_label_msg_box_info = node.advertise<visualization_msgs::MarkerArray>("label_msg_box",10);
  pub_cluster_msg_info = node.advertise<visualization_msgs::MarkerArray>("cluster_msg_info",10);
  pub_track_msg_info = node.advertise<visualization_msgs::MarkerArray>("track_msg_info",10);

  communicator = new Robosense::Communication();
  communicator->initReceiver();
  communicator->setPort(60000);

  dr = new DrawRviz();
  std::vector<Robosense::PerceptResultMsg> result_msgs;
  Robosense::Header header;
  Robosense::PoseMsg pose;
  pcl::PCLHeader _header;
  _header.frame_id = "rslidar";
//  std::string filename = ros::package::getPath("robosense_demonstration") + "/result/";

  while(true)
  {
    std::vector<Robosense::PerceptResultMsg>().swap(result_msgs);
    communicator->receMsg(result_msgs,header,pose);

    float yaw = 0;
    dr->show_labelmsg_info(pub_label_msg_info,_header,result_msgs);
    dr->show_cluster_msg_info(pub_cluster_msg_info,_header,result_msgs);
    dr->show_track_msg_info(pub_track_msg_info,_header,result_msgs,yaw);
    dr->show_labelmsg_info_box(pub_label_msg_box_info,_header,result_msgs);
  }
  return 0;
}


