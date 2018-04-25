

/**
 * Example of Robosense SDK usage, with map.
 */

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include "module_manager/module_manager.h"
#include "robo_draw_rviz.h"
#include "robosense/robosense.h"
#include "communication/communication.h"


#define DEBUGING 1
#define USE_COMM 1
#define WITH_DRAW 1

using namespace Robosense;

static int frame_id=0;

// give the lisense key file and data path to module manager to authorization.
// Notice that this module statement must lay before all SDK module defined.
ModuleManager *module_manager;//

DrawRviz<NormalPoint> *dr;

RobosenseALL *robosense_all;
Communication *communicator;

ros::Publisher pub_ground;
ros::Publisher pub_no_ground;
ros::Publisher pub_percept;
ros::Publisher pub_car;
ros::Publisher pub_clusters_cloud;
//ros::Publisher pub_vehicle_cloud;
ros::Publisher pub_ori_cloud;

/**
 * @brief callback function: accept pointcloud data parsed by rslidar driver and process perception algorithm
 * @param pts_msg[in]: input pointcloud message obtained by rslidar driver.
 * @param pose_nav[in]: input localization information, including position, posture, vehicle velocity and yaw direciotn of velocity.
 * Usually those infomation is obtained from GPS/IMU and encoder or obd information and processed by our localization module and passed here.
 */
void fullscanCallback(const sensor_msgs::PointCloud2::ConstPtr& pts_msg, const nav_msgs::Odometry::ConstPtr& pose_nav)
{
#if DEBUGING
  COUT("---------------------------"<<frame_id<<"-----------------------------");
  clock_t timer_total = clock();
#endif

  //-----------step 1: translate the ROS PointCloud2 msg into pcl pointcloud which is the standard input format for our sdk.
  //PointCloud2 transform to pcl::PointCloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pts_msg, pcl_pc2);
  PointCloudPtr pcloud_in(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcloud_in);
  pcloud_in->header.frame_id = "base_link2";

  COUT(pcloud_in->width<<" "<<pcloud_in->height);


  //-----------step 2: calculate the transform matrix between vehicle coord and global coord

  NormalPointCloudPtr object_cloud_ptr(new NormalPointCloud);//after align
  NormalPointCloudPtr ground_cloud_ptr(new NormalPointCloud);//after align
//  NormalPointCloudPtr vehicle_cloud_ptr(new NormalPointCloud);

  std::vector<NormalRoboPerceptron> percept_vec_filtered; //internal results for algorithm
  std::vector<PerceptOutput> out_put_list; //last output results for users

  //map ROI transmatrix calculation
  tfScalar cur_roll, cur_pitch, cur_yaw;
  geometry_msgs::Quaternion q = pose_nav->pose.pose.orientation; // orientation;

  tf::Quaternion b_q;
  tf::quaternionMsgToTF(q, b_q);
  tf::Matrix3x3(b_q).getRPY( cur_roll, cur_pitch, cur_yaw);

#if DEBUGING
  COUT("current rpy: "<<cur_roll<<" "<<cur_pitch<<" "<<cur_yaw);
#endif


  Pose v2g_pose;
  v2g_pose.pos(0) = pose_nav->pose.pose.position.x;
  v2g_pose.pos(1) = pose_nav->pose.pose.position.y;
  v2g_pose.pos(2) = pose_nav->pose.pose.position.z;
  v2g_pose.angle(0) = cur_roll;
  v2g_pose.angle(1) = cur_pitch;
  v2g_pose.angle(2) = cur_yaw;
  Eigen::Matrix4f v2g_mat = calcTransformMatrix(v2g_pose);

  //-----------step 3: call the SDK functions by a ensemble proxy entrance "robosense_all"

  robosense_all->mainPerceptionProcess(pcloud_in, pose_nav->twist.twist.linear.x, cur_yaw, v2g_mat);

  //get result
  robosense_all->getGroundPoints(ground_cloud_ptr); //internal ground detect results
  robosense_all->getObjectPoints(object_cloud_ptr); //internal object points detect results
  percept_vec_filtered = robosense_all->getObjectPeceptResults();//original filtered perception-result-list for developer debug.
  out_put_list = robosense_all->getLastResultsForUser();//Compact perception-result-list for users, if you don‘t want debug, you can just use it

  Point2f self_velocity = Point2f(pose_nav->twist.twist.linear.x*cosf(cur_yaw), pose_nav->twist.twist.linear.x*sinf(cur_yaw));

#if DEBUGING
  COUTG( "frame total time: " << (double) (clock() - timer_total) / CLOCKS_PER_SEC << "s. " );
  timer_total = clock();
#endif

  //------------step 4: collect the perception results and display them by ros rviz
  //if you want to transport the perception results across computers, please enable the socket module
#if USE_COMM
  Header _rheader;
  PoseMsg pose;
  _rheader.frame_id = frame_id;
  _rheader.timestamp = pcloud_in->header.stamp;
  _rheader.object_num = percept_vec_filtered.size();
  pose.x = v2g_pose.pos(0);
  pose.y = v2g_pose.pos(1);
  pose.z = v2g_pose.pos(2);
  pose.roll = v2g_pose.angle(0);
  pose.pitch = v2g_pose.angle(1);
  pose.yaw = v2g_pose.angle(2);

  communicator->sendMsg(out_put_list, _rheader,pose);

#endif

#if WITH_DRAW

  pub_ori_cloud.publish(pcloud_in);

  pcl::PCLHeader _header = pcloud_in->header;

  //publish ground and non-ground pointcloud
  ground_cloud_ptr->header = _header;
  pub_ground.publish(ground_cloud_ptr);

  object_cloud_ptr->header = _header;
  pub_no_ground.publish(object_cloud_ptr);

//  vehicle_cloud_ptr->header = _header;
//  pub_vehicle_cloud.publish(vehicle_cloud_ptr);

  //============draw the infos in rviz======================
  Eigen::VectorXf car_pos = Eigen::VectorXf::Zero(9);
  car_pos[0] = pose_nav->pose.pose.position.x;
  car_pos[1] = pose_nav->pose.pose.position.y;
  car_pos[2] = pose_nav->pose.pose.position.z;
  car_pos[3] = cur_roll;
  car_pos[4] = cur_pitch;
  car_pos[5] = cur_yaw;
  car_pos[6] = self_velocity.x;
  car_pos[7] = self_velocity.y;

  //draw and display the perception results infos in rviz
  dr->show_car_self(pub_car, _header, car_pos, "rs_odom");
  dr->show_cluster(pub_clusters_cloud, _header, percept_vec_filtered);
  dr->show_percept(pub_percept, pcloud_in->header, percept_vec_filtered, cur_yaw);

#if DEBUGING
  COUTG("draw time: "<<(double)(clock()-timer_total)/CLOCKS_PER_SEC<< "s. ");
#endif

#endif

  frame_id++;
}


int main(int argc, char** argv)
{
  //check versions
  COUT("OpenCV version: "<<CV_VERSION);
  COUT("ROS version: "<<ROS_VERSION_MAJOR<<"."<<ROS_VERSION_MINOR<<"."<<ROS_VERSION_PATCH);
  COUT("PCL version: "<<PCL_VERSION_PRETTY);
  COUT("Boost version: "<<BOOST_LIB_VERSION);

  ros::init(argc, argv, "test_node_roi");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~"); //parameter node

  //obtain the global configuration parameters from launch file
  std::string key_path, pcap_path, eth_name, lidar_config_path, args_path, receiv_ip;
  int port, receiv_port;
  private_nh.param("key_path", key_path, std::string(""));
  private_nh.param("pcap", pcap_path, std::string(""));
  private_nh.param("ethernet_name", eth_name, std::string(""));
  private_nh.param("port", port, 6699);
  private_nh.param("lidar_config_path", lidar_config_path, std::string(""));
  private_nh.param("args_path", args_path, std::string(""));
  private_nh.param("receiv_ip", receiv_ip, std::string(""));
  private_nh.param("receiv_port", receiv_port, 60000);

  //------------------------------------------init the moduels--------------------------------------------
// Authorize the SDK, notice that this should be placed before any SDK module works.
  module_manager = new ModuleManager(key_path, pcap_path, eth_name, port);

  //configure the socket transport module, thd src code is provided in test/tools/communication folder, you can
  // write your own communication code by refering our open-sourced code.
  communicator = new Communication();
  communicator->setServerIP(receiv_ip);
  communicator->setPort(receiv_port);
  communicator->initSender();

  robosense_all = new RobosenseALL(lidar_config_path, args_path);

  dr = new DrawRviz<NormalPoint>();

  message_filters::Subscriber<sensor_msgs::PointCloud2> pts_sub(private_nh, "/rslidar_points", 10);
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub(private_nh, "/rs_pose", 10);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pts_sub, pose_sub);
  sync.registerCallback(boost::bind(&fullscanCallback, _1, _2));

#if WITH_DRAW
  pub_ground = node.advertise<sensor_msgs::PointCloud2>("ground",10);
  pub_no_ground = node.advertise<sensor_msgs::PointCloud2>("non_ground",10);
  pub_clusters_cloud = node.advertise<sensor_msgs::PointCloud2>("cluster",10);
  pub_percept = node.advertise<visualization_msgs::MarkerArray>("percept_info", 10);
  pub_car = node.advertise<visualization_msgs::MarkerArray>("car_info", 10);
//  pub_vehicle_cloud = node.advertise<sensor_msgs::PointCloud2>("ori_cloud_in_vehicle",10);
  pub_ori_cloud = node.advertise<sensor_msgs::PointCloud2>("ori_cloud",10);

#endif

  ros::spin();

  return 0;
}

