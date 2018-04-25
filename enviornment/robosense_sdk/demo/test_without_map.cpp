
/**
 * Example of Robosense SDK usage, bare use without map and localization.
 */

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "robosense/robosense.h"
#include "communication/communication.h"
#include "module_manager/module_manager.h"
#include "robo_draw_rviz.h"

#define DEBUGING 1
#define USE_COMM 1
#define WITH_DRAW 1

using namespace Robosense;

static int frame_id = 0;

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
ros::Publisher pub_ori_cloud;
//ros::Publisher pub_vehicle_cloud;

float obd_velocity = 0;

/**
 * @brief callback function for vehicle velocity without localization and direction information, notice that this is a coarse
 * complementation without considering direction, that is assuming the global velocity is aligned and parallel to the x-axis
 * of global coordinate.
 * @param pose_nav
 */
void obd_callback(const nav_msgs::Odometry::ConstPtr& pose_nav)
{
  obd_velocity = pose_nav->twist.twist.linear.x;
}

/**
 * @brief callback function: accept pointcloud data parsed by rslidar driver and process perception algorithm
 * @param pts_msg[in]: input pointcloud message obtained by rslidar driver.
 */
void fullscanCallback(const sensor_msgs::PointCloud2::ConstPtr &pts_msg)
{
#if DEBUGING
  COUT( "---------------------------" << frame_id << "-----------------------------" );
  clock_t timer_total = clock();
#endif

  //-----------step 1: translate the ROS PointCloud2 msg into pcl pointcloud which is the standard input format for our sdk.
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*pts_msg, pcl_pc2);
  PointCloudPtr pcloud_in(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcloud_in);

  //-----------step 2: calculate the transform matrix between vehicle coord and global coord
  NormalPointCloudPtr object_cloud_ptr(new NormalPointCloud);
  NormalPointCloudPtr ground_cloud_ptr(new NormalPointCloud);
//  NormalPointCloudPtr vehicle_cloud_ptr(new NormalPointCloud);

  std::vector<NormalRoboPerceptron> percept_vec, percept_vec_filtered;
  std::vector<PerceptOutput> out_put_list;

  COUT(pcloud_in->width<<" "<<pcloud_in->height);

  //-----------step 3: call the SDK functions by a ensemble proxy entrance "robosense_all"

  robosense_all->mainPerceptionProcess(pcloud_in, obd_velocity);

  //get perception result
  robosense_all->getGroundPoints(ground_cloud_ptr);
  robosense_all->getObjectPoints(object_cloud_ptr);
  percept_vec = robosense_all->getObjectOriPeceptResults();//original perception-result-list for developer debug.
  percept_vec_filtered = robosense_all->getObjectPeceptResults();//original filtered perception-result-list for developer debug.
  out_put_list = robosense_all->getLastResultsForUser();//Compact perception-result-list for users, if you don‘t want debug, you can just use it

  Point2f self_velocity = Point2f(obd_velocity,0);

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

  communicator->sendMsg(out_put_list, _rheader, pose);
#endif


#if WITH_DRAW

  pub_ori_cloud.publish(pcloud_in);

  //publish ground and non-ground pointcloud
  ground_cloud_ptr->header = pcloud_in->header;
  pub_ground.publish(ground_cloud_ptr);

  object_cloud_ptr->header = pcloud_in->header;
  pub_no_ground.publish(object_cloud_ptr);

//  vehicle_cloud_ptr->header = pcloud_in->header;
//  pub_vehicle_cloud.publish(vehicle_cloud_ptr);

  //============draw the infos in rviz======================
  Eigen::VectorXf car_pos = Eigen::VectorXf::Zero(9);
  car_pos(6) = self_velocity.x;
  car_pos(7) = self_velocity.y;

  //draw and display the perception results infos in rviz
  dr->show_car_self(pub_car, pcloud_in->header, car_pos, "rslidar");
  dr->show_cluster(pub_clusters_cloud, pcloud_in->header, percept_vec_filtered);
  dr->show_percept(pub_percept, pcloud_in->header, percept_vec_filtered, 0);

#if DEBUGING
  COUTG( "draw time: " << (double) (clock() - timer_total) / CLOCKS_PER_SEC << "s. " );
#endif

#endif

  frame_id++;
}


int main(int argc, char **argv)
{

  //check versions
  COUTG("OpenCV version: "<<CV_VERSION);
  COUTG("ROS version: "<<ROS_VERSION_MAJOR<<"."<<ROS_VERSION_MINOR<<"."<<ROS_VERSION_PATCH);
  COUTG("PCL version: "<<PCL_VERSION_PRETTY);
  COUTG("Boost version: "<<BOOST_LIB_VERSION);
  COUTG("Eigen version: "<<EIGEN_WORLD_VERSION<<"."<<EIGEN_MAJOR_VERSION<<"."<<EIGEN_MINOR_VERSION);

  ros::init(argc, argv, "test_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~"); //parameter node

  std::string key_path, pcap_path, eth_name, lidar_config_path, args_path, receiv_ip;
  int port, receiv_port;

  //obtain the global configuration parameters from launch file
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

  communicator = new Communication();
  communicator->setServerIP(receiv_ip);
  communicator->setPort(receiv_port);
  communicator->initSender();

  robosense_all = new RobosenseALL(lidar_config_path, args_path);

  dr = new DrawRviz<NormalPoint>();

  ros::Subscriber sub_fullscan = node.subscribe("/rslidar_points", 10, fullscanCallback);
  ros::Subscriber sub_obd = node.subscribe("/canbus/canbus", 10, obd_callback);

#if WITH_DRAW
  pub_ground = node.advertise<sensor_msgs::PointCloud2>("ground", 10);
  pub_no_ground = node.advertise<sensor_msgs::PointCloud2>("non_ground", 10);
  pub_clusters_cloud = node.advertise<sensor_msgs::PointCloud2>("cluster", 10);
  pub_percept = node.advertise<visualization_msgs::MarkerArray>("percept_info", 10);
  pub_car = node.advertise<visualization_msgs::MarkerArray>("car_info", 10);
  pub_ori_cloud = node.advertise<sensor_msgs::PointCloud2>("ori_cloud",10);
//  pub_vehicle_cloud = node.advertise<sensor_msgs::PointCloud2>("ori_cloud_in_vehicle",10);

#endif

  ros::spin();

  return 0;
}


