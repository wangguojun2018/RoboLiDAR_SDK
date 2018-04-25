/**
	The following is an example for using robosense perception sdk without ROS. 
	Note that you should change the path to your local and recompile.
	
	How to use:
	1. copy the /include and /lib file to your project (name say "sdk_without_ros");
	2. "cd sdk_without_ros" and "chmod -R 777 *" to avoid authority problem; 
	3. compile the project:
		mkdir build
		cd build
		cmake ..
		make
	4. run the demo: 
		./sdk_without_ros	

*/


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "robosense/robosense.h"
#include "module_manager/module_manager.h"


using namespace Robosense;
using namespace std;

ModuleManager *module_manager;

int main()
{
	
	// Authorize the SDK, notice that this should be placed before any SDK module works.
    std::string key_path = "/home/wangbin/sdk_without_ros/key";
    std::string pcap_path = "/home/wangbin/sdk_without_ros/data/test.pcap";
    std::string eth_name = "eth0";

	// !!!!!!!!Do not forget give the user and lidar configuration files!!!!!!!!
    std::string lidar_config_path = "/home/wangbin/sdk_without_ros/args/align.txt"; //extracted from "/auto_align_tool/save_config"
    std::string usr_config_path = "/home/wangbin/sdk_without_ros/args/perception_args.xml";//extracted from "/robosense_sdk/usr_args"
	
  	module_manager = new ModuleManager(key_path, pcap_path, eth_name);

    RobosenseALL *robosense_all;
	// !!!!!!!!Do not forget initialize the class type!!!!!!!!
    robosense_all = new RobosenseALL(lidar_config_path, usr_config_path);

    NormalPointCloudPtr object_cloud_ptr(new NormalPointCloud);
    NormalPointCloudPtr ground_cloud_ptr(new NormalPointCloud);
            
    std::vector<NormalRoboPerceptron> percept_vec, percept_vec_filtered; 
    std::vector<PerceptOutput> out_put_list;

    pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints_filtered(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::io::loadPCDFile("/home/wangbin/sdk_without_ros/data/test_pcd.pcd", *outPoints_filtered);
	cout <<"pointcoud size:"<<outPoints_filtered->width<<"  "<<outPoints_filtered->height<< endl;

    // Here gives the zero velocity and yaw and identity matrix for simplicity, you may have to give the true velocity and yaw of car and 
    // calculate and give the transform matrix from vehicle to global.
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    robosense_all->mainPerceptionProcess(outPoints_filtered, 0,0,mat);

    robosense_all->getGroundPoints(ground_cloud_ptr);
    robosense_all->getObjectPoints(object_cloud_ptr);

    percept_vec = robosense_all->getObjectOriPeceptResults();//original perception-result-list for developer debug.
    percept_vec_filtered = robosense_all->getObjectPeceptResults();//original filtered perception-result-list for developer debug.
    out_put_list = robosense_all->getLastResultsForUser();//Compact perception-result-list for user in lidar coordinate, if you don‘t want debug, you can just use it
    cout <<"Output list size: " <<percept_vec_filtered.size() << endl;


}
