1) non-ROS driver for RS-16:
no ros-depended version driver for rs-16;

2) sdk_without_ros:
an example using robosense perception sdk without using ROS.

The above two packages should be used together, the driver can parse the original udp packets from lidar to pointcloud format in PCL as the input to sdk.
