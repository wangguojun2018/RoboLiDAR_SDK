cmake_minimum_required( VERSION 2.8.3 )
project(percept_resut_receiver)

SET( CMAKE_BUILD_TYPE "Release" )

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS
		roscpp
		rospy
		pcl_ros
		cv_bridge
		)

#find_package(Eigen3 REQUIRED)
find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )

include_directories(
		include
		demo
		display
		communication
		tools
		${catkin_INCLUDE_DIRS}
		#	${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS} )

catkin_package()

add_library(robo_test
		communication/communication.cpp
		display/robo_draw_rviz.cpp)
add_dependencies(robo_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(robo_test ${catkin_LIBRARIES})

add_executable(perception_msg_node
		demo/perception_msg.cpp
		)
add_dependencies(perception_msg_node ${catkin_EXPORTED_TARGETS} robo_test)
target_link_libraries(perception_msg_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} robo_test )
