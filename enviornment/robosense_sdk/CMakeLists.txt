cmake_minimum_required( VERSION 2.8.3 )
project( robosense_sdk )

SET( CMAKE_BUILD_TYPE "Release" )

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS
              cv_bridge
              genmsg
              pcl_conversions
              pcl_ros
              roscpp
              rospy
              std_msgs
              )
find_package( Boost REQUIRED thread signals )
find_package( PkgConfig )
find_package( OpenCV REQUIRED )

catkin_package()

set( libpcap_LIBRARIES -lpcap )

#file(GLOB libso lib/*.so)
link_directories(lib)

include_directories(
    include
    demo
    tools
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_definitions( ${EIGEN_DEFINITIONS} )
add_definitions( ${PCL_DEFINITIONS} )


# test without map
add_executable( test_without_map
                demo/test_without_map.cpp
                tools/robo_draw_rviz.cpp
				tools/communication/communication.cpp
                )
target_link_libraries( test_without_map robosense tracking classification detection preprocessing common module_manager ${catkin_LIBRARIES} ${OpenCV_LIBS} )


# test with map
add_executable( test_with_map
                demo/test_with_map.cpp
                tools/robo_draw_rviz.cpp
				tools/communication/communication.cpp
                )
target_link_libraries( test_with_map robosense tracking classification detection preprocessing common module_manager ${catkin_LIBRARIES} ${OpenCV_LIBS} )

			

