cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sdk_without_ros)

SET( CMAKE_BUILD_TYPE "Release" )

set (CMAKE_CXX_STANDARD 11)

find_package( Boost REQUIRED thread signals )
find_package( PkgConfig )
find_package( OpenCV REQUIRED )
message("Open CV version is ${OpenCV_VERSION}")

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})

set(libpcap_LIBRARIES -lpcap)

link_directories(/home/wangbin/sdk_without_ros/lib) # please replace the "usr_name" to your own

include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_definitions( ${EIGEN_DEFINITIONS} )
add_definitions( ${PCL_DEFINITIONS} )



add_executable(sdk_without_ros demo/sdk_without_ros.cpp)
target_link_libraries(sdk_without_ros robosense classification tracking detection preprocessing common module_manager ${OpenCV_LIBS} ${PCL_LIBRARIES} ${libpcap_LIBRARIES})


