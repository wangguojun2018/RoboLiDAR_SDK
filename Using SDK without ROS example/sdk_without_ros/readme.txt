1) Please replace the path values in cmakelists.txt and demo/sdk_without_ros.cpp to your local and recompile to run;
2) Please make sure the underlaying libraries, such as boost, eigen, opencv, pcl and libpcap are configured right before hand.

Notice that this example is tested on Ubuntu 14.04, if your system is note same, please replace the corresponding /lib .

How to run:
================================================

mkdir build
cd build
cmake ..
make
./sdk_without_ros
================================================
