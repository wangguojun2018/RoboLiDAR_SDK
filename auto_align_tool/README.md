# README

## Introduction

this project include two packages: control_tool and rslidar

we add some code in rslidar, so you must build our rslidar package then control tool will be aviliable.

## Dependence

- ros indigo(Ubuntu 14.04)
- qt  5.2.1 or higher

## How to Build


```
% cd ws/src
% git clone git@192.168.1.20:moujiajun/play_tool.git
% cd ..
% catkin_make
    
```

## How to Start


- add a "save_data" dirctory in control_tool/ , the tool can save .txt and .pcd data in this directory.

```
% mkdir data
% mkdir save_data
% roslaunch control_tool demo.launch
% push button "load",choose a directory had pcd file ,and load a pcd file
```

- 在auto_align包中新建save_data文件夹,该文件夹是用于保存数据的，最后保存的pcd文件和txt文件都在这个文件夹中
- roslaunch control_tool demo.launch
- 在交互面板中，点击load，至一个保存了pcd文件的文件夹中，会load该文件夹中所有的pcd文件
- 设置adjust中的“set origin lidar height”
- 调整x,y,z,roll,pitch,yaw，将点云地面与参考平面对应
- 点击save保存

##demo

![demo](http://192.168.1.20/moujiajun/program_primer/raw/440d8f5709530ce5d23310438eea6305b90d87fa/img/2017-09-05%2017:47:55%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png) 

