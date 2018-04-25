# How to make ROI filter map

ROI filter map is a kind of mask map which is used to filter out the regin we don't care so the so-called Regin Of Interest that defines the zone where the perception sdk mainly cares is left. Since the pointcloud captured by lidar is not a kind of dense and informative data, perception algorithm can not filtering the distraction out automatically by itself. Here introduces the usage of the API for ROI filter map within robosense perception SDK.

All the settings are configured by a file named "info.txt":
```
img_size: 500
grid_size: 0.1
vec_length: 4
vec_width: 6
max_size(max_x,min_x,max_y,min_y): 1651.53 -1119.57 839.531 -724.285
``` 
and the ROI filter map is defined by a sequence of grayscale images with binary values (**255 for ROI zone and 0 for don't care zone**) and format ".png" with the sequential names like: "0_0_lable.png", "0_1_lable.png", "1_2_lable.png" ... .

The configuration file "info.txt" and ROI filter map images must placed in the same path (same folder).

The following steps show the detail:

## 1. Define the perception range

The perception range is defined by a 2D rectangle defined by： 
```
max_size(max_x,min_x,max_y,min_y)
```
"max_x" and "min_x" define the range within X axis, so as the "max_y" and "min_y". The coordiantes are defined by users, can be **local lidar coordinate (for use without map) or global coordinate (for use with map, the translation between local lidar coordiante and global map coordinate should also be provided)**.

## 2. Define the resolution

The rest parameters are mainly set for resolution:
```
img_size: 500
grid_size: 0.1
vec_length: 4
vec_width: 6
```
All the corresponding ROI zone (defined by step 1) are encoded with a sequency of grayscale images. Every image must be a square (with same length and width), the size is defined by "img_size" (Note that this size is measured by "space size", not "pixel size", i.e. "img_size" = "pixel size" x "grid_size").  Then the image need to be projected to the ROI zone, "grid_size" defines the projection resolution, i.e, one pixel corresponds a square space by edge of "grid_size" in meter.  As one ROI filter map image may not cover all the ROI zone, we need a tile schedule, "vec_length" and "vec_width" define the tile number in X and Y direction respectively. We can guess the meaning of the image names, like "1_2_lable.png", means the index of the tile is (1,2), i.e., second column (x axis) and third row (y axis), the index is 0-based. All the tile image should be adjacent to each other and composing a full ROI map.

## 3. Configure the ROI filter map path

When ROI filter map data is done, we have to pass the right path to the SDK to get enabled. The path is configured in the file `"perception_args.xml" ` named "roi_filter_map" and should be enabled by "use_roi".

**We provide an example of roi filter map data.** See it within the same folder.

## 4. How to make ROI filter map images

Users can use any image processing software like PhotoShop to make the mask image. In order to get precise result, you can try this way:

- get the original pointcloud;

- make the pointcloud horizontal as possible through rotation;

- cut off the points high over 2 meters from the ground in the pointcloud, in order to get rid of the influence of crown of trees sideway so that the curb line can be seen clearly;

- project the pointcloud into x-y plane with grid (0.1 meters resolution) to get an 2D image;

- tile the image into pieces if necessary, in order to get the big original image tiled equally by square images, you have to pad it with black pixels;

- for every tile image, trace the edge of road by key points, and use image processing software like PS to get the ROI zone by polygon.

- give right name for every tile image as their names are read as indexes, so don't make it wrong.


## 5. Our own best practice now is open-soured for you !
We provide our own practice when developing the ROI function, namly "**roi_filter_map_building_tool**", you can download and unzip it. We also provide demo testing data containning: **1) static use without map** and **2) dynamic use with map and localizaion**. For the detail of how to use the tool, there is another "**readme.md**" file whthin the package. 








