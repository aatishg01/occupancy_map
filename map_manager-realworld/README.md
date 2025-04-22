# 3D semantic map for multi heterogeneous robots
This repo contains 3D [occupancy map](https://en.wikipedia.org/wiki/Occupancy_grid_mapping) and semantic object mapping working with FAST_LIO mapping for multiple heterognenous robots.
```
sudo ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
```

### I. Install
```
sudo apt-get install ros-noetic-octomap*
git clone https://github.com/Shawn207/map_manager.git
cd ~/catkin_ws
catkin_make
```
### II. Run demo (simulation)
1. You can run it with [Roboteam_simulator](https://github.com/Shawn207/roboteam_simulator) for ```main``` branch
2. Or run it with ROS bag with FAST_LIO on ```realworld``` branch
3. For semantic mapping with SPOT sensors, run [FAST_LIO_FOR_Roboteam](https://github.com/Shawn207/FAST_LIO.git) with /rviz/spot_map_lio.rviz. 
Run the regular single robot occupancy map launch: 
```
roslaunch map_manager occupancy_map.launch
```
Run the occupancy map for multiple robots:
```
roslaunch map_manager multi_robo_occ_map_ugv.launch
```


### III. Parameters
Please find parameters in ```map_manager/cfg/***.yaml``` files.
For semantic mapping, remember to check yolo topic as well as pointcloud input

### IV. ROS Topics
Subsribe the following topics for occupancy and ESDF map:
  - Localization topic: ```/odom_ugv(uav)``` (please enter the name of your topic in the parameter files)
  - Depth camera topic: ```/depth_ugv(uav)``` (defined in the config file)
  - Lidar point cloud topic: ```/point_cloud_ugv```
  
Publish the following topics:
  - occupancy map visualization: ```occupancy_map/inflated_voxel_map```(under robot namespace if running multi-robot mapping)
  - object map visualization ```occupancy_map/object_map```, ```occupancy_map/object_dense_cloud```







