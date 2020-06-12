# pcl_matching

The package is used to implement basic online mapping using PCL library.  

## Requirements
1. Eigen3
2. PCL

## Nodes
1. Node **ndt_mapping_node**
2. Node **icp_mapping_node**

## 1. Node ndt_mapping_node

### (a) Setup
Before we start running the mapping module, there is one configuration file needed to be specfied.  

1. path:  
Their path is  
```
~/pcl_matching/config/config.yaml
```

2. format:  
**config.yaml**
> topic_name_lidar: "/points_raw"  
> map_saved: true  
> map_resolution: 0.02  
> map_update:  
>   displacement: 5  
>   time_difference: 1  

**topic_name_lidar** is the topic name of point cloud.  
**map_saved** is used to indicate the node to save the current map.  
(i.e. If you intend to save the current map, type ```rosparam set /ndt_mapping_node/map_saved false```, it'll save the current map in ~/pcl_matching/data/map.pcd.)  
**map_resolution** is the resolution of built map. It use voxel grid filter to downsample the map point cloud.  
**map_update/displacement** and **map_update/time_difference** determines when to update map (add new point cloud to map), either by the displacement or time difference to last updated pose.  

### (b) Getting Started.
1. Launch the node   
```
roslaunch pcl_matching ndt_mapping.launch
```

### (c) Subscribed Topics
* /points_raw (sensor_msgs/PointCloud2)  
Point cloud for building map.  

### (d) Published Topics
* /ndt_mapping_node/local_map (sensor_msgs/PointCloud2)  
Built map. The first LiDAR frame is set as the map frame.  

## 2. Node icp_mapping_node

### (a) Setup
Before we start running the mapping module, there is one configuration file needed to be specfied.  

1. path:  
Their path is  
```
~/pcl_matching/config/config.yaml
```

2. format:  
**config.yaml**
> topic_name_lidar: "/points_raw"  
> map_saved: true  
> map_resolution: 0.02  
> map_update:  
>   displacement: 5  
>   time_difference: 1  


**topic_name_lidar** is the topic name of point cloud.  
**map_saved** is used to indicate the node to save the current map.  
(i.e. If you intend to save the current map, type ```rosparam set /icp_mapping_node/map_saved false```, it'll save the current map in ~/pcl_matching/data/map.pcd.)  
**map_resolution** is the resolution of built map. It use voxel grid filter to downsample the map point cloud.  
**map_update/displacement** and **map_update/time_difference** determines when to update map (add new point cloud to map), either by the displacement or time difference to last updated pose.  

### (b) Getting Started.
1. Launch the node   
```
roslaunch pcl_matching icp_mapping.launch
```

### (c) Subscribed Topics
* /points_raw (sensor_msgs/PointCloud2)  
Point cloud for building map.  

### (d) Published Topics
* /icp_mapping_node/local_map (sensor_msgs/PointCloud2)  
Built map. The first LiDAR frame is set as the map frame.  

* Result of ndt mapping
<img src="https://github.com/tom13133/pcl_matching/blob/master/images/mapping.png" width="1000">
