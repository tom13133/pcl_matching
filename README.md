# pcl_matching

The package is used to implement basic real-time mapping using PCL library.  

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

**topic_name_lidar** is the topic name of point cloud.  
**map_saved** is used to indicate the node to save the current map.  
(i.e. If you intend to save the current map, type ```rosparam set /ndt_mapping_node/map_saved false```, it'll save the current map in ~/pcl_matching/data/map.pcd.)  

### (b) Getting Started.
1. Launch the node   
```
roslaunch pcl_matching ndt_mapping.launch
```

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

**topic_name_lidar** is the topic name of point cloud.  
**map_saved** is used to indicate the node to save the current map.  
(i.e. If you intend to save the current map, type ```rosparam set /icp_mapping_node/map_saved false```, it'll save the current map in ~/pcl_matching/data/map.pcd.)  

### (b) Getting Started.
1. Launch the node   
```
roslaunch pcl_matching icp_mapping.launch
```

* Result of ndt mapping
<img src="https://github.com/tom13133/pcl_matching/blob/master/images/mapping.png" width="1000">
