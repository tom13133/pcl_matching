////////////////////////////////////////////////////////////////////////////////
//
// Filename:      icp_mapping.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// This file defines a class icp_mapping to subscribe point cloud and
// estimate the poses with time stamp.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
//
// This file is part of {pcl_matching}.
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO(Yu-Han):
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl_matching.hpp>

namespace pcl_matching {

// class icp_mapping subscribe point cloud, then publish processed map
class icp_mapping {
 public:
  explicit icp_mapping(ros::NodeHandle* nh);

  ros::NodeHandle* p_nh;

  // Publisher
  ros::Publisher map_pub;

  // Subscriber
  ros::Subscriber lidar_sub;

  // callback function: subscribe original PointCloud and process
  void cb_lidar(const sensor_msgs::PointCloud2& msg);

  ~icp_mapping();

 private:
  std::ofstream outfile_pose;

  std::string topic_name_lidar;
  std::string topic_frame_lidar;
  std::string path;

  PointCloud local_map;

  bool first_frame;
  bool map_saved;
  float map_resolution;
  float update_disp, update_td;
  std::vector<std::pair<double, Eigen::Matrix4f>> last_n_pose;
  Eigen::Matrix4f initial_guess;
  std::pair<double, Eigen::Matrix4f> last_pose;

  tf::TransformBroadcaster br;
  tf::Transform transform;
};
}  // namespace pcl_matching
