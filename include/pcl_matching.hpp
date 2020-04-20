////////////////////////////////////////////////////////////////////////////////
//
// Filename:      pcl_matching.hpp
// Authors:       Yu-Han, Hsueh
//
//////////////////////////////// FILE INFO /////////////////////////////////////
//
// Scan matching code using pcl library
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2020 Yu-Han, Hsueh <zero000.ece07g@nctu.edu.tw>
//
// This file is part of {pcl_matching}.
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_PCL_MATCHING_HPP_
#define INCLUDE_PCL_MATCHING_HPP_
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Geometry>
#include "Eigen/Dense"

namespace pcl_matching {
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

Eigen::Matrix4f NDT_matching(const PointCloud::Ptr source,
                             const PointCloud::Ptr target,
                             const Eigen::Matrix4f& initial_guess);

Eigen::Matrix4f ICP_matching(const PointCloud::Ptr source,
                             const PointCloud::Ptr target,
                             const Eigen::Matrix4f& initial_guess);

}  // namespace pcl_matching
#endif  // INCLUDE_PCL_MATCHING_HPP_
