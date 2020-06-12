#include <algorithm>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>

#include <icp_mapping.hpp>

namespace pcl_matching {
typedef Eigen::Vector3d Point;

icp_mapping::icp_mapping(ros::NodeHandle* nh) {
  p_nh = nh;

  // Publisher
  map_pub = p_nh->advertise<PointCloud>("local_map", 1, true);

  path = ros::package::getPath("pcl_matching");
  p_nh->getParam("/icp_mapping_node/topic_name_lidar", topic_name_lidar);
  p_nh->getParam("/icp_mapping_node/map_saved", map_saved);
  p_nh->getParam("/icp_mapping_node/map_resolution", map_resolution);
  p_nh->getParam("/icp_mapping_node/map_update/displacement", update_disp);
  p_nh->getParam("/icp_mapping_node/map_update/time_difference", update_td);

  // Subscriber
  lidar_sub = p_nh->subscribe(topic_name_lidar, 100, &icp_mapping::cb_lidar, this);

  // Read start points of targets
  outfile_pose.open(path + "/data/pose.csv");
  outfile_pose << "time_stamp, x, y, z, qw, qx, qy, qz" << std::endl;
  // open file and save the processed target point
  first_frame = false;

  initial_guess = Eigen::Matrix4f::Identity();
}


// callback function: subscribe lidar pointcloud and process
void icp_mapping::cb_lidar(const sensor_msgs::PointCloud2& msg) {
  double current_time = msg.header.stamp.toSec();
  // Change point cloud type from sensor_msgs::PointCloud2 to pcl::PointXYZI
  PointCloud::Ptr source(new PointCloud);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *source);

  PointCloud::Ptr cloud_clipped(new PointCloud);
  pcl::CropBox<pcl::PointXYZI> box_filter;
  box_filter.setMin(Eigen::Vector4f(-4, -4, -4, 1.0));
  box_filter.setMax(Eigen::Vector4f(4, 4, 4, 1.0));
  box_filter.setInputCloud(source);
  box_filter.setNegative(true);
  box_filter.filter(*cloud_clipped);

  if (first_frame == false) {
    first_frame = true;
    topic_frame_lidar = msg.header.frame_id;
    local_map = *cloud_clipped;
    last_pose = std::make_pair(current_time, Eigen::Matrix4f::Identity());
    return;
  }
  local_map.header.frame_id = "map";
  map_pub.publish(local_map);

  PointCloud::Ptr target(new PointCloud);
  *target = local_map;
  Eigen::Matrix4f icp_transformation = ICP_matching(source, target, initial_guess);
  std::cout << "icp_transformation:\n " << icp_transformation << std::endl;

  // Localization initial guess updated
  last_n_pose.push_back(std::make_pair(current_time, icp_transformation));
  if (last_n_pose.size() == 1) {
    initial_guess = icp_transformation;
  } else {
    Eigen::Matrix4f diff = icp_transformation * last_n_pose.back().second.inverse();
    initial_guess = diff * icp_transformation;
  }

  // ros tf
  Eigen::Affine3f T_ml(icp_transformation);
  Eigen::Quaternionf q_ml(T_ml.rotation());
  transform.setOrigin(tf::Vector3(T_ml.translation().x(),
                                  T_ml.translation().y(),
                                  T_ml.translation().z()));
  transform.setRotation(tf::Quaternion(q_ml.x(),
                                       q_ml.y(),
                                       q_ml.z(),
                                       q_ml.w()));
  br.sendTransform(tf::StampedTransform(transform,
                                        msg.header.stamp,
                                        "map",
                                        msg.header.frame_id));

  // Save lidar pose for each frame
  outfile_pose << std::to_string(current_time) << ", "
               << T_ml.translation().x() << ", "
               << T_ml.translation().y() << ", "
               << T_ml.translation().z() << ", "
               << q_ml.w() << ", "
               << q_ml.x() << ", "
               << q_ml.y() << ", "
               << q_ml.z() << std::endl;

  Eigen::Affine3f last_T(last_pose.second);
  Point current_position{T_ml.translation().x(),
                         T_ml.translation().y(),
                         T_ml.translation().z()};
  Point last_position{last_T.translation().x(),
                      last_T.translation().y(),
                      last_T.translation().z()};

  if ((current_position - last_position).norm() > update_disp
       || (current_time - last_pose.first) > update_td) {
    std::cout << "Map updated" << std::endl;
    PointCloud::Ptr cloud_transform(new PointCloud);
    pcl::transformPointCloud(*cloud_clipped, *cloud_transform, icp_transformation);
    PointCloud::Ptr local_map_(new PointCloud);
    PointCloud::Ptr local_map_filtered(new PointCloud);
    local_map += *cloud_transform;
    last_pose = std::make_pair(current_time, icp_transformation);

    *local_map_ = local_map;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(local_map_);
    sor.setLeafSize(map_resolution, map_resolution, map_resolution);
    sor.filter(*local_map_filtered);
    local_map = *local_map_filtered;
  }

  p_nh->getParam("/icp_mapping_node/map_saved", map_saved);
  if (!map_saved) {
    std::string map_name = path + "/data/map.pcd";
    pcl::io::savePCDFileASCII(map_name, local_map);
    std::cout << "Save " + path + "/data/map.pcd" << std::endl;
    map_saved = true;
    p_nh->setParam("/icp_mapping_node/map_saved", map_saved);
  }
}


icp_mapping::~icp_mapping() {
  outfile_pose.close();
}
}  // namespace pcl_matching
