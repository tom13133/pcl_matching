#include <algorithm>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>

#include <ndt_mapping.hpp>

namespace pcl_matching {
typedef Eigen::Vector3d Point;

ndt_mapping::ndt_mapping(ros::NodeHandle* nh) {
  p_nh = nh;

  // Publisher
  map_pub = p_nh->advertise<PointCloud>("local_map", 1, true);

  path = ros::package::getPath("pcl_matching");
  p_nh->getParam("/ndt_mapping_node/topic_name_lidar", topic_name_lidar);
  p_nh->getParam("/ndt_mapping_node/map_saved", map_saved);

  // Subscriber
  lidar_sub = p_nh->subscribe(topic_name_lidar, 100, &ndt_mapping::cb_lidar, this);

  // Read start points of targets
  outfile_pose.open(path + "/data/pose.csv");
  outfile_pose << "time_stamp, x, y, z, qw, qx, qy, qz" << std::endl;
  // open file and save the processed target point
  first_frame = false;

  initial_guess = Eigen::Matrix4f::Identity();
  last_pose = Eigen::Matrix4f::Identity();
}


// callback function: subscribe lidar pointcloud and process
void ndt_mapping::cb_lidar(const sensor_msgs::PointCloud2& msg) {
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
    return;
  }
  local_map.header.frame_id = "map";
  map_pub.publish(local_map);

  PointCloud::Ptr target(new PointCloud);
  *target = local_map;
  Eigen::Matrix4f ndt_transformation = NDT_matching(source, target, initial_guess);
  std::cout << "ndt_transformation:\n " << ndt_transformation << std::endl;

  // Localization initial guess updated
  last_n_pose.push_back(std::make_pair(current_time, ndt_transformation));
  if (last_n_pose.size() == 1) {
    initial_guess = ndt_transformation;
  } else {
    Eigen::Matrix4f diff = ndt_transformation * last_n_pose.back().second.inverse();
    initial_guess = diff * ndt_transformation;
  }

  // ros tf
  Eigen::Affine3f T_ml(ndt_transformation);
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

  Eigen::Affine3f last_T(last_pose);
  Point current_position{T_ml.translation().x(),
                         T_ml.translation().y(),
                         T_ml.translation().z()};
  Point last_position{last_T.translation().x(),
                      last_T.translation().y(),
                      last_T.translation().z()};

  if ((current_position - last_position).norm() > 5) {
    std::cout << "Map updated" << std::endl;
    PointCloud::Ptr cloud_transform(new PointCloud);
    pcl::transformPointCloud(*cloud_clipped, *cloud_transform, ndt_transformation);
    PointCloud::Ptr local_map_(new PointCloud);
    PointCloud::Ptr local_map_filtered(new PointCloud);
    local_map += *cloud_transform;
    last_pose = ndt_transformation;

    *local_map_ = local_map;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(local_map_);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(*local_map_filtered);
    local_map = *local_map_filtered;
  }

  p_nh->getParam("/ndt_mapping_node/map_saved", map_saved);
  if (!map_saved) {
    std::string map_name = path + "/data/map.pcd";
    pcl::io::savePCDFileASCII(map_name, local_map);
    std::cout << "Save " + path + "/data/map.pcd" << std::endl;
    map_saved = true;
    p_nh->setParam("/ndt_mapping_node/map_saved", map_saved);
  }
}


ndt_mapping::~ndt_mapping() {
  outfile_pose.close();
}
}  // namespace pcl_matching
