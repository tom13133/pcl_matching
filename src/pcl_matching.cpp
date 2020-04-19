#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>
#include <pcl_matching.hpp>

namespace pcl_matching {

Eigen::Matrix4f NDT_matching(const PointCloud::Ptr source,
                             const PointCloud::Ptr target,
                             const Eigen::Matrix4f& initial_guess) {
  // lidar localization using NDT
  PointCloud::Ptr cloud_filtered(new PointCloud);
  PointCloud::Ptr map(new PointCloud);
  PointCloud::Ptr temp_1(new PointCloud);
  PointCloud::Ptr temp_2(new PointCloud);

  // NDT source
  pcl::CropBox<pcl::PointXYZI> box_filter;
  box_filter.setMin(Eigen::Vector4f(-2, -2, -2, 1.0));
  box_filter.setMax(Eigen::Vector4f(2, 2, 2, 1.0));
  box_filter.setInputCloud(source);
  box_filter.setNegative(true);
  box_filter.filter(*temp_1);

  box_filter.setMin(Eigen::Vector4f(-40, -40, -40, 1.0));
  box_filter.setMax(Eigen::Vector4f(40, 40, 40, 1.0));
  box_filter.setInputCloud(temp_1);
  box_filter.setNegative(false);
  box_filter.filter(*temp_2);

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(temp_2);
  sor.setLeafSize(0.3f, 0.3f, 0.3f);
  sor.filter(*cloud_filtered);

  // NDT target
  Eigen::Affine3f T(initial_guess);
  box_filter.setMin(Eigen::Vector4f(T.translation().x()-50,
                                    T.translation().y()-50,
                                    T.translation().z()-50,
                                    1.0));
  box_filter.setMax(Eigen::Vector4f(T.translation().x()+50,
                                    T.translation().y()+50,
                                    T.translation().z()+50,
                                    1.0));
  box_filter.setInputCloud(target);
  box_filter.filter(*temp_1);
  sor.setInputCloud(temp_1);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*map);

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.1);
  ndt.setResolution(2.0);
  ndt.setMaximumIterations(25);

  ndt.setInputSource(cloud_filtered);
  ndt.setInputTarget(map);

  PointCloud::Ptr cloud_transform(new PointCloud);
  ndt.align(*cloud_transform, initial_guess);

  return ndt.getFinalTransformation();
}
}  // namespace pcl_matching
