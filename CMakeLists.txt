cmake_minimum_required(VERSION 2.8.3)
project(pcl_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

catkin_package(
  CATKIN_DEPENDS roscpp roslib pcl_ros sensor_msgs
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  roslib
  pcl_ros
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/calib_3d.cpp
# )
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pcl_matching.cpp
  src/ndt_mapping.cpp
)



add_executable(ndt_mapping_node scripts/ndt_mapping_node.cpp)
add_dependencies(ndt_mapping_node ${main_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ndt_mapping_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PCL_LIBRARIES})


