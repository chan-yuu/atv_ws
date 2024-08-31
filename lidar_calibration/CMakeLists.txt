cmake_minimum_required(VERSION 2.8.3)
project(lidar_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 添加子目录并构建（如果需要的话）
# add_subdirectory(lib/pre_process/roi_clip)
# add_subdirectory(lib/pre_process/voxel_grid_filter)
# add_subdirectory(lib/cluster/euclideanCluster)
# add_subdirectory(lib/bounding_box)
# add_subdirectory(lib/ground_detector/patchwork)
# add_subdirectory(lib/visualization)

add_executable(${PROJECT_NAME}_node src/lidar_calibration_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
