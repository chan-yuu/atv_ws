cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        message_runtime
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
        car_interfaces
        )


find_package(PCL REQUIRED QUIET)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        message_generation message_runtime
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
        CATKIN_DEPENDS car_interfaces
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter1 nodes/points_downsampler.cpp)

add_dependencies(voxel_grid_filter1 ${catkin_EXPORTED_TARGETS})

target_link_libraries(voxel_grid_filter1 ${catkin_LIBRARIES})

add_executable(map_loader nodes/map_loader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ndt_localizer_node nodes/ndt.cpp)
add_dependencies(ndt_localizer_node ${catkin_EXPORTED_TARGETS} car_interfaces_gencpp)

target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
