cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
)

# catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# install(PROGRAMS
#   src/camera_pub.py
#   src/detect_fog.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )