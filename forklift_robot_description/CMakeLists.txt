# cmake_minimum_required(VERSION 3.10)
# project(forklift_robot_description)

# find_package(ament_cmake REQUIRED)


# install(
#     DIRECTORY forklift_webots  
#     DESTINATION share/${PROJECT_NAME}
# )

# ament_package()


cmake_minimum_required(VERSION 2.8.3)
project(forklift_robot_description)

find_package(catkin REQUIRED COMPONENTS
#   ament_cmake
)

catkin_package()

install(
  DIRECTORY forklift_webots
  DESTINATION share/${PROJECT_NAME}
)

# ament_package()