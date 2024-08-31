#!/bin/bash
# 启动雷达和相机相关内容
# chengwulidar
# source ~/panda_ws/devel/setup.bash
# nohup roslaunch lidar lidar_pub_sub.launch >/dev/null 2>&1 &

# # gps
# nohup roslaunch perception gps.launch >/dev/null 2>&1 &

# # camera
# # nohup /home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py >/dev/null 2>&1 &

# nohup rosrun faultdiagnosis self_check.py >/dev/null 2>&1 &

# # lidar
# cd /home/nvidia/RoboSense/release_orin/release/build/demo
# source ~/.bashrc
# nohup ./rs_sdk_demo >/dev/null 2>&1 &
# gnome-terminal --tab --title="RViz" -- bash -c "source ~/.bashrc; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz; exec bash"

# gnome-terminal --tab --title="rslidar" -- bash -c "source ~/.bashrc;roslaunch rslidar_sdk start.launch; exec bash"
gnome-terminal --tab --title="rslidar" -- bash -c "source ~/.bashrc;roslaunch lslidar_driver lslidar_ch128x1_1.launch; exec bash"
sleep 3s

gnome-terminal --tab --title="rslidar" -- bash -c "source ~/.bashrc;roslaunch lidar_calibration lidar_calibration.launch; exec bash"

#gnome-terminal --tab --title="livox_lidar" -- bash -c "source ~/.bashrc;roslaunch livox_ros_driver2 rviz_MID360.launch; exec bash"
sleep 3s
gnome-terminal --tab --title="camera" -- bash -c "source ~/.bashrc;roslaunch camera camera.launch; exec bash"
sleep 3s
gnome-terminal --tab --title="fusion" -- bash -c "source ~/.bashrc;roslaunch lidarObstacleDetect lidar_obstacle_detection.launch; exec bash"
