#!/bin/bash


pkill -f "roslaunch perception gps_imu.launch"
pkill -f "roslaunch planning planning.launch"
pkill -f "roslaunch control_pkg control.launch"

pkill -f "roslaunch wirecontrol wirecontrol_pub.launch"

pkill -f "roslaunch rslidar_sdk start.launch"
pkill -f "roslaunch livox_ros_driver2 rviz_MID360.launch"
pkill -f "roslaunch fusion_pointclouds fusion_pointclouds.launch"
pkill -f "roslaunch lidarObstacleDetect lidar_obstacle_detection.launch"
pkill -f "rosrun decision decision.py"


# 其他形式
# PROCESSB=$(ps -ef | grep path_speed_control.py | awk '{print $2}')
# kill -9 $PROCESSB

# PROCESSB=$(ps -ef | grep plan_net.py | awk '{print $2}')
# kill -9 $PROCESSB

# PROCESSB=$(ps -ef | grep decision.py | awk '{print $2}')
# kill -9 $PROCESSB

# PROCESSB=$(ps -ef | grep wire_control_pub.py | awk '{print $2}')
# kill -9 $PROCESSB
# PROCESSB=$(ps -ef | grep mark.py | awk '{print $2}')
# kill -9 $PROCESSB

# gnome-terminal  --title="quit" -- bash -c "rosrun wirecontrol quiting_process.py; exec bash"
#f终端
# nohup bash -c "source ~/.bashrc;rosrun wirecontrol quiting_process.py" &