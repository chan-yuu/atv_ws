#!/bin/bash
# data_publisher control msg
# 记录bag数据
# 修改文件路径
gnome-terminal --tab --title="gps" -- bash -c "source ~/.bashrc;rosbag record -O ~/Documents/data/bag_file.bag \
  /gps_imu \
  /path_speed_tracking_data; exec bash"


