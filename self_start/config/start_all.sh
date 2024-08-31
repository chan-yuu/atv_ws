#!/bin/bash
# 一键重启系统：

# 先关闭所有的节点：
# 需要测试。
pkill -f roscore
pkill -f "rosrun lidar lidar"
pkill -f "rosrun perception gps_imu_pub.py"
pkill -f "python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py"
pkill -f "rosrun faultdiagnosis fault_diagnosis.py"
pkill -f "rs_sdk_demo"
pkill -f rviz
pkill -f "rosrun hmi hmi_node"



PROCESSB=$(ps -ef | grep path_speed_control.py | awk '{print $2}')
kill -9 $PROCESSB
PROCESSB=$(ps -ef | grep plan_net.py | awk '{print $2}')
kill -9 $PROCESSB
PROCESSB=$(ps -ef | grep mark.py | awk '{print $2}')
kill -9 $PROCESSB
PROCESSB=$(ps -ef | grep decision.py | awk '{print $2}')
kill -9 $PROCESSB
PROCESSB=$(ps -ef | grep wire_control_pub.py | awk '{print $2}')
kill -9 $PROCESSB



# PROCESSB=$(ps -ef | grep wire_control_pub.py | awk '{print $2}')
# kill -9 $PROCESSB


# 再重新启动
################################################################终端实验版
#roscore
gnome-terminal --tab --title="roscore" -- bash -c "source ~/las/devel/setup.bash;export ROS_HOSTNAME=localhost;export ROS_MASTER_URI=http://localhost:11311;roscore; exec bash"
sleep 1s
# 开 lidar解析模块
gnome-terminal --tab --title="lidar_ex" -- bash -c "source ~/las/devel/setup.bash;rosrun lidar lidar; exec bash"
# 开gps
sleep 1s
gnome-terminal --tab --title="gps_imu_pub" -- bash -c "source ~/las/devel/setup.bash;sudo chmod 777 /dev/ttyUSB0;rosrun perception gps_imu_pub.py; exec bash"

# camera
sleep 1s
gnome-terminal --tab --title="camera" -- bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py; exec bash"

# 自检 现在运行在了faultdiagnose中
sleep 1s
gnome-terminal --tab --title="self_check" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis self_check.py; exec bash"

# 故障诊断
gnome-terminal --tab --title="fault_diagnosis" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis fault_diagnosis.py; exec bash"

# lidar-sdk
sleep 1s
gnome-terminal --tab --title="lidar_sdk" -- bash -c "cd /home/nvidia/RoboSense/release_orin/release/build/demo;source ~/las/devel/setup.bash;./rs_sdk_demo; exec bash"

sleep 1s
# rviz
gnome-terminal --tab --title="RViz" -- bash -c "source ~/las/devel/setup.bash; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz; exec bash"
sleep 1s

gnome-terminal --tab --title="RViz" -- bash -c "source ~/las/devel/setup.bash; rosrun rviz rviz -d /home/nvidia/las/src/planning/config/rviz15.rviz; exec bash"

# hmi
sleep 1s
gnome-terminal --tab --title="Hmi" -- bash -c "source ~/las/devel/setup.bash;rosrun hmi hmi_node;exec bash"



# # 先关闭所有的节点：
# # 需要测试。
# pkill -f roscore
# pkill -f "rosrun lidar lidar"
# pkill -f "rosrun perception gps_imu_pub.py"
# pkill -f "python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py"
# pkill -f "rosrun faultdiagnosis fault_diagnosis.py"
# pkill -f "rs_sdk_demo"
# pkill -f rviz
# pkill -f "rosrun hmi hmi_node"



# PROCESSB=$(ps -ef | grep path_speed_control.py | awk '{print $2}')
# kill -9 $PROCESSB
# PROCESSB=$(ps -ef | grep plan_net.py | awk '{print $2}')
# kill -9 $PROCESSB
# PROCESSB=$(ps -ef | grep mark.py | awk '{print $2}')
# kill -9 $PROCESSB
# PROCESSB=$(ps -ef | grep decision.py | awk '{print $2}')
# kill -9 $PROCESSB
# PROCESSB=$(ps -ef | grep wire_control_pub.py | awk '{print $2}')
# kill -9 $PROCESSB



# # PROCESSB=$(ps -ef | grep wire_control_pub.py | awk '{print $2}')
# # kill -9 $PROCESSB


# # 再重新启动
# ################################################################终端实验版
# #roscore
# gnome-terminal --tab --title="roscore" -- bash -c "source ~/las/devel/setup.bash;export ROS_HOSTNAME=localhost;export ROS_MASTER_URI=http://localhost:11311;roscore; exec bash"
# sleep 1s
# # 开 lidar解析模块
# gnome-terminal --tab --title="lidar_ex" -- bash -c "source ~/las/devel/setup.bash;rosrun lidar lidar; exec bash"
# # 开gps
# sleep 1s
# gnome-terminal --tab --title="gps_imu_pub" -- bash -c "source ~/las/devel/setup.bash;sudo chmod 777 /dev/ttyUSB0;rosrun perception gps_imu_pub.py; exec bash"

# # camera
# sleep 1s
# gnome-terminal --tab --title="camera" -- bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py; exec bash"

# # 自检 现在运行在了faultdiagnose中
# # sleep 1s
# # gnome-terminal --tab --title="self_check" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis self_check.py; exec bash"

# # 故障诊断
# gnome-terminal --tab --title="fault_diagnosis" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis fault_diagnosis.py; exec bash"

# # lidar-sdk
# sleep 1s
# gnome-terminal --tab --title="lidar_sdk" -- bash -c "cd /home/nvidia/RoboSense/release_orin/release/build/demo;source ~/las/devel/setup.bash;./rs_sdk_demo; exec bash"

# sleep 1s
# # rviz
# gnome-terminal --tab --title="RViz" -- bash -c "source ~/las/devel/setup.bash; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz; exec bash"
# gnome-terminal --tab --title="RViz" -- bash -c "source ~/las/devel/setup.bash; rosrun rviz rviz -d /home/nvidia/las/src/planning/config/rviz15.rviz; exec bash"

# # hmi
# sleep 1s
# gnome-terminal --tab --title="Hmi" -- bash -c "source ~/las/devel/setup.bash;rosrun hmi hmi_node;exec bash"


# 非终端版：
################################################################非终端版
# sleep 1s
# nohup bash -c "source ~/.bashrc;export ROS_HOSTNAME=localhost;export ROS_MASTER_URI=http://localhost:11311;roscore" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun lidar lidar" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun perception gps_imu_pub.py" &
# sleep 1s
# nohup bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun faultdiagnosis fault_diagnosis.py" &
# sleep 1s
# nohup bash -c "cd /home/nvidia/RoboSense/release_orin/release/build/demo;source ~/.bashrc;./rs_sdk_demo" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz" &
# nohup bash -c "source ~/.bashrc; rosrun rviz rviz -d /home/nvidia/panda_ws/src/planning/config/rviz15.rviz" &

# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun hmi hmi_node" &