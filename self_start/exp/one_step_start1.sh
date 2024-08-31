#!/bin/bash
#description: 启动循迹所需内容 除去线控
#

gnome-terminal --tab --title="gps" -- bash -c "source ~/.bashrc;roslaunch perception gps_imu.launch; exec bash"
sleep 3s
gnome-terminal --tab --title="planning" -- bash -c "source ~/.bashrc;roslaunch planning planning.launch; exec bash"
sleep 3s
gnome-terminal --tab --title="v2x" -- bash -c "source ~/.bashrc;rosrun v2x 112.py; exec bash"
sleep 3s

gnome-terminal --tab --title="control" -- bash -c "source ~/.bashrc;roslaunch control_pkg control.launch; exec bash"


# add cyun 8.16:
gnome-terminal --tab --title="rslidar" -- bash -c "source ~/.bashrc;roslaunch rslidar_sdk start.launch; exec bash"
sleep 1s
gnome-terminal --tab --title="wire" -- bash -c "source ~/.bashrc;rosrun wirecontrol wire_control_can_pub.py; exec bash"
sleep 1s
gnome-terminal --tab --title="wire" -- bash -c "source ~/.bashrc;rosrun hmi hmi_node; exec bash"


# gnome-terminal --tab --title="decision" -- bash -c "source ~/.bashrc;rosrun decision decision.py; exec bash"

# # # camera
# sleep 1s
# gnome-terminal --tab --title="camera" -- bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py; exec bash"

# # 自检
# sleep 1s
# gnome-terminal --tab --title="self_check" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis self_check.py; exec bash"

# #故障诊断
# #NOTE
# gnome-terminal --tab --title="fault_diagnosis" -- bash -c "source ~/las/devel/setup.bash;rosrun faultdiagnosis fault_diagnosis.py; exec bash"


# ##终端不显示版：
# sleep 1s
# nohup bash -c "source ~/.bashrc;export ROS_HOSTNAME=localhost;export ROS_MASTER_URI=http://localhost:11311;roscore" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun lidar lidar" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun perception gps_imu_pub.py" &
# sleep 1s
# nohup bash -c "/home/nvidia/Downloads/envs/YOLO/bin/python3 /home/nvidia/panda_ws/src/traffic_light/Traffic_light_detection.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun faultdiagnosis self_check.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun faultdiagnosis fault_diagnosis.py" &
# sleep 1s
# nohup bash -c "cd /home/nvidia/RoboSense/release_orin/release/build/demo;source ~/.bashrc;./rs_sdk_demo" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun rviz rviz -d /home/nvidia/RoboSense/release_orin/release/config/rviz/perception.rviz" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun hmi hmi_node" &


