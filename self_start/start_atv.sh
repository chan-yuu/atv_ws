#!/bin/bash
# 启动自动驾驶（最终版）

sleep 1s
gnome-terminal --title="Window1" -- bash -c "cd /home/nvidia/las/src/self_start/exp;./one_step_start1.sh;exec bash"
# sleep 8s
gnome-terminal --title="Window2" -- bash -c "cd /home/nvidia/panda_ws/src/self_start/exp;./one_step_start2.sh;exec bash" 

gnome-terminal --title="Window3" -- bash -c "cd /home/nvidia/panda_ws/src/self_start/exp;./one_step_start2.sh;exec bash" 
