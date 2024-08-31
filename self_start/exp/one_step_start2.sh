#!/bin/bash
#description: 启动线控内容车走

gnome-terminal --tab --title="wirecontrol" -- bash -c "source ~/.bashrc;roslaunch wirecontrol wirecontrol_pub.launch; exec bash"

# gnome-terminal --tab --title="Control" -- bash -c "source ~/las/devel/setup.bash;rosrun control path_speed_control.py; exec bash"
# sleep 3s
# gnome-terminal --tab --title="Plan" -- bash -c "source ~/las/devel/setup.bash; rosrun planning plan_net.py; exec bash"
# gnome-terminal --tab --title="show" -- bash -c "source ~/las/devel/setup.bash; rosrun show_rviz mark.py; exec bash"

# sleep 1s
# gnome-terminal --tab --title="decision" -- bash -c "source ~/las/devel/setup.bash; rosrun decision decision.py; exec bash"
# sleep 1s
# gnome-terminal --tab --title="Wirecontrol" -- bash -c "source ~/las/devel/setup.bash; rosrun wirecontrol wire_control_pub.py ; exec bash" 



# PROCESSB=$(ps -ef | grep quiting_process.py | awk '{print $2}')
# kill -9 $PROCESSB

# gnome-terminal  --title="Control" -- bash -c "source ~/las/devel/setup.bash;rosrun control path_speed_control.py; exec bash"
# sleep 3s
# gnome-terminal  --title="Plan" -- bash -c "source ~/las/devel/setup.bash; rosrun planning plan_net.py; exec bash"
# gnome-terminal  --title="show" -- bash -c "source ~/las/devel/setup.bash; rosrun show_rviz mark.py; exec bash"

# sleep 1s
# gnome-terminal --title="decision" -- bash -c "source ~/las/devel/setup.bash; rosrun decision decision.py; exec bash"
# sleep 1s
# gnome-terminal --title="Wirecontrol" -- bash -c "source ~/las/devel/setup.bash; rosrun wirecontrol wire_control_pub.py ; exec bash" 


#非终端版
# PROCESSB=$(ps -ef | grep quiting_process.py | awk '{print $2}')
# kill -9 $PROCESSB
# sleep 1s
# nohup bash -c "source ~/.bashrc;rosrun control path_speed_control.py" &
# sleep 3s
# nohup bash -c "source ~/.bashrc; rosrun planning plan_net.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun decision decision.py" &
# sleep 1s
# nohup bash -c "source ~/.bashrc; rosrun wirecontrol wire_control_pub.py" &