#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 作者： cyun
# version： 使用class管理相关的函数和变量

import rospy
import signal
import os
import threading
import can
import cantools
from car_interfaces.msg import GpsImuInterface, CarOriInterface
from gps_imu import GpsImu
import prcolor

def quit(signal, frame):
    rospy.logwarn("Stopping...")
    rospy.signal_shutdown("Quit signal received")

def main():
    signal.signal(signal.SIGINT, quit)
    rospy.init_node("gps_imu_pub")
    rospy.loginfo("\033[1;32m----> GPS started.\033[0m")
    
    # gps_dbc_file = rospy.get_param("~line_length", '/home/nvidia/atv_ws/src/perception/config/CAN_.dbc')
    gps_dbc_file = rospy.get_param("~gps_dbc_file", '/home/nvidia/atv_ws/src/perception/config/CAN_.dbc') # 可以从launch文件中进行修改
    gps_can_channel = rospy.get_param("~gps_can_channel", 'can0')
    gps_dbc_file = '/home/nvidia/atv_ws/src/perception/config/CAN_.dbc'
    gps_can_channel = 'can0'

    gps_imu = GpsImu(gps_dbc_file, gps_can_channel)

    pub = rospy.Publisher('/gps_imu', GpsImuInterface, queue_size=10)
    rospy.Subscriber('car_ori_data', CarOriInterface, gps_imu.call_back_VCU) # gps给出的速度效果不好，所以用线控反馈替代

    while not rospy.is_shutdown():
        gps_imu.publish_data(pub)
        # prcolor.prGreen("gps is", gps_imu.GPS_MSG.satellite_status, "now_speed is:", gps_imu.now_speed)
        print(gps_imu.GPS_MSG.satellite_status)
if __name__ == "__main__":
    main()
