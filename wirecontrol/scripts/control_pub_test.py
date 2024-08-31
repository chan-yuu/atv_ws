#!/usr/bin/python3
# -*- coding: utf-8 -*-


import can
import cantools
import rospy
import time
from car_interfaces.msg import CarOriInterface,PathSpeedCtrlInterface
import os
import signal
import time
import can
import cantools
import rospy
import signal
import os



def main():

    rospy.init_node('path_speed_ctrl_publisher')

    # 创建一个Publisher，指定消息类型为PathSpeedCtrl，话题名称为'/path_speed_ctrl'
    pub = rospy.Publisher('/path_speed_tracking_data', PathSpeedCtrlInterface, queue_size=10)

    # 创建一个PathSpeedCtrl对象，将所有字段的值设置为0
    msg = PathSpeedCtrlInterface()
    msg.Parking_Req = 0
    msg.IPC_En = 1
    msg.Target_Gear = 3
    msg.TargetPedalOpen = 0
    msg.Target_Angle = 0
    msg.Brake_En = 0
    msg.Target_Travel = 0
    msg.DippedLamp = 0
    msg.TurnLamp = 0
    msg.FarLamp = 0
    msg.Horn = 0
    msg.Car_OFF = 0
    msg.AlarmLamp = 0
    msg.OutLineLamp = 0
    rate = rospy.Rate(100)  # 发布频率为100Hz 0.01s

    while not rospy.is_shutdown():
        # 发布消息
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()

