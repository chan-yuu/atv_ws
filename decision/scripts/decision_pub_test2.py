#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
测试换道的决策指令
'''
import can
import cantools
import rospy
import time
from car_interfaces.msg import CarOriInterface,PathSpeedCtrlInterface, DecisionInterface
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
    pub = rospy.Publisher('/decision_data', DecisionInterface, queue_size=10)

    msg = DecisionInterface()
    msg.Change_Line = 2
    msg.Brake_En = 0
    msg.Target_Travel = 0

    rate = rospy.Rate(100)  # 发布频率为100Hz

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()

 