#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 功能描述：本文件主要用于自动驾驶前的自检判断，根据车门开闭、EBS故障等信息，来确定是否可以进行自动驾驶，并同时输出当前需要的部分信息

import can
import cantools
import rospy
import time
import std_msgs.msg
from car_interfaces.msg import CarOriInterface,PathSpeedCtrlInterface
import os
import signal
import json


safe_detect = {}    # 用于记录安全检查的相关情况
is_exist = {"0C05A5D0": False, "0C06A5D0": False, "0C07A5D0": False, "0C09A5D0": False}       # 用于判断是否收到相应报文
allow_auto = False # 用于自检最终判定是否可以进行自动驾驶


def msg_recv_from_vcu(dbc, bus):  # 从整车VCU中得到车辆目前实际相关信息
    global safe_detect
    global is_exist

    messages_recv = bus.recv()

    if messages_recv.arbitration_id == 0x0C05A5D0:
        is_exist["0C05A5D0"] = True
        decoded_messages_05 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
        # msg_car_ori.steerangle = decoded_messages_05['act_steering_angle']  #   当前角度
        safe_detect["steering_state"] = decoded_messages_05['act_steering_state']  # 1为自动驾驶，4为手动模式，5为人工介入模式，6为警告模式，7为错误，8为力矩叠加

    if messages_recv.arbitration_id == 0x0C06A5D0:
        is_exist["0C06A5D0"] = True
        decoded_messages_06 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
        safe_detect["EBS_state"] = decoded_messages_06['EBS_state']  # 检测为1是故障，1为正常
        safe_detect["brake_state"] = decoded_messages_06["act_brake_pedal"]  # 检测为1有刹车，0为无刹车（需要确认是手刹还是脚刹）

    if messages_recv.arbitration_id == 0x0C07A5D0:
        is_exist["0C07A5D0"] = True
        decoded_messages_07 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
        safe_detect["front_door_state"] = decoded_messages_07["act_front_door_state"]  # 前门检测为开置1，关为0
        safe_detect["middle_door_state"] = decoded_messages_07["act_middle_door_state"]  # 中门检测为开置1，关为0

    if messages_recv.arbitration_id == 0x0C09A5D0:
        is_exist["0C09A5D0"] = True
        decoded_messages_09 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)


def main():
    global safe_detect
    global is_exist
    global allow_auto

    rospy.init_node('self_check', anonymous=False)
    self_check_pub = rospy.Publisher('/from_vcu_self_check', std_msgs.msg.String, queue_size=10)

    signal.signal(signal.SIGINT, quit)
    gps_dbc_file = '/home/nvidia/panda_ws/src/wirecontrol/config/car_vcu2.dbc'
    dbc = cantools.db.load_file(gps_dbc_file)
    bus = can.interface.Bus(channel='can0', bustype='socketcan')


    try:
        while True :        # 循环判断当前是否满足自动驾驶条件
            while True:     # 循环遍历拿到所有整车的CAN报文，并开始自检判断是否可以自动驾驶
                msg_recv_from_vcu(dbc, bus)
                can_rec_num = 0
                for i in is_exist:        # 如果is_exist中所有的报文接收都为True
                    if is_exist[i] == True:
                        can_rec_num += 1
                if can_rec_num == 4:             # 说明四个报文均接收到了，均为True，则可以退出循环
                    break

            if safe_detect["steering_state"] in range(5,8):
                print("驾驶员手动接入！！！自动驾驶解除")
                allow_auto = False
            elif safe_detect["front_door_state"] == 1:
                print("前门车门未关！！！不可以进行自动驾驶")
                allow_auto = False
            elif safe_detect["middle_door_state"] == 1:
                print("中间车门未关！！！不可以进行自动驾驶")
                allow_auto = False
            # elif safe_detect["EBS_state"] == 1:
            #     print("车辆EBS系统发生故障！！！不可以进行自动驾驶")
            #     allow_auto = False          # 如果不满足条件，则不可以运行自动驾驶
            else:
                print("允许自动驾驶")
                allow_auto = True   # 否则可以运行条件
            time.sleep(0.01)
            body_state = {"allow_auto":allow_auto, "detect_state":safe_detect}
            self_check_pub.publish(json.dumps(body_state))
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
