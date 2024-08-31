#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 作者： CYUN
# 功能描述：
# CAN信号握手

import os
import signal
import threading
import time
import can
import cantools
import rospy
from car_interfaces.msg import CarOriInterface, PathSpeedCtrlInterface, DecisionInterface

class CANInterface:
    def __init__(self, dbc_file, can_channel='can1'):
        self.lock = threading.Lock()
        self.load_dbc(dbc_file)
        self.connect_can(can_channel)
        self.initialize_variables()
        self.setup_messages()
        self.is_exit = False
        self.brake_en_counter = 0
        self.brake_en_threshold = 100  # 100次

    def load_dbc(self, dbc_file):
        self.dbc = cantools.db.load_file(dbc_file)

    def connect_can(self, can_channel):
        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
            self.connect_flag = True
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}")
            self.connect_flag = False

    def initialize_variables(self):
        self.Brake_En = 0
        self.Target_Angle = 0
        self.TargetPedalOpen = 0
        self.Target_Travel = 0
        self.control_flag = 0
        self.decision_flag = 0

    def setup_messages(self):
        self.messages_210 = [
            {
                'message_id': 0x210,
                'message_name': 'IPC_210',
                'signals': {
                    'Parking_Req': 0,
                    'IPC_En': 0,
                    'Target_Gear': 0,
                    'Target_Speed': 0,
                    'Target_Angle': 0,
                    'Brake_En': 0,
                    'Target_Travel': 0,
                    'DippedLamp': 0,
                    'TurnLamp': 0,
                    'FarLamp': 0,
                    'Horn': 0,
                    'Car_OFF': 0,
                    'AlarmLamp': 0,
                    'OutLineLamp': 0
                }
            }
        ]

    #  Parking_Req   # 驻车请求  （0：驻车  1：释放） 
    #  IPC_En   # 使能信号 （0：未使能   1：使能）
    #  Target_Gear    # 车辆档位信号（0x00： P 档；  0x01：倒档R；  0x02：N 档；  0x03：D 档/L档）
    #  TargetPedalOpen  #油门踏板开度： 取值0～100，最小计量单元1%
    #  Target_Angle    # 角度旋转到当前数值对应的角度 (-390°~+390°前轮方向盘转，正为左转)，0°为对应中点位置
    #  Brake_En   # 刹车使能 （0：未使能   1：使能）
    #  Target_Travel   # 行程值 （有效值：0-60，制动为行程控制）
    #  DippedLamp    # 近光灯 （0：关闭，1：打开）
    #  TurnLamp    # 转向灯控制 （0x00：归位；  0x01：左转向；  0x02：右转向）
    #  FarLamp   # 远光灯 （0：关闭，1：打开）
    #  Horn     # 喇叭 （0：关闭，1：打开）
    #  Car_OFF    # 关机 （0：关闭，1：打开）
    #  AlarmLamp    # 双闪灯（APP接入） （0：关闭，1：打开）
    #  OutLineLamp   # 轮廓灯 （0：关闭，1：打开）

    def callback_PathSpeedCtrl(self, msg):
        # with self.lock:
        self.Target_Angle = msg.Target_Angle
        self.TargetPedalOpen = msg.TargetPedalOpen

        self.messages_210[0]['signals']['DippedLamp'] = msg.DippedLamp
        self.messages_210[0]['signals']['TurnLamp'] = msg.TurnLamp
        self.messages_210[0]['signals']['FarLamp'] = msg.FarLamp
        self.messages_210[0]['signals']['Horn'] = msg.Horn
        self.messages_210[0]['signals']['Car_OFF'] = msg.Car_OFF
        self.messages_210[0]['signals']['AlarmLamp'] = msg.AlarmLamp
        self.messages_210[0]['signals']['OutLineLamp'] = msg.OutLineLamp

        if msg.Target_Travel != 0:
            self.messages_210[0]['signals']['Target_Travel'] = msg.Target_Travel
            self.messages_210[0]['signals']['Brake_En'] = 1

        self.control_flag = 1
        # print("sub control")

    def callback_decision(self, msg):
        # with self.lock:
        self.Brake_En = msg.Brake_En
        self.Target_Travel = msg.Target_Travel
        self.decision_flag = 1

    def select_msg(self):
        # with self.lock:
        self.messages_210[0]['signals']['Parking_Req'] = 1
        self.messages_210[0]['signals']['IPC_En'] = 1
        self.messages_210[0]['signals']['Target_Gear'] = 3
        self.messages_210[0]['signals']['Target_Speed'] = self.TargetPedalOpen
        self.messages_210[0]['signals']['Target_Angle'] = self.Target_Angle

        print(self.messages_210[0]['signals']['Target_Speed'])
        print(self.messages_210[0]['signals']['Target_Travel'])


    def send_can_messages(self):
        msg_send = self.dbc.get_message_by_name(self.messages_210[0]['message_name'])
        messages_ = list(msg_send.encode(self.messages_210[0]['signals']))
        messages_send = can.Message(arbitration_id=msg_send.frame_id, data=messages_, is_extended_id=False)
        self.bus.send(messages_send)

    def shutdown(self):
        self.bus.shutdown()

    def signal_handler(self, sig, frame):
        self.is_exit = True

    def run(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('pub_vcu_node', anonymous=False)
        rospy.Subscriber('/path_speed_tracking_data', PathSpeedCtrlInterface, self.callback_PathSpeedCtrl)
        rospy.Subscriber('/decision_data', DecisionInterface, self.callback_decision)

        rate = rospy.Rate(100)  # 100 Hz

        while not self.control_flag and not rospy.is_shutdown():
            rospy.sleep(0.1)
            rospy.logwarn("please wait for control or decision")

        while not self.is_exit and not rospy.is_shutdown():
            start_time = time.time()
            self.select_msg()
            self.send_can_messages()
            rate.sleep()
            print("can frequency:", time.time() - start_time)
            

        self.shutdown()
        print('\nall threads were ended by Ctrl-C')


if __name__ == '__main__':
    script_directory = os.path.dirname(os.path.abspath(__file__))
    relative_path = "../config/IPC_VCU_ZRD.dbc"
    gps_dbc_file = os.path.join(script_directory, relative_path)

    can_interface = CANInterface(dbc_file=gps_dbc_file, can_channel='can1')
    can_interface.run()
