#!/usr/bin/python3
# -*- coding: utf-8 -*-

# 作者： CYUN
# 功能描述
# 接收CAN并解析后发布car_ori

import can
import cantools
import rospy
import time
from car_interfaces.msg import CarOriInterface, PathSpeedCtrlInterface
import os
import signal
from rospy.rostime import Time


class CANReceiver:
    def __init__(self, dbc_file, can_channel='can1'):
        self.load_dbc(dbc_file)
        self.connect_can(can_channel)
        self.running_flag = True
        self.setup_publisher()
        self.flag_time =False
        self.flag_vel = False
        self.flag_fault = False
        self.flag_soc = False
        self.flag_vcu = False
        
    def load_dbc(self, dbc_file):
        self.dbc = cantools.db.load_file(dbc_file)

    def connect_can(self, can_channel):
        try:
            self.bus = can.interface.Bus(channel=can_channel, bustype='socketcan')
            self.connect_flag = True
        except Exception as e:
            print(f"Failed to connect to CAN bus: {e}")
            self.connect_flag = False

    def setup_publisher(self):
        self.pub = rospy.Publisher('/car_ori_data', CarOriInterface, queue_size=10)

    def signal_handler(self, sig, frame):
        self.running_flag = False

    def run(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('pub_vcu', anonymous=False)
        rospy.logwarn("start")

        while not rospy.is_shutdown():
            start = time.time()
            messages_recv = self.bus.recv()

            msg_car_ori = CarOriInterface()
            if messages_recv.arbitration_id == 0x201:
                decoded_messages = self.dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
                msg_car_ori.Angle = decoded_messages['Angle']  # 车轮转角反馈
                msg_car_ori.Drive_Mode = decoded_messages['Drive_Mode']  # 驾驶模式
                msg_car_ori.EPOSts = decoded_messages['EPOSts']  # 急停 #类型
                msg_car_ori.Gear = decoded_messages['Gear']  # 档位  ##类型
                msg_car_ori.Car_Speed = decoded_messages['Car_Speed'] / 3.6  # 车速
                msg_car_ori.Motor_Torque = decoded_messages['Motor_Torque']  # 油门反馈
                # msg_car_ori.YK_F = decoded_messages['YK_F']  # 复位H
                # msg_car_ori.YK_H = decoded_messages['YK_H']  # 遥控F
                msg_car_ori.EPB_SystemState = decoded_messages['EPB_SystemState']  # EPB开关状态
                flag_vel = True
                # print(1)
                self.pub.publish(msg_car_ori)
                rospy.Rate(100).sleep()  ##???


            elif messages_recv.arbitration_id == 0x202:
                decoded_messages = self.dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
                msg_car_ori.Fault1 = decoded_messages['Fault1']  # 故障代码1
                msg_car_ori.Fault2 = decoded_messages['Fault2']  # 故障代码2
                msg_car_ori.Fault3 = decoded_messages['Fault3']  # 故障代码3
                msg_car_ori.Fault4 = decoded_messages['Fault4']  # 故障代码4
                msg_car_ori.Mileage = decoded_messages['Mileage']  # 累计里程
                flag_fault = True
                # print(1)

            elif messages_recv.arbitration_id == 0x203:
                decoded_messages = self.dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
                msg_car_ori.Brake_Pressure = decoded_messages['Brake_Pressure']  # 制动压力采样值
                msg_car_ori.LR_WheelSpeed = decoded_messages['LR_WheelSpeed']
                msg_car_ori.RR_WheelSpeed = decoded_messages['RR_WheelSpeed']
                msg_car_ori.SOC = decoded_messages['SOC']  # SOC
                msg_car_ori.CarSts1 = decoded_messages['CarSts1']  # 车辆状态1
                msg_car_ori.CarSts2 = decoded_messages['CarSts2']  # 车辆状态2
                flag_soc = True

            elif messages_recv.arbitration_id == 0x261:
                decoded_messages = self.dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
                msg_car_ori.Time = decoded_messages['Time']  # 日期
                msg_car_ori.LV_Main = decoded_messages['LV_Main']  # 主版本
                msg_car_ori.LV_zCode = decoded_messages['LV_zCode']  # 子版本
                msg_car_ori.LV_Sun = decoded_messages['LV_Sun']  # 修订版本
                self.flag_time = True

            elif messages_recv.arbitration_id == 0x260:
                decoded_messages = self.dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
                msg_car_ori.CarStartState = decoded_messages['CarStartState']  # 车辆启动状态
                msg_car_ori.VCU_Service_Voltage = decoded_messages['VCU_Service_Voltage']  # VCU供电电压
                msg_car_ori.VCU_Sts = decoded_messages['VCU_Sts']  # VCU状态
                msg_car_ori.Fault = decoded_messages['Fault']  # 整车故障
                flag_vcu = True

            # if self.flag_time and self.flag_vel and self.flag_fault and self.flag_soc and self.flag_vcu:
            # if self.flag_vel:
            #     self.pub.publish(msg_car_ori)
            #     self.flag_vel = False
            #     # self.flag_fault = False
            #     # self.flag_soc = False
            #     # self.flag_vcu = False
            #     # self.flag_time = False
            #     rospy.Rate(100).sleep()  ##???
            #     end = time.time()
            #     print("time", end - start)

    def shutdown(self):
        self.running_flag = False
        self.bus.shutdown()


if __name__ == '__main__':
    script_directory = os.path.dirname(os.path.abspath(__file__))
    relative_path = "../config/IPC_VCU_ZRD.dbc"
    gps_dbc_file = os.path.join(script_directory, relative_path)

    can_receiver = CANReceiver(dbc_file=gps_dbc_file, can_channel='can1')
    can_receiver.run()
