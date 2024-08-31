#!/usr/bin/python3
# -*- coding: utf-8 -*-


import time
import rospy
from car_interfaces.msg import GpsImuInterface
from hmi.msg import FaultDiagnosisInterface
from car_interfaces.msg import RadarStateInterface
import can
import cantools
import os,sys
import signal
import json
import std_msgs.msg

# 获取当前脚本文件的所在目录
script_directory = os.path.dirname(os.path.abspath(__file__))
# 图片文件相对于脚本文件的路径
image_relative_path = 'lib'
# 构建图片文件的完整路径
icon_path = os.path.join(script_directory, image_relative_path)
# absolute_path = os.path.abspath(icon_path)

# print(absolute_path)
sys.path.append(icon_path)
from base_function import *

lidar_start = 0 #雷达启动
Gps_state_fault = False 	#惯导通讯故障 0：异常；1：正常
can_state = False #can通信故障

is_exit = False

def msg_recv_from_vcu(dbc,bus):
    global can_state
    try:
        # print("1111111111")
        # if len(bus.recv())!=0:
        # message = await bus.recv_async() 
        messages_recv = bus.recv(timeout=0.1)
        if messages_recv.arbitration_id == 0x0C06A5D0:
            decoded_messages = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
            can_state = True ##有can信号就true
        else:
            pass
    except:
        can_state = False
        rospy.logwarn("vcu通信异常")

def call_back_CurGNSS(msg):
    global Gps_state_fault 
    try:
        content = json.loads(msg.data)
        now_pos_x = content["UTM_x"]
        if now_pos_x !=0:
            Gps_state_fault = True
    except:
        Gps_state_fault = False
        rospy.logwarn('gps error')


def call_back_lidarState(msg):
    #认为lidar解析通过后就启动雷达了
    global lidar_start
    try:
        content = json.loads(msg.data)
        lidar_start = content["lidar_communication_fault"]
    except:
         lidar_start = 0

def signal_handler(signal, frame):
    global is_exit
    is_exit = True

def main():
    global is_exit
    global lidar_start
    global Gps_state_fault
    global can_state
    
    signal.signal(signal.SIGINT, signal_handler)
    # script_directory = os.path.dirname(os.path.abspath(__file__))
    # relative_path = "../config/car_vcu2.dbc"
    # gps_dbc_file = os.path.join(script_directory, relative_path)

    # dbc = cantools.db.load_file(gps_dbc_file)
    # bus = can.interface.Bus(channel='can0', bustype='socketcan')
    
    rospy.init_node('fault_diagnosis')
    pub = rospy.Publisher('fault_diagnosis_data', FaultDiagnosisInterface, queue_size=10)
    rospy.Subscriber('/ztbus/location', std_msgs.msg.String, call_back_CurGNSS)
    rospy.Subscriber('objectlist_json',std_msgs.msg.String,call_back_lidarState)
    prGreen("start success")
    while not is_exit:
        # msg_recv_from_vcu(dbc,bus)
        msg_faultcode = FaultDiagnosisInterface()
        msg_faultcode.lidar_start = 1
        msg_faultcode.Gps_state_fault = True
        msg_faultcode.can_state = True
        pub.publish(msg_faultcode) 

    # 添加清理代码，确保关闭 SocketCAN 总线
    # bus.shutdown()

if __name__ == '__main__':
    main()
