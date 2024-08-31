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

target_vel = 0#17  #车辆速度设置
target_steer_vel = 0  # 车辆方向盘转角速度


# safe_detect = {"steering_state":0,"front_door_state":0,"middle_door_state":0}    # 用于记录安全检查的相关情况
# is_exist = {"0C05A5D0": False, "0C06A5D0": False, "0C07A5D0": False, "0C09A5D0": False}       # 用于判断是否收到相应报文
# allow_auto = False # 用于自检最终判定是否可以进行自动驾驶


messages_1 = [      # 发送给整车VCU的报文类型
    {
        'message_id': 0x0C01D0A5,    # 车辆方向盘转向信息报文
        'message_name': 'steering_control',
        'signals': {
            'DCU_valid': 1,                 # VCU状态，1为正常，0为不正常，1 bit表示
            'steering_control_valid': 1,    # 车辆转角状态，1为正常，0不正常，1 bit表示
            'target_steering_torque': 0,    # 转向叠加扭矩信号，目前可以先取128中间值，一字节表示
            'steering_msg_life': 0,         # 该消息发送的生命周期
        }
    },
    {
        'message_id': 0x0C02D0A5,   # 车辆qudong信息报文
        'message_name': 'drive_control',
        'signals': {
            'target_direction': 1,
            'fault_code': 0,
            'drive_msg_life': 0,
        }
    },
    {
        'message_id': 0x0C03D0A5,  # 车辆zhidong
        # 信息报文
        'message_name': 'barke_control',
        'signals': {
            'XBR2_EBI_Mode': 0,
            'XBR2_Priority': 0,
            'XBR2_Ctrl_Mode': 0,
            'XBR2_message_counter': 0,
            'XBR2_checksum': 0,
        }
    },
    {
        'message_id': 0x0C04D0A5,  # 车辆整车速度信息报文
        'message_name': 'body_control',
        'signals': {
            'mode_disp': 0,
            'body_state': 1,
            'turning_lighting_control': 0,
            'high_low_beam_control': 2,
            'hazard_lights_control': 0,
            'backup_light_control': 0,
            'width_lamp_control': 0,
            'wiper_wash_switch' : 0,
            'front_door_control': 0,
            'middle_door_control': 0,
            'horn_control' : 0,
        }
    },
    {
        'message_id': 0x0C08D0A5,  # 车辆整车速度信息报文
        'message_name': 'parking_control',
        'signals': {
            'park_msg_checksum': 10,
        }
    },
]

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


# def msg_send_to_vcu(messages_1,dbc,bus):
#     #置发送了message
#     global count
#     global brake_flag
#     global bus_vel
#     # 以下向车辆VCU发送了方向盘转角规划信息
#     global flag_steer
#     global Brake
#     global q_num
#     global quit_mod
#     global steer_mod
#     global count1

#     msg_send = dbc.get_message_by_name(messages_1[0]['message_name'])
#     messages_ = list(msg_send.encode(messages_1[0]['signals']))
#     messages_send = can.Message(arbitration_id=msg_send.frame_id, data=messages_, is_extended_id=True)

#     msg_send11 = dbc.get_message_by_name(messages_1[1]['message_name'])
#     messages_11 = list(msg_send11.encode(messages_1[1]['signals']))
#     messages_send11 = can.Message(arbitration_id=msg_send11.frame_id, data=messages_11, is_extended_id=True)

#     msg_send33 = dbc.get_message_by_name(messages_1[2]['message_name'])
#     messages_33 = list(msg_send33.encode(messages_1[2]['signals']))
#     messages_send33 = can.Message(arbitration_id=msg_send33.frame_id, data=messages_33, is_extended_id=True)

#     msg_send44 = dbc.get_message_by_name(messages_1[3]['message_name'])
#     messages_44 = list(msg_send44.encode(messages_1[3]['signals']))
#     messages_send44 = can.Message(arbitration_id=msg_send44.frame_id, data=messages_44, is_extended_id=True)

#     msg_send88 = dbc.get_message_by_name(messages_1[4]['message_name'])
#     messages_88 = list(msg_send88.encode(messages_1[4]['signals']))
#     messages_send88 = can.Message(arbitration_id=msg_send88.frame_id, data=messages_88, is_extended_id=True)
#     bus.send(messages_send)
#     bus.send(messages_send11)
#     # bus.send(messages_send33)
#     bus.send(messages_send44)
#     # bus.send(messages_send88)


def msg_recv_from_vcu(dbc,bus):
    global can_state #can连接正常
    global safe_detect#自动驾驶自检
    global messages_1

    try:
        messages_recv = bus.recv()
        # if messages_recv.arbitration_id == 0x0C06A5D0:
        #     decoded_messages = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
            
        # else:
        #     pass
        if messages_recv.arbitration_id == 0x0C05A5D0:
            # is_exist["0C05A5D0"] = True
            decoded_messages_05 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
            # msg_car_ori.steerangle = decoded_messages_05['act_steering_angle']  #   当前角度
            safe_detect["steering_state"] = decoded_messages_05['act_steering_state']  # 1为自动驾驶，4为手动模式，5为人工介入模式，6为警告模式，7为错误，8为力矩叠加
            
            can_state = True ##有can信号就true

        if messages_recv.arbitration_id == 0x0C06A5D0:
            # is_exist["0C06A5D0"] = True
            decoded_messages_06 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
            safe_detect["EBS_state"] = decoded_messages_06['EBS_state']  # 检测为1是故障，1为正常
            safe_detect["brake_state"] = decoded_messages_06["act_brake_pedal"]  # 检测为1有刹车，0为无刹车（需要确认是手刹还是脚刹）
        
        if messages_recv.arbitration_id == 0x0C07A5D0:
            # is_exist["0C07A5D0"] = True
            decoded_messages_07 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)
            safe_detect["front_door_state"] = decoded_messages_07["act_front_door_state"]  # 前门检测为开置1，关为0
            k_light_f = safe_detect["front_door_state"]
            # print(f"当前front反馈为{k_light_f}")
            safe_detect["middle_door_state"] = decoded_messages_07["act_middle_door_state"]  # 中门检测为开置1，关为0
            k_light_m = safe_detect["middle_door_state"]
            # print(f"当前middle反馈为{k_light_m}")

        if messages_recv.arbitration_id == 0x0C09A5D0:
            # is_exist["0C09A5D0"] = True
            decoded_messages_09 = dbc.decode_message(messages_recv.arbitration_id, messages_recv.data)

        #是否只需要生命周期or XBR2_EBI_Mode ？如果是，直接用
        # drive_life = messages_1[1]['signals']['drive_msg_life']
        # drive_life += 1
        # if drive_life > 255:
        #     drive_life -= 256
        # messages_1[1]['signals']['drive_msg_life'] = drive_life
        # park_life = messages_1[4]['signals']['park_msg_checksum']
        # park_life += 1
        # if park_life > 255:
        #     park_life -= 256
        # messages_1[4]['signals']['park_msg_checksum'] = park_life

        # messages_1[2]['signals']['XBR2_Ctrl_Mode'] = 0
        # messages_1[2]['signals']['XBR2_EBI_Mode'] = 0
        # messages_1[2]['signals']['XBR2_Priority'] = 0

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
    #lidar开始发送数据，说明已启动
    global lidar_start
    try:
        content = json.loads(msg.data)
        lidar_start = content["lidar_communication_fault"] #1
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
    
    global safe_detect
    # global is_exist
    global allow_auto


    signal.signal(signal.SIGINT, signal_handler)
    script_directory = os.path.dirname(os.path.abspath(__file__))
    relative_path = "../config/car_vcu2.dbc"
    gps_dbc_file = os.path.join(script_directory, relative_path)

    try:
        dbc = cantools.db.load_file(gps_dbc_file)
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except:
        print("连接故障")
    

    rospy.init_node('fault_diagnosis')
    pub = rospy.Publisher('fault_diagnosis_data', FaultDiagnosisInterface, queue_size=10)
    # rospy.Subscriber('/ztbus/location', std_msgs.msg.String, call_back_CurGNSS)
    # rospy.Subscriber('objectlist_json',std_msgs.msg.String,call_back_lidarState)
    # self_check_pub = rospy.Publisher('/from_vcu_self_check', std_msgs.msg.String, queue_size=10)
    msg_faultcode = FaultDiagnosisInterface()
    prGreen("start success")

    
    while not is_exit:

        # msg_recv_from_vcu(dbc,bus)
        # print(safe_detect)
        # msg_send_to_vcu(messages_1, dbc, bus)

        # if len(safe_detect)!=0:
        #     if safe_detect["steering_state"] in range(5,8):
        #         print("驾驶员手动接入！！！自动驾驶解除")
        #         allow_auto = False
        #     elif safe_detect["front_door_state"] == 1:
        #         print("前门车门未关！！！不可以进行自动驾驶")
        #         allow_auto = False
        #     elif safe_detect["middle_door_state"] == 1:
        #         print("中间车门未关！！！不可以进行自动驾驶")
        #         allow_auto = False

        #     # elif safe_detect["EBS_state"] == 1:
        #     #     print("车辆EBS系统发生故障！！！不可以进行自动驾驶")
        #     #     allow_auto = False          # 如果不满足条件，则不可以运行自动驾驶
            
        #     else:
        #         print("允许自动驾驶")  # 否则可以运行条件
        #         allow_auto = True  
        #         body_state = {"allow_auto":allow_auto, "detect_state":safe_detect}
        #         # self_check_pub.publish(json.dumps(body_state))

        msg_faultcode.lidar_start = 1
        msg_faultcode.Gps_state_fault = 1
        msg_faultcode.can_state = 1
        pub.publish(msg_faultcode)

        time.sleep(0.01)

    # 添加清理代码，确保关闭 SocketCAN 总线
    bus.shutdown()

if __name__ == '__main__':
    main()
