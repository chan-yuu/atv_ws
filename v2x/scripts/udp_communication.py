#!/usr/bin/python3
# -*- coding: utf-8 -*-


import sys
import json
import time
import math
import rospy
import socket
import std_msgs.msg
from std_msgs.msg import String
import threading
from lib import color_print_lib

# 定义全局变量
udp_socket = None
ego_content = {}
red_green_point_lat = 0
red_green_point_lon = 0
dis_to_stopline = 0
is_write = False

is_send = False
stopline = 0
udp_socket = None

# 初始化UDP
def udpInit(IP, port):
    global udp_socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    localladdr = (IP, port)
    udp_socket.bind(localladdr) # 自己电脑的ip&port

# UDP接收数据
def udpReceive():
    global udp_socket, ego_content, red_green_point_lat, red_green_point_lon, dis_to_stopline, is_write

    while True:
        recv_data = udp_socket.recvfrom(1024)[0]
        recv_msg = json.loads(recv_data.decode('utf-8'))
        tag = recv_msg["tag"]
        print('recv_data:', recv_data)

        if tag == 2101:
            # 心跳数据上报
            device_no = recv_msg['deviceNo']
            print('Heartbeat from device:', device_no)

        elif tag == 2001:
            # 红绿灯控制下发
            device_no = recv_msg['deviceNo']
            intention = recv_msg['intention']
            print(f'Control light from device {device_no}, intention: {intention}')

        elif tag == 2002:
            # 红绿灯控制上报
            status = recv_msg.get('status', 1)
            print('Light control status:', status)

        elif tag == 2104:
            # 红绿灯计时数据上报
            recv_light_control = recv_msg['data']
            red_green_point_lat = recv_light_control['lat']
            red_green_point_lon = recv_light_control['lon']
            UTM = [ego_content['UTM_x'], ego_content['UTM_y']]
            dis_to_stopline = math.sqrt(red_gereen_point_straight([red_green_point_lat, red_green_point_lon], UTM) ** 2 - 59.29)
            print('2104light:', recv_light_control)
            print('dis_to_stopline:', dis_to_stopline)
            if recv_light_control and dis_to_stopline < 60:
                redline_json = {"deviceNo": "TJUID1", "intention": 1, "tag": 2001}
                redline_json_addr = ('192.168.195.102', 8070)
                udpSend(redline_json, redline_json_addr)

        elif tag == 10300097:
            # 事件信息上报
            Send_pub = rospy.Publisher('/I2V_send', String, queue_size=10)
            Send_Points = recv_msg['roadPoints']
            send_content = {'Send_Points': Send_Points}
            Send_pub.publish(json.dumps(send_content))

        elif tag == 1001:
            # 感知数据共享下发
            device_no = recv_msg['deviceNo']
            longitude = recv_msg['longitude']
            latitude = recv_msg['latitude']
            obstacle_type = recv_msg['obstacleType']
            timestamp = recv_msg['timestamp']
            print(f'Sensing data from device {device_no}: ({latitude}, {longitude}), type: {obstacle_type}, timestamp: {timestamp}')

        elif tag == 1002:
            # 感知数据共享反馈上报
            status = recv_msg.get('status', 1)
            print('Sensing data status:', status)

        elif tag == 6001:
            # 协作式变道请求上报
            device_no = recv_msg['deviceNo']
            remote_lon = recv_msg['remoteLon']
            remote_lat = recv_msg['remoteLat']
            remote_speed = recv_msg['remoteSpeed']
            remote_heading = recv_msg['remoteHeading']
            timestamp = recv_msg['timestamp']
            print(f'Lane change request from device {device_no}: ({remote_lat}, {remote_lon}), speed: {remote_speed}, heading: {remote_heading}, timestamp: {timestamp}')

        elif tag == 6002:
            # 协作式变道反馈下发
            device_no = recv_msg['deviceNo']
            status = recv_msg['status']
            timestamp = recv_msg['timestamp']
            print(f'Lane change feedback from device {device_no}, status: {status}, timestamp: {timestamp}')

# udpSend 函数
def udpSend(data_json, dest_addr):
    global udp_socket
    # 编码使用utf-8 还是 'GBK'?
    udp_socket.sendto(json.dumps(data_json).encode('utf-8'), dest_addr)

# ROS订阅处理
def call_back_lc_decision(msg):
    global udp_socket
    recv_msg = json.loads(msg.data)
    lat = 39.1769783830
    lon = 117.4654973042
    ticks = time.time()
    if lat != 0:
        data_json = {"deviceNo": "TJUID1", "elevation": 0, "latitude": lat, "longitude": lon, "tag": 1001, "timestamp": ticks}
        dest_addr = ('192.168.195.102', 8070)
        udpSend(data_json, dest_addr)

def call_back_lc_decision1(msg):
    global ego_content
    ego_content = json.loads(msg.data)

# 辅助函数
def red_gereen_point_straight(intersection_stopline, ego_content):
    coord4 = from_latlon(intersection_stopline[0], intersection_stopline[1])
    dis_intersection_stopline = math.sqrt(math.pow(coord4[0] - ego_content[0], 2) + math.pow(coord4[1] - ego_content[1], 2))
    return dis_intersection_stopline

def from_latlon(latitude, longitude, force_zone_number=None):
    K0 = 0.9996
    E = 0.00669438
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1.0 - E)
    SQRT_E = math.sqrt(1 - E)
    _E = (1 - SQRT_E) / (1 + SQRT_E)
    _E2 = _E * _E
    _E3 = _E2 * _E
    _E4 = _E3 * _E
    _E5 = _E4 * _E
    M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
    M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
    M3 = (15 * E2 / 256 + 45 * E3 / 1024)
    M4 = (35 * E3 / 3072)
    P2 = (3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5)
    P3 = (21. / 16 * _E2 - 55. / 32 * _E4)
    P4 = (151. / 96 * _E3 - 417. / 128 * _E5)
    P5 = (1097. / 512 * _E4)
    R = 6378137
    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    if not -80.0 <= latitude <= 84.0:
        raise OutOfRangeError('latitude out of range (must be between 80 deg S and 84 deg N)')
    if not -180.0 <= longitude <= 180.0:
        raise OutOfRangeError('longitude out of range (must be between 180 deg W and 180 deg E)')
    lat_rad = math.radians(latitude)
    lat_sin = math.sin(lat_rad)
    lat_cos = math.cos(lat_rad)
    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2
    if force_zone_number is None:
        zone_number = latlon_to_zone_number(latitude, longitude)
    else:
        zone_number = force_zone_number
    zone_letter = latitude_to_zone_letter(latitude)
    lon_rad = math.radians(longitude)
    central_lon = zone_number_to_central_longitude(zone_number)
    central_lon_rad = math.radians(central_lon)
    n = R / math.sqrt(1 - E * lat_sin ** 2)
    c = E_P2 * lat_cos ** 2
    a = lat_cos * (lon_rad - central_lon_rad)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a
    m = R * (M1 * lat_rad - M2 * math.sin(2 * lat_rad) + M3 * math.sin(4 * lat_rad) - M4 * math.sin(6 * lat_rad))
    easting = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000
    northing = K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) + a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))
    if latitude < 0:
        northing += 10000000
    return easting, northing, zone_number, zone_letter

def latitude_to_zone_letter(latitude):
    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    if -80 <= latitude <= 84:
        return ZONE_LETTERS[int(latitude + 80) >> 3]
    else:
        return None

def latlon_to_zone_number(latitude, longitude):
    if 56 <= latitude < 64 and 3 <= longitude < 12:
        return 32
    if 72 <= latitude <= 84 and longitude >= 0:
        if longitude <= 9:
            return 31
        elif longitude <= 21:
            return 33
        elif longitude <= 33:
            return 35
        elif longitude <= 42:
            return 37
    return int((longitude + 180) / 6) + 1

def zone_number_to_central_longitude(zone_number):
    return (zone_number - 1) * 6 - 180 + 3

class OutOfRangeError(ValueError):
    pass

def main():
    rospy.init_node('WIDC', anonymous=False)
    udpInit("192.168.195.101", 8070)
    rospy.Subscriber('/cone_pub_WIDC', std_msgs.msg.String, call_back_lc_decision)
    rospy.Subscriber('/ztbus/location', std_msgs.msg.String, call_back_lc_decision1)
    # 障碍物识别回调。（主要是cone 小孩 等）
    rospy.Subscriber('/cone_pub_WIDC', std_msgs.msg.String, call_back_lc_decision)
    rospy.Subscriber('/gps_imu', GpsImuInterface, call_back_lc_decision1)
    
    udp_thread = threading.Thread(target=udpReceive)
    udp_thread.setDaemon(True)
    udp_thread.start()
    rospy.spin()

if __name__ == '__main__':
    main()

