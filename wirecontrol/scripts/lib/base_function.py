#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import serial
import rospy
import std_msgs.msg
from car_interfaces.msg import GpsImuInterface, GpsImuAddInterface
import math
from datetime import datetime, timedelta
import time
import pyproj
import signal

buf_pos_lat = 0
buf_pos_lon = 0
buf_vel = 0
buf_angle_heading = 0
# 库函数和帮助函数的导入 ...



import numpy as np
def from_latlon(latitude, longitude, force_zone_number=None, force_zone_letter=None):
    """This function converts Latitude and Longitude to UTM coordinate
        Parameters
        ----------
        latitude: float or NumPy array
            Latitude between 80 deg S and 84 deg N, e.g. (-80.0 to 84.0)
        longitude: float or NumPy array
            Longitude between 180 deg W and 180 deg E, e.g. (-180.0 to 180.0).
        force_zone_number: int
            Zone number is represented by global map numbers of an UTM zone
            numbers map. You may force conversion to be included within one
            UTM zone number.  For more information see utmzones [1]_
        force_zone_letter: str
            You may force conversion to be included within one UTM zone
            letter.  For more information see utmzones [1]_
        Returns
        -------
        easting: float or NumPy array
            Easting value of UTM coordinates
        northing: float or NumPy array
            Northing value of UTM coordinates
        zone_number: int
            Zone number is represented by global map numbers of a UTM zone
            numbers map. More information see utmzones [1]_
        zone_letter: str
            Zone letter is represented by a string value. UTM zone designators
            can be accessed in [1]_
       .. _[1]: http://www.jaworski.ca/utmzones.htm
    """
    
    def latlon_to_zone_number(latitude, longitude):
        # If the input is a numpy array, just use the first element
        # User responsibility to make sure that all points are in one zone
        if isinstance(latitude, np.ndarray):
            latitude = latitude.flat[0]
        if isinstance(longitude, np.ndarray):
            longitude = longitude.flat[0]
        if 56 <= latitude < 64 and 3 <= longitude < 12:
            return 32
        if 72 <= latitude <= 84 and longitude >= 0:
            if longitude < 9:
                return 31
            elif longitude < 21:
                return 33
            elif longitude < 33:
                return 35
            elif longitude < 42:
                return 37
        return int((longitude + 180) / 6) + 1
    
    def latitude_to_zone_letter(latitude):
        if -80 <= latitude <= 84:
            return ZONE_LETTERS[int(latitude / 8) + 10]
        return "Z"
    
    def zone_number_to_central_longitude(zone_number):
        return zone_number * 6 - 183
    
    def mod_angle(value):
        """Returns angle in radians to be between -pi and pi"""
        return (value + np.pi) % (2 * np.pi) - np.pi
    def mixed_signs(x):
        return np.min(x) < 0 and np.max(x) >= 0
    def negative(x):
        return np.max(x) < 0
    
    K0 = 0.9996
 
    E = 0.00669438
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1 - E)
 
    SQRT_E = np.sqrt(1 - E)
    _E = (1 - SQRT_E) / (1 + SQRT_E)
    _E2 = _E * _E
    _E3 = _E2 * _E
    _E4 = _E3 * _E
    _E5 = _E4 * _E
 
    M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
    M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
    M3 = (15 * E2 / 256 + 45 * E3 / 1024)
    M4 = (35 * E3 / 3072)
 
    P2 = (3 / 2 * _E - 27 / 32 * _E3 + 269 / 512 * _E5)
    P3 = (21 / 16 * _E2 - 55 / 32 * _E4)
    P4 = (151 / 96 * _E3 - 417 / 128 * _E5)
    P5 = (1097 / 512 * _E4)
 
    R = 6378137
 
    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    
    lat_rad = np.radians(latitude)
    lat_sin = np.sin(lat_rad)
    lat_cos = np.cos(lat_rad)
 
    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2
 
    if force_zone_number is None:
        zone_number = latlon_to_zone_number(latitude, longitude)
    else:
        zone_number = force_zone_number
 
    if force_zone_letter is None:
        zone_letter = latitude_to_zone_letter(latitude)
    else:
        zone_letter = force_zone_letter
 
    lon_rad = np.radians(longitude)
    central_lon = zone_number_to_central_longitude(zone_number)
    central_lon_rad = np.radians(central_lon)
 
    n = R / np.sqrt(1 - E * lat_sin**2)
    c = E_P2 * lat_cos**2
 
    a = lat_cos * mod_angle(lon_rad - central_lon_rad)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a
 
    m = R * (M1 * lat_rad -
             M2 * np.sin(2 * lat_rad) +
             M3 * np.sin(4 * lat_rad) -
             M4 * np.sin(6 * lat_rad))
 
    easting = K0 * n * (a +
                        a3 / 6 * (1 - lat_tan2 + c) +
                        a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000
 
    northing = K0 * (m + n * lat_tan * (a2 / 2 +
                                        a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c**2) +
                                        a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))
 
    if mixed_signs(latitude):
        raise ValueError("latitudes must all have the same sign")
    elif negative(latitude):
        northing += 10000000
 
    return easting, northing, zone_number, zone_letter
    

# 0-360 ->  -pi - pi
def angle_2_angle(angle):
    angle -= 90
    while (angle < -180):
        angle = angle + 360
    while (angle > 180):
        angle = angle - 360
    return -angle
    

# def connect_to_serial(port_name, baud_rate):
#     try:
#         return serial.Serial(port_name, baudrate=baud_rate, timeout=1)
#     except serial.SerialException as e:
#         rospy.logerr(f"Unable to connect to {port_name} with baud rate {baud_rate}: {e}")
#         return None

# def read_from_port(serial_port):
#     global buf_pos_lat
#     global buf_pos_lon
#     global buf_vel
#     global buf_angle_heading
    
#     content = {}
#     # 此处放置读取串口的逻辑 ...
#     buf_new = serial_port.readline()
#     #print("buf new",buf_new)
    
#     if ("INSPVAXA" in buf_new.decode() and "FINESTEERING" in buf_new.decode()):
            
#         buf_whole_array = buf_new.decode().split(";")
#         buf_front_arary = buf_whole_array[0].split(",")
#         # print(buf_whole_array)
#         buf_back_array = buf_whole_array[1].split(",")


#         buf_pos_lon = float(buf_back_array[3])
#         buf_pos_lat = float(buf_back_array[2])


#         # buf_pos_alt = float(buf_back_array[4])

#         buf_vel_east= float(buf_back_array[7])
#         buf_vel_north= float(buf_back_array[6])
#         buf_vel_u= float(buf_back_array[8])

#         # 车速通过串口拿不到数据
#         buf_vel= math.sqrt(buf_vel_east**2+buf_vel_north**2+buf_vel_u**2)

#         buf_angle_heading = angle_2_angle(float(buf_back_array[11]))
#         # buf_angle_pitch = float(buf_back_array[10])
#         # buf_angle_roll = float(buf_back_array[9])

#         if(buf_back_array[0] == "INS_SOLUTION_GOOD"):
#             buf_system_state = 3
#         else:
#             buf_system_state = 0

#         # 搜星数量
#         buf_gps_num_sats_used = 0
#         buf_gps_num_sats_1 = 0
#         buf_gps_num_sats_2 = 0

#         if buf_front_arary[4] == "FINEBACKUPSTEERING":
#             buf_satellite_status = 1
#         else:
#             buf_satellite_status = 0

#         # 差分延时
#         buf_gps_age = 0
#         buf_pos_x = 0
#         buf_pos_y = 0

#         # 进程处理时间
#         buf_process_time=0
#         # 惯导通信故障
#         buf_gps_cfault = False

#         content["Lat"] = buf_pos_lat # GPS_MSG.PosLat
#         content["Lon"] = buf_pos_lon # GPS_MSG.PosLon
#         content["Head"] = buf_angle_heading # GPS_MSG.AngleHeading
#         content["Speed"] = buf_vel # GPS_MSG.Vel
#         content["UTM_x"] = from_latlon(buf_pos_lat, buf_pos_lon)[0]
#         content["UTM_y"] = from_latlon(buf_pos_lat, buf_pos_lon)[1]

#         # print(content["UTM_x"])

#         # msg.PosLon =1
#         # # msg.PosLan =1
#         # msg.VelE =2

#         # # msg.posX =  388652.2938084109
#         # # msg.posY =  4963429.720356053
#         # msg.posX =  from_latlon(buf_pos_lat, buf_pos_lon)[0]
#         # msg.posY =  from_latlon(buf_pos_lat, buf_pos_lon)[1]
#         # msg.Vel = buf_vel
#         # msg.AngleHeading = buf_angle_heading
#         # msg.GpsNumSatsUsed = 20

#         return content




# ANSI颜色代码
class ANSI:
    RESET = '\033[0m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'



def prRed(skk): 
    print("\033[91m {}\033[00m".format(skk))


def prGreen(skk): 
    print("\033[92m {}\033[00m".format(skk))


def prYellow(skk): 
    print("\033[93m {}\033[00m".format(skk))


def prBlue(skk): 
    print("\033[94m {}\033[00m".format(skk))


def prPurple(skk): 
    print("\033[95m {}\033[00m".format(skk))


def prCyan(skk): 
    print("\033[96m {}\033[00m".format(skk))


def prOrange(skk): 
    print("\033[33m {}\033[00m".format(skk))


def prPink(skk): 
    print("\033[95m {}\033[00m".format(skk))



class Road:
    def __init__(self):
        self.id = -1
        self.lane = []
        pass

#lane id, attr and pos info
class Lane:
    def __init__(self):
        self.id = -1
        self.left = -1
        self.right = -1
        self.spd = -1
        self.width = -1
        self.points = []
        pass

#lat, lon and head
class Pos:
    def __init__(self, x, y, head):
        self.x = x
        self.y = y
        self.head = head
        pass

class Point:
    def __init__(self, x, y, head):
        self.x = x
        self.y = y
        self.head = head
        pass


Map = []
def readMap(mapfile):
    global Map
    vel_values = []
    with open(mapfile) as m:
        rows = m.readlines()
        for i in range(1, len(rows)):
            road = Road()
            lane = Lane()
            col = rows[i].split('\t')
            road.id = int(col[0])
            lane.id = int(col[1])
            lane.left = int(col[2])
            lane.right = int(col[3])
            lane.spd = int(col[4])  #之前就有speed存在的
            for pos in col[5:]:
                seg = pos.split(',')
                # print("aaaa")
                if len(seg) > 2:
                    seg_new = []
                    vel_values.append(float(seg[5]))
                    seg_new = from_latlon(float(seg[1]),float(seg[0]))
                    lane.points.append(Pos(float(seg_new[0]), float(seg_new[1]), float(seg[2])))
            road.lane.append(lane)
            Map.append(road)
        for pos in lane.points:
            # print(f"Pos: x={pos.x}, y={pos.y}, head={pos.head}")
            pass
    # print (Map[0].lane[0].points)
    # print('mapfile read success!!!!')
    print(f'{ANSI.GREEN}mapfile read success!!!!{ANSI.RESET}')

    x_values = [pos.x for pos in lane.points]
    y_values = [pos.y for pos in lane.points]
    head_values = [pos.head for pos in lane.points]

    return x_values,y_values,head_values,vel_values

def color_ros():
    rospy.logdebug('This is a debug message')
    rospy.loginfo('This is an info message')
    rospy.logwarn('This is a warning message')
    rospy.logerr('This is an error message')
    rospy.logfatal('This is a fatal message')