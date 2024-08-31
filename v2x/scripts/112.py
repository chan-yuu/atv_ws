#!/usr/bin/python3
# -*- coding: utf-8 -*-

import socket
import json
import rospy
import time
from std_msgs.msg import String
import math
import pyproj
from car_interfaces.msg import GpsImuInterface, CameraInterface

# Global variables
feedback_count = 0
obstacleType = None
ego_content = {}
camera_flag = False

# Coordinate conversion function
def convert_from_utm(utm_x, utm_y, utm_zone, northern_hemisphere=True):
    utm_epsg_code = 32600 + utm_zone if northern_hemisphere else 32700 + utm_zone
    utm = pyproj.CRS(f'EPSG:{utm_epsg_code}')
    wgs84 = pyproj.CRS('EPSG:4326')
    transformer = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
    longitude, latitude = transformer.transform(utm_x, utm_y)
    return latitude, longitude

# Callback functions
def call_back_ego_vehicle_data(msg):
    global ego_content
    try:
        ego_content['Lat'] = msg.lat
        ego_content['Lon'] = msg.lon
    except Exception as e:
        print(f'Error in ego vehicle data callback: {e}')

def callback_Camera(msg):
    global obstacleType
    global camera_flag
    obstacleType = msg.type
    camera_flag = True

def callback_TrafficLightControlSend(msg):
    data = json.loads(msg.data)
    if data["data"]["dist"] <= 45:
        send_data = {"deviceNo":"WIDC0025","intention":1,"tag":2001}
        udp_socket.sendto(json.dumps(send_data).encode(), dest_addr)
        print(data["data"]["dist"])
        time.sleep(1)

def callback_PerceptionShareSend(msg):
    global obstacleType
    global ego_content
    
    data = json.loads(msg.data)
    utm_zone = (math.floor((ego_content['Lon'] + 180) / 6) % 60) + 1
    latitude, longitude = convert_from_utm(data['UTM_x'], data['UTM_y'], utm_zone)
    timestamp = rospy.Time.now()
    
    if camera_flag:
        data_perception = {
            "deviceNo": "WIDC0025",
            "elevation": 0,
            "latitude": latitude,
            "longitude": longitude,
            "obstacleType": 2,
            "timestamp": timestamp
        }
        udp_socket.sendto(json.dumps(data_perception).encode(), dest_addr)
        print(data_perception, "data_perception")

# UDP receive function
def udp_receive():
    
    while not rospy.is_shutdown():
        recv_data, addr = udp_socket.recvfrom(1024)
        message = recv_data.decode()
        data = json.loads(message)
        
        print("Received data:", data)

        if data['tag'] == 2101:
            msg = json.dumps(data)
            obu_heartbeat_pub.publish(msg)

        if data['tag'] == 2002:
            msg = json.dumps(data)
            traffic_light_control_pub.publish(msg)

        if data['tag'] == 2104:
            if data["data"]["dist"] <= 45:
                send_data = {"deviceNo":"WIDC0025","intention":1,"tag":2001}
                udp_socket.sendto(json.dumps(send_data).encode(), dest_addr)
                print(data["data"]["dist"])

        if data['tag'] == 10300097:
            msg = json.dumps(data)
            obu_event_message_pub.publish(msg)

        if data['tag'] == 1002:
            msg = json.dumps(data)
            perception_share_feedback_pub.publish(msg)
            
        if data['tag'] == 6001:
            time_now = rospy.Time.now()
            timestamp = time_now
            msg = json.dumps(data)
            lane_change_request_to_decision.publish(msg)
            data_callback = {
                "deviceNo":"WIDC0025",
                "tag":6002,
                "timestamp":timestamp,
                "status":1,
                "detail":"参赛车已同意变道"
            }
            udp_socket.sendto(json.dumps(data_callback).encode(), dest_addr)
        rate.sleep()
        
    udp_socket.close()

# Main execution
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('data_publisher', anonymous=True)
    rate = rospy.Rate(10)

    # Publishers
    obu_heartbeat_pub = rospy.Publisher('obu_heartbeat', String, queue_size=10)
    traffic_light_control_pub = rospy.Publisher('traffic_light_control', String, queue_size=10)
    traffic_light_timing_pub = rospy.Publisher('traffic_light_timing', String, queue_size=10)
    obu_event_message_pub = rospy.Publisher('obu_event_message', String, queue_size=10)
    perception_share_feedback_pub = rospy.Publisher('perception_share_feedback', String, queue_size=10)
    lane_change_request_to_decision = rospy.Publisher('lane_change_request_to_decision', String, queue_size=10)

    # Subscribers
    rospy.Subscriber("topic_1001", String, callback_PerceptionShareSend)
    rospy.Subscriber('/gps_imu', GpsImuInterface, call_back_ego_vehicle_data)
    rospy.Subscriber('/camera', CameraInterface, callback_Camera)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_addr = ('192.168.1.102', 8888)
    dest_addr = ('192.168.1.100', 6666)
    udp_socket.bind(local_addr)

    # Start UDP receive loop
    udp_receive()
