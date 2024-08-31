import socket
import json
import rospy, time
# from car_interfaces.msg import TrafficLightControlSend,  PerceptionShareSend, CooperativeLaneChangeSend  
from std_msgs.msg import String
import math
import pyproj
import time
from car_interfaces.msg import  GpsImuInterface, CameraInterface


udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest_addr = ('192.168.1.100', 6666)

def callback_TrafficLightControlSend(msg):
    data = json.loads(msg.data)
    # print(data)
    if data["data"]["dist"] <= 45:
        send_data = {"deviceNo":"test0001","intention":1,"tag":2001}
        udp_socket.sendto(json.dumps(send_data).encode(), dest_addr)
        print(data["data"]["dist"])
        # print("Message sent:", data)
        time.sleep(1)

def convert_from_utm(utm_x, utm_y, utm_zone, northern_hemisphere=True):
    utm_epsg_code = 32600 + utm_zone if northern_hemisphere else 32700 + utm_zone
    utm = pyproj.CRS(f'EPSG:{utm_epsg_code}')
    wgs84 = pyproj.CRS('EPSG:4326')  # WGS84
    transformer = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
    longitude, latitude = transformer.transform(utm_x, utm_y)
    return latitude, longitude

def call_back_ego_vehicle_data(msg):
        global ego_content
        try:
            ego_content['Lat'] = msg.lat
            ego_content['Lon'] = msg.lon
        except:
            print('content error')

def callback_Camera(msg):
    type = msg.type
    global obstacleType 
    obstacleType = type

def callback_PerceptionShareSend(msg):
    global feedback_count
    global obstacleType
    
    # OBU接受感知信息下发
    data = json.loads(msg.data)
    print("Received lane change request from device:", data["deviceNo"])
    # 如果感知共享信息发送了三次，OBU就不再进行处理
    if feedback_count >= 3:
        print("Already sent 3 feedbacks. Ignoring request.")
        feedback_count = 0  # 重置计数器
    else:
        # 计算UTM区域
        utm_zone = (math.floor((ego_content['Lon'] + 180) / 6) % 60) + 1

        # 转换UTM坐标为经纬度
        latitude, longitude = convert_from_utm(data['UTM_x'], data['UTM_y'], utm_zone)
        time_now = rospy.Time.now()
        timestamp = time_now
        # 更新数据字典
        data.update({
           "deviceNo": "WIDC0025",
            "elevation": 0,
            "latitude": latitude,
            "longitude": longitude,
            "obstacleType": obstacleType,
            "timestamp": timestamp
        })
        # {"deviceNo":"test0001","elevation":0,"latitude":31.7816292,"longitude":117.3551324,"tag":1001, "obstacleType":1, "timestamp":1646881026}

        udp_socket.sendto(json.dumps(data).encode(), dest_addr)
        # print("Message sent:", data)
        time.sleep(1)
        feedback_count += 1


    
    
    
def listener():
    rospy.init_node('udp_send_node', anonymous=True)
    rospy.Subscriber("topic_2001", String, callback_TrafficLightControlSend)
    rospy.Subscriber("topic_1001", String, callback_PerceptionShareSend)
    rospy.Subscriber("traffic_light_timing", String, callback_TrafficLightControlSend)
    rospy.Subscriber('/gps_imu', GpsImuInterface, call_back_ego_vehicle_data)
    rospy.Subscriber('/camera', CameraInterface, callback_Camera)
    rospy.spin()

if __name__ == '__main__':
    feedback_count = 0
    listener()
