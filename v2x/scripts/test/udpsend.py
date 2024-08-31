import socket
import json
import rospy
from car_interfaces.msg import TrafficLightControlSend,  PerceptionShareSend, CooperativeLaneChangeSend  



# 创建一个字典来存储每个标签对应的发送频率
send_freqs = {
    "2001": 1,
    "1001": 1,  
    "6002": 1
}


udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest_addr = ('192.168.1.50', 8888)

def callback_2001(data):
    message = data.__dict__
    message["tag"] = 2001
    udp_socket.sendto(json.dumps(message).encode(), dest_addr)
    print("Message sent:", message)

def callback_1001(data):
    message = data.__dict__
    message["tag"] = 1001
    udp_socket.sendto(json.dumps(message).encode(), dest_addr)
    print("Message sent:", message)

def callback_6002(data):
    global feedback_count
    # 处理接收到的变道请求
    print("Received lane change request from device:", data.deviceNo)
    # 如果已经发送了3次反馈，就不再发送
    if feedback_count >= 3:
        print("Already sent 3 feedbacks. Ignoring request.")
        feedback_count = 0  # 重置计数器
    else:
        message = data.__dict__
        message["tag"] = 6002
        udp_socket.sendto(json.dumps(message).encode(), dest_addr)
        print("Message sent:", message)
        feedback_count += 1
    
def listener():
    rospy.init_node('udp_send_node', anonymous=True)
    rospy.Subscriber("topic_2001", TrafficLightControlSend, callback_2001)
    rospy.Subscriber("topic_1001", PerceptionShareSend, callback_1001)
    rospy.Subscriber("topic_6002",  CooperativeLaneChangeSend, callback_6002)
    rospy.spin()

if __name__ == '__main__':
    feedback_count = 0
    listener()
