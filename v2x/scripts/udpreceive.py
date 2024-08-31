import socket
import json
import time
import rospy
from std_msgs.msg import String
# from car_interfaces.msg import ObuHeartbeat, TrafficLightControlReceive, TrafficLightTiming, ObuEventMessage, PerceptionShareFeedback, CooperativeLaneChangeRequest, RoadMessage, PointMessage


# 创建发布者
obu_heartbeat_pub = rospy.Publisher('obu_heartbeat', String, queue_size=10)
traffic_light_control_pub = rospy.Publisher('traffic_light_control', String, queue_size=10)
traffic_light_timing_pub = rospy.Publisher('traffic_light_timing', String, queue_size=10)
obu_event_message_pub = rospy.Publisher('obu_event_message', String, queue_size=10)
perception_share_feedback_pub = rospy.Publisher('perception_share_feedback', String, queue_size=10)
lane_change_request_to_decision = rospy.Publisher('lane_change_request_to_decision', String, queue_size=10)

# 初始化ROS节点
rospy.init_node('data_publisher', anonymous=True)

rate = rospy.Rate(100)
# dest_addr = ('192.168.1.100', 6666)

def udp_receive():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_addr = ('192.168.1.102', 8888)
    dest_addr = ('192.168.1.102', 8888)
    udp_socket.bind(local_addr)
    
    while not rospy.is_shutdown():
        recv_data, addr = udp_socket.recvfrom(1024)
        message = recv_data.decode()
        data = json.loads(message)
        
        print("Received data:", data)
        # 可以添加适当的逻辑处理

        if data['tag'] == 2101:  # OBU心跳数据上报
            
            msg = json.dumps(data)
            obu_heartbeat_pub.publish(msg)
            # {"deviceNo":"test0001","tag":2101}
            rate.sleep()


        if data['tag'] == 2002: # 红绿灯控制上报
            
            msg = json.dumps(data)
            traffic_light_control_pub.publish(msg)
            # 收到控灯指令，未完成控灯：
            # {"detail":"does not heading a node!","status":1,"tag":2002} 
            # 收到控灯指令，已完成控灯：
            # {"detail":"received valid light control data","status":0,"tag":2002}
            rate.sleep()
            print("Received data:", data)


        if data['tag'] == 2104: # 红绿灯计时数据上报
            msg = json.dumps(data)
            traffic_light_timing_pub.publish(msg)
            # {"data":{"device_id":"R:255_L:5","dist":44.5443129376588,"ele":0.0,"hea":0.0,"lat":39.2003257,"local_id":5,"lon":117.4453693,"phase":[{"color":2,"direction":270,"guide_speed_max":41.0,"guide_speed_min":0.0,"id":1,"time":4},{"color":2,"direction":0,"guide_speed_max":41.0,"guide_speed_min":0.0,"id":1,"time":4}],"region_id":255},"tag":2104}
            # {"data":{"device_id":"R:255_L:5","dist":21.5771626095655,"ele":0.0,"hea":0.0,"lat":39.2003257,"local_id":5,"lon":117.4453693,"phase":[{"color":1,"direction":270,"guide_speed_max":80.0,"guide_speed_min":2.0,"id":1,"time":25},{"color":1,"direction":0,"guide_speed_max":80.0,"guide_speed_min":2.0,"id":1,"time":25}],"region_id":255},"tag":2104}
            rate.sleep()

                
        if data['tag'] == 10300097: # OBU发送事件消息
            msg = json.dumps(data)
            obu_event_message_pub.publish(msg)
            # {"deviceNo":"test0001","roadPoints":[{"elevation":0.0,"latitude":39.1964416,"longitude":117.4401555},{"elevation":0.0,"latitude":11.1111111,"longitude":111.1111111},{"elevation":0.0,"latitude":22.2222222,"longitude":222.2222222}],"tag":10300097,"timestamp":1717638413092}
            rate.sleep()


        if data['tag'] == 1002: # 感知共享反馈上报
            msg = json.dumps(data)
            perception_share_feedback_pub.publish(msg)
            # {"detail":"received valid sensor data","status":0,"tag":1002}
            rate.sleep()


        if data['tag'] == 6001:  # 协作式变道请求
            time_now = rospy.Time.now()
            timestamp = time_now

            msg = json.dumps(data)
            lane_change_request_to_decision.publish(msg)
            data_callback = {"deviceNo":"WIDC0025","tag":6002,"timestamp":timestamp,"status":1,"detail":"参赛车已同意变道"}
            udp_socket.sendto(json.dumps(data_callback).encode(), dest_addr)
            # receive{"deviceNo":"test0001","remoteLat":31.7816292,"remoteLon":117.3551324,"tag":6001,"timestamp":1646881026,"remoteSpeed":5.1,"remoteHeading":50.23}
            
            
        
    udp_socket.close()

if __name__ == '__main__':
    udp_receive()