import socket
import json
import time
import rospy
from std_msgs.msg import String
from car_interfaces.msg import ObuHeartbeat, TrafficLightControlReceive, TrafficLightTiming, ObuEventMessage, PerceptionShareFeedback, CooperativeLaneChangeRequest, RoadMessage, PointMessage


# 创建发布者
obu_heartbeat_pub = rospy.Publisher('obu_heartbeat', String, queue_size=10)
traffic_light_control_pub = rospy.Publisher('traffic_light_control', String, queue_size=10)
traffic_light_timing_pub = rospy.Publisher('traffic_light_timing', String, queue_size=10)
obu_event_message_pub = rospy.Publisher('obu_event_message', String, queue_size=10)
perception_share_feedback_pub = rospy.Publisher('perception_share_feedback', String, queue_size=10)
cooperative_lane_change_request_pub = rospy.Publisher('cooperative_lane_change_request', String, queue_size=10)

# 初始化ROS节点
rospy.init_node('data_publisher', anonymous=True)


# 创建一个字典来存储每个标签对应的发送频率
send_freqs = {
    "2101": 1,
    "2002": 1,  # 收到红绿灯控制下发消息，并执行后
    "2104": 1/3,  # 最少每秒 3 次
    "10300097": 1/3,  # 最少每秒 3 次
    "1002": 0,  # 无要求
    "6001": 1/10  # 10Hz
}

def udp_receive():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_addr = ('192.168.1.50', 8888)
    udp_socket.bind(local_addr)
    
    last_send_times = {key: 0 for key in send_freqs.keys()}
    
    while True:
        recv_data, addr = udp_socket.recvfrom(1024)
        message = recv_data.decode()
        data = json.loads(message)
        # 将数据转换为JSON格式的字符串
        data_str = json.dumps(data)
        # 创建一个String消息
        msg = String()
        msg.data = data_str
        # 根据数据的类型，发布到不同的主题
        if data['type'] == 'obu_heartbeat':
            obu_heartbeat_pub.publish(msg)
        elif data['type'] == 'traffic_light_control':
            traffic_light_control_pub.publish(msg)
        elif data['type'] == 'traffic_light_timing':
            traffic_light_timing_pub.publish(msg)
        elif data['type'] == 'obu_event_message':
            obu_event_message_pub.publish(msg)
        elif data['type'] == 'perception_share_feedback':
            perception_share_feedback_pub.publish(msg)
        elif data['type'] == 'cooperative_lane_change_request':
            cooperative_lane_change_request_pub.publish(msg)
        print("Received data:", data)
        # 可以添加适当的逻辑处理

        if data['tag'] == 2101:  # OBU心跳数据上报
            deviceNo = data['deviceNo'] # OBU的ID
            msg = ObuHeartbeat()
            msg.deviceNo = deviceNo
            obu_heartbeat_pub.publish(msg)
            print(json.dumps({"deviceNo": deviceNo, "tag": 2101}))


        if data['tag'] == 2002: # 红绿灯控制上报
            detail = data['detail']
            status = data['status']
            if status == 0:
                detail = "received valid light control data"
            else:
                detail = "does not heading a node!"
        msg = TrafficLightControlReceive()
        msg.detail = detail
        msg.status = status
        traffic_light_control_pub.publish(msg)
        print(json.dumps({"detail": detail, "status": status, "tag": 2002}))


        if data['tag'] == 2104: # 红绿灯计时数据上报
            device_id = data['data']['device_id'] # RSU设备ID
            lon = data['data']['lon'] # 红绿灯所在路口中心点的经度
            lat = data['data']['lat'] # 红绿灯所在路口中心点的纬度
            ele = data['data']['ele'] # 保留字段
            hea = data['data']['hea'] # 保留字段
            region_id = data['data']['region_id'] # 保留字段
            local_id = data['data']['local_id'] # 保留字段
            dist = data['data']['dist'] # 保留字段
            phases = data['data']['phase']

            msg = TrafficLightTiming()
            msg.device_id = device_id
            msg.lon = lon
            msg.lat = lat
            msg.ele = ele
            msg.hea = hea
            msg.region_id = region_id
            msg.local_id = local_id
            msg.dist = dist 
            print(json.dumps({"device_id": device_id, "lon": lon, "lat": lat, "ele": ele, "hea": hea, "region_id": region_id, "local_id": local_id, "dist": dist}))
            
            for phase in phases:
                id = phase['id'] # 相位ID
                color = phase['color'] # 红绿灯颜色 1:红 2:绿 3:黄
                time = phase['time'] # 红绿灯持续时间
                direction = phase['direction'] # 红绿灯方向 左转：direction > 180&&direction < 315右转：direction > 45 && direction <= 180 直行：其他角度 掉头：-180相位对应转向方向(0~360°即以当前道路方向为参考，邻接道路方向与当前方向的顺时针方向夹角)
                guide_speed_max = phase['guide_speed_max']
                guide_speed_min = phase['guide_speed_min']

                phase_msg = phase()  
                phase_msg.id = phase['id'] # 相位ID
                phase_msg.color = phase['color'] # 红绿灯颜色 1:红 2:绿 3:黄
                phase_msg.time = phase['time'] # 红绿灯持续时间
                phase_msg.direction = phase['direction'] # 红绿灯方向
                phase_msg.guide_speed_max = phase['guide_speed_max']
                phase_msg.guide_speed_min = phase['guide_speed_min']
                msg.phases.append(phase_msg)

            traffic_light_timing_pub.publish(msg)
            print(json.dumps({"id": id, "color": color, "time": time, "direction": direction, "guide_speed_max": guide_speed_max, "guide_speed_min": guide_speed_min}))
            print(json.dumps({"tag": 2104}))   


        if data['tag'] == 10300097: # OBU发送事件消息
            deviceNo = data['deviceNo'] # OBU设备id
            timestamp = data['timestamp'] # 时间戳,精确到毫秒
            roadPoints = data['roadPoints'] # 对应事件点位的经纬度，其中终点、途径点1、途径点2各包含一组经纬度数据
            msg = ObuEventMessage()
            msg.deviceNo = deviceNo
            msg.timestamp = timestamp
            print(json.dumps({"deviceNo": deviceNo, "timestamp": timestamp, "tag": 10300097}))
            for i, point in enumerate(roadPoints):
                longitude = point['longitude'] # 终点经度
                latitude = point['latitude'] # 终点纬度
                elevation = point['elevation'] # 终点高程数据，默认为 0，暂未使用。
                longitude = point['longitude'] # 途径点1经度
                latitude = point['latitude'] # 途径点1纬度
                elevation = point['elevation'] # 途径点1高程数据，默认为 0，暂未使用。
                longitude = point['longitude'] # 途径点2经度
                latitude = point['latitude'] # 途径点2纬度
                elevation = point['elevation'] # 途径点2高程数据，默认为 0，暂未使用。

                
                point1 = {'longitude': 0, 'latitude': 0, 'elevation': 0}
                point2 = {'longitude': 0, 'latitude': 0, 'elevation': 0}
                point3 = {'longitude': 0, 'latitude': 0, 'elevation': 0}
                
                # 假设 points 是一个包含所有点的列表
                points = [point1, point2, point3] 
                
                msg = RoadMessage()  
                
                for point in points:
                    point_msg = PointMessage() 
                    point_msg.longitude = point['longitude']
                    point_msg.latitude = point['latitude']
                    point_msg.elevation = point['elevation']
                    msg.roadPoints.append(point_msg)
                
                pub = rospy.Publisher('road_points', RoadMessage, queue_size=10)
                pub.publish(msg)
                print(json.dumps({"longitude": longitude, "latitude": latitude, "elevation": elevation}))

            obu_event_message_pub.publish(msg)


        if data['tag'] == 1002: # 感知共享反馈上报
            detail = data['detail'] # 保留字段不必关注
            status = data['status'] # 是否收到。0：收到；1：未收到
            msg = PerceptionShareFeedback()
            msg.detail = detail
            msg.status = status
            perception_share_feedback_pub.publish(msg)
            print(json.dumps({"detail": detail, "status": status}))


        if data['tag'] == 6001:  # 协作式变道请求
            deviceNo = data['deviceNo'] #第三方设备的ID
            remoteLon = data['remoteLon'] #背景车经度
            remoteLat = data['remoteLat'] # 背景车纬度
            remoteSpeed = data['remoteSpeed'] # 背景车速度
            remoteHeading = data['remoteHeading'] # 背景车航向
            timestamp = data['timestamp'] # 时间戳，精确到毫秒

            msg = CooperativeLaneChangeRequest()
            msg.deviceNo = deviceNo
            msg.remoteLon = remoteLon
            msg.remoteLat = remoteLat
            msg.remoteSpeed = remoteSpeed
            msg.remoteHeading = remoteHeading
            msg.timestamp = timestamp
            cooperative_lane_change_request_pub.publish(msg)

            print(json.dumps({"deviceNo": deviceNo, "remoteLon": remoteLon, "remoteLat": remoteLat, "remoteSpeed": remoteSpeed, "remoteHeading": remoteHeading, "timestamp": timestamp, "tag": 6001}))
  
        udp_socket.close()

if __name__ == '__main__':
    udp_receive()