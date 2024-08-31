#!/usr/bin/env python3
#coding=utf-8
'''
describe = "WIDC trajectary tracking port control"
author = "cyun"
version: 通过class管理相关的函数和变量
time = "2024.6.11"
'''
#!/usr/bin/env python3
#coding=utf-8

import os
import sys
import math
import time
import rospy
import signal
import threading
import yaml
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from sklearn.neighbors import KDTree

from car_interfaces.msg import PathSpeedCtrlInterface, GpsImuInterface, CarOriInterface, GlobalPathPlanningInterface, DecisionInterface

import prcolor
from path_control import PathControl
from speed_control import SpeedControl

class ControlNode:
    def __init__(self):
        self.is_exit = False
        self.CTE = 0
        self.dHead = 0
        self.dHead_filtered = 0
        self.mark = 0
        self.now_speed = 0
        self.change_line = 0
        self.end_point = []
        self.path_list_obj = []
        self.Brake = False
        self.insec_point = []
        self.station_point = []
        self.kdtree = None
        self.ego_content = {}
        self.trajectory = []
        self.L = 2.1

        self.gear_from_plan = 0
        self.action_from_plan = 0
        self.position = None
        self.x_list_ = []
        self.y_list_ = []

        self.gps_flag = False
        self.plan_flag = False
        self.camera_flag = False

        self.path_pub = None
        self.marker_pub = None
        # self.yaml_data = None
        self.pre_mark = None
        self.load_config()
        # print(self.yaml_data)
        self.dspeed = 0

        self.decision_flag = False


    def load_config(self):
        script_directory = os.path.dirname(os.path.abspath(__file__))
        relative_path = "../config/ControlParameter.yaml"
        file_path = os.path.join(script_directory, relative_path)

        try:
            with open(file_path, 'r') as file:
                self.yaml_data = yaml.load(file, Loader=yaml.FullLoader)
        except FileNotFoundError:
            print("ControlParameter.yaml 文件不存在")
            self.yaml_data = {}

    def global_path_planning_data_callback(self, msg):
        path_list = []
        self.x_list_ = list(msg.x_list)
        self.y_list_ = list(msg.y_list)
        self.insec_point = msg.incppoint
        self.end_point = msg.endpoint
        self.startpoint = msg.startpoint

        path_list_obj1 = []
        path_list = msg.routedata
        path_list = [float(num) for num in path_list]

        self.gear_from_plan = msg.plan_over
        self.action_from_plan = msg.action

        class PathItem:
            def __init__(self, x, y, head, speed, curva):
                self.x = x
                self.y = y
                self.head = head
                self.speed = speed
                self.curva = curva

        group_size = 5
        for i in range(len(path_list) // group_size):
            start_index = i * group_size
            end_index = start_index + group_size
            group = path_list[start_index:end_index]
            path_item = PathItem(*group)
            path_list_obj1.append(path_item)
        
        self.end_point = msg.endpoint
        self.path_list_obj = path_list_obj1

        x_list = path_list[::5]
        y_list = path_list[1::5]
        self.trajectory = [list(coord) for coord in zip(x_list, y_list)]
        self.plan_flag = True
        # print("path sub")

    def decision_data_callback(self, msg):
        self.change_line = msg.Change_Line
        self.Brake_En = msg.Brake_En
        self.Target_Travel = msg.Target_Travel
        self.dspeed = msg.dspeed
        self.decision_flag = True
        # print("**************")
        # print(msg.Change_Line)
        # print("sub decision")

    def call_back_GNSS(self, GPS_MSG):
        # x = GPS_MSG.x + self.L * math.cos(GPS_MSG.yaw)
        # y = GPS_MSG.y + self.L * math.sin(GPS_MSG.yaw)
        # self.ego_content['now_pos_x'] = x
        # self.ego_content['now_pos_y'] = y
        self.ego_content['now_pos_x'] = GPS_MSG.x
        self.ego_content['now_pos_y'] = GPS_MSG.y
        self.ego_content['now_pos_head'] = GPS_MSG.yaw
        self.now_pos_x = GPS_MSG.x
        self.now_pos_y = GPS_MSG.y
        # self.now_pos_x = x
        # self.now_pos_y = y

        self.now_pos_head = GPS_MSG.yaw
        self.now_speed = GPS_MSG.Vel
        self.gps_flag = True
        # print("gps sub")


    def signal_handler(self, signal, frame):
        self.is_exit = True

    def caculate_cte_wheel_angle(self, line_length):

        #lat, lon and head
        class Pos:
            def __init__(self, x, y, head):
                self.x = x
                self.y = y
                self.head = head
                pass

        class Point_:
            def __init__(self, x, y, head):
                self.x = x
                self.y = y
                self.head = head
                pass


        if len(self.path_list_obj) == 0:
            return 0, 0

        perception_mark = self.yaml_data['perception_mark'] if self.yaml_data else 8

        if self.now_speed <= 3.5:
            perception_mark = int(self.now_speed * 2.5)
        if self.now_speed >= 6:
            perception_mark = int(self.now_speed * 1)
        if self.now_speed >= 8:
            perception_mark = 10
        if self.now_speed >= 10.5:
            perception_mark = 9

        perception_mark = 8
        if self.pre_mark is not None:
            perception_mark = self.pre_mark

        curPos_kd = [self.ego_content['now_pos_x'], self.ego_content['now_pos_y']]
        curPos = Point_(self.ego_content['now_pos_x'], self.ego_content['now_pos_y'], self.ego_content['now_pos_head'])

        self.kdtree = KDTree(self.trajectory)
        distances, indices = self.kdtree.query([curPos_kd], k=1)
        self.mark = indices[0][0]
        curDis = float(distances)

        mark_cood = Point_(self.path_list_obj[self.mark].x, self.path_list_obj[self.mark].y, self.path_list_obj[self.mark].head)
        ptB = Point_(self.path_list_obj[min((self.mark + 1), len(self.path_list_obj)-1)].x, self.path_list_obj[min((self.mark + 2), len(self.path_list_obj)-1)].y,
                    self.path_list_obj[min((self.mark + 1), len(self.path_list_obj)-1)].head)
        dx = mark_cood.x - ptB.x
        dy = mark_cood.y - ptB.y
        Rx = curPos.x - ptB.x
        Ry = curPos.y - ptB.y

        self.CTE = (Ry * dx - Rx * dy) / ((dx ** 2 + dy ** 2) ** 0.5 + 0.0000001)

        # TODO dHead滤波

        try:
            curPos_head_ = curPos.head
            path_head_ = self.path_list_obj[self.mark].head
            path_head_p = self.path_list_obj[min((self.mark + perception_mark), len(self.path_list_obj)-1)].head
            self.dHead = curPos_head_ - path_head_p
            # prcolor.prRed("aaaaaaa")
        except:

            self.dHead = curPos_head_ - path_head_

        if self.dHead < -180:
            self.dHead = self.dHead + 360
        if self.dHead > 180:
            self.dHead = self.dHead - 360

        kappa = 0

        #  换道指令
        if self.change_line == 0:
            self.CTE = self.CTE
        elif self.change_line == 1:
            self.CTE += line_length
        elif self.change_line == 2:
            self.CTE -= line_length
        # print("self.yaml_data",self.yaml_data)
        print("111111111111111111")
        print("cte",self.CTE)
        pathcontrol = PathControl(self.yaml_data)
        wheel_angle = pathcontrol.path_control(self.CTE, self.dHead, kappa, self.now_speed, self.yaml_data, dt=None)
        if wheel_angle > 100:
            self.pre_mark = 15
        if wheel_angle > 250:
            self.pre_mark = 20
        
        speed_ctrl_to_wire = self.path_list_obj[self.mark].speed
        print("self.path_list_obj",len(self.path_list_obj),self.mark)

        return wheel_angle, speed_ctrl_to_wire

    def publish_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "/map"

        if self.x_list_ and self.y_list_ and len(self.x_list_) == len(self.y_list_):
            for x, y in zip(self.x_list_, self.y_list_):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = "/map"
                pose_stamped.pose.position.x = x - self.startpoint[0]
                pose_stamped.pose.position.y = y - self.startpoint[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path.poses.append(pose_stamped)
        return path

    def publish_mark(self):
        if len(self.startpoint) == 0:
            return None

        vehicle_position = Point()
        vehicle_position.x = self.now_pos_x - self.startpoint[0]
        vehicle_position.y = self.now_pos_y - self.startpoint[1]
        vehicle_position.z = 0.0

        heading = self.convert_deg_to_rad(self.now_pos_head)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = vehicle_position
        quaternion = quaternion_from_euler(0, 0, heading)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        marker.scale.x = 3
        marker.scale.y = 1.5
        marker.scale.z = 1.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.3
        marker.color.b = 0.2
        return marker

    def convert_deg_to_rad(self, degrees):
        return (degrees + 180) * (2 * math.pi / 360)

    def main(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('control', anonymous=False)
        pub_control_ = rospy.Publisher("path_speed_tracking_data", PathSpeedCtrlInterface, queue_size=10)
        self.path_pub = rospy.Publisher("trajectory", Path, queue_size=10)
        self.marker_pub = rospy.Publisher("vehicle_marker", Marker, queue_size=10)

        rospy.Subscriber('gps_imu', GpsImuInterface, self.call_back_GNSS)
        rospy.Subscriber("global_path_planning_data", GlobalPathPlanningInterface, self.global_path_planning_data_callback)
        rospy.Subscriber("/decision_data", DecisionInterface, self.decision_data_callback)

        rospy.logwarn("control start success")
        rospy.logwarn("********等待规划层消息********")
        rospy.logwarn("********等待gps消息********")


        line_length = rospy.get_param("~line_length", 3.6)

        msg = PathSpeedCtrlInterface()

        wheel_angle = 0
        rate = rospy.Rate(100)
        while not self.plan_flag or not self.gps_flag and not rospy.is_shutdown():
            rospy.sleep(1)

        while not rospy.is_shutdown():
            wheel_angle, speed_ctrl_to_wire = self.caculate_cte_wheel_angle(line_length)
            speedcontrol = SpeedControl(self.yaml_data)
            # speed_ctrl_to_wire -= 2
            speed_ctrl_to_wire += self.dspeed
            # if self.dspeed != 0:
            #     wheel_angle = wheel_angle / 2
            print("wheel", self.dspeed, wheel_angle)
            speedTorque, speedBrake = speedcontrol.speed_control(wheel_angle, self.now_speed, speed_ctrl_to_wire, self.yaml_data)
            print("speedTorque",speedTorque)
            msg.Target_Angle = wheel_angle
            msg.TargetPedalOpen = speedTorque
            msg.Target_Travel = speedBrake
            msg.CTE = self.CTE
            msg.dHead = self.dHead
            msg.Target_velocity = speed_ctrl_to_wire
            msg.now_speed = self.now_speed
            msg.Parking_Req = 1
            msg.IPC_En = 1
            msg.Target_Gear = 4

            pub_control_.publish(msg)
            print("speed_ctrl_to_wire",speed_ctrl_to_wire)
            # path = self.publish_path()
            # self.path_pub.publish(path)

            # marker = self.publish_mark()
            # self.marker_pub.publish(marker)

            rate.sleep()
            

if __name__ == '__main__':
    control_node = ControlNode()
    control_node.main()
