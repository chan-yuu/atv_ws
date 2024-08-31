#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# cyun 2023 12

# cyun 2024 WIDC

import sys
import os
import rospy
import signal
import json
import threading
import time
import math

from sklearn.neighbors import KDTree
from car_interfaces.msg import PathSpeedCtrlInterface,CameraInterface,ChangeMapPlanInterface, GpsImuInterface, GlobalPathPlanningInterface, DecisionInterface
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
# from hmi.msg import NodePointsInterface
import numpy as np
# from lidar_select import point_select
import prcolor
import readMap

class DecisionNode:
    def __init__(self):
        # 初始化节点
        rospy.init_node('car_decision', anonymous=False)
        
        # 全局变量初始化
        self.is_exit = False
        self.plan_flag = False
        self.lidar_know_flag = False
        self.lidar_unknow_flag = False
        self.control_flag = False
        self.gps_flag = False
        self.local_map_flag = False
        self.front_obj = None
        self.left_front_obj = None
        self.right_front_obj = None
        self.front_obj_unkown = None
        self.left_front_obj_unkown = None
        self.right_front_obj_unkown = None
        self.behind_obj = None
        self.left_behind_obj = None
        self.right_behind_obj = None
        self.lateral_left_obj = None
        self.lateral_right_obj = None
        self.ego_content = {}
        self.local_Map_dict = None
        self.lidar_know_objs_list = []
        self.lidar_unknow_objs_list = []
        self.offset_fromplan = 0
        self.mapfile = None
        self.current_map = None
        self.ROI_obj_dict = {}
        self.ROI_obj_dict_record = {}
        self.now_speed = 0
        # self.old_path = os.path.abspath('.')
        # self.new_path = self.old_path.replace(".ros", self.work_space_name)
        # self.decision_lib_file = os.path.join(self.new_path, 'src/decision/scripts/')
        # sys.path.append(self.decision_lib_file)
        self.static_num = 0
        self.flag_map = 0

        self.heart_up_msg = {}
        self.redgreen_up_msg = {}
        self.redgreen_countdown_up_msg = {}
        self.obu_up_msg = {}
        self.perceive_up_msg = {}

        self.time  = 0.0
        self.Brake_En = 0
        self.Target_Travel = 0

        self.Change_Line = 0
        self.laneid = 0 # 0-中间车道 1-左车道 2-右车道

        self.current_Emergency = 0
        self.num = 0
        self.while_number = 0
        # self.current_Emergency = None
        self.obj_list = []
        self.laneid = 0

        self.is_fog = 0
        self.lap = 0
        self.dspeed = 0

    def signal_handler(self, signal, frame):
        self.is_exit = True

    def call_back_lidar_msg(self, msg):
        try:
            lidar_know_objs_msg = json.loads(msg.data)
            self.lidar_know_objs_list = lidar_know_objs_msg['objects']
            self.lidar_know_flag = True
        except:
            self.lidar_know_objs_list = []
            print('lidar know data error')


    def call_back_ego_vehicle_data(self, msg):
        try:
            self.ego_content['UTM_x'] = msg.posX
            self.ego_content['UTM_y'] = msg.posY
            self.ego_content['Speed'] = msg.Vel
            self.ego_content['Lat'] = msg.PosLat
            self.ego_content['Lon'] = msg.PosLon
            self.ego_content['yaw'] = msg.AngleHeading
            self.gps_flag = True
        except:
            print('content error')

    def global_path_planning_data_callback(self, msg):
        self.mapfile = msg.roadid
        self.plan_flag = True

    def Heart_Up_Callback(self, msg):
        heart_up_msg = json.loads(msg.data)
        self.heart_up_msg['tag'] = heart_up_msg['tag']
        self.heart_up_msg['deviceNo'] = heart_up_msg['deviceNo']

    def RedGreen_Up_Callback(self, msg):
        redgreen_up_msg = json.loads(msg.data)
        self.redgreen_up_msg['tag'] = redgreen_up_msg['tag']
        self.redgreen_up_msg['detail'] = redgreen_up_msg['detail']
        self.redgreen_up_msg['status'] = redgreen_up_msg['status']
   
    def RedGreen_Countdown_Up_Callback(self, msg):
        redgreen_countdown_up_msg = json.loads(msg.data)
        self.redgreen_countdown_up_msg['tag'] = redgreen_countdown_up_msg['tag']
        self.redgreen_countdown_up_msg['data'] = redgreen_countdown_up_msg['data']
        self.redgreen_countdown_up_msg['data']['device_id'] = redgreen_countdown_up_msg['data']['device_id']
        self.redgreen_countdown_up_msg['data']['lon'] = redgreen_countdown_up_msg['data']['lon']
        self.redgreen_countdown_up_msg['data']['lat'] = redgreen_countdown_up_msg['data']['lat']
        self.redgreen_countdown_up_msg['data']['ele'] = redgreen_countdown_up_msg['data']['ele']
        self.redgreen_countdown_up_msg['data']['hea'] = redgreen_countdown_up_msg['data']['hea']
        self.redgreen_countdown_up_msg['data']['region_id'] = redgreen_countdown_up_msg['data']['region_id']
        self.redgreen_countdown_up_msg['data']['local_id'] = redgreen_countdown_up_msg['data']['local_id']
        self.redgreen_countdown_up_msg['data']['dist'] = redgreen_countdown_up_msg['data']['dist']
        self.redgreen_countdown_up_msg['data']['phase'] = redgreen_countdown_up_msg['data']['phase']
        self.redgreen_countdown_up_msg['data']['phase']['id'] = redgreen_countdown_up_msg['data']['phase']['id']
        self.redgreen_countdown_up_msg['data']['phase']['color'] = redgreen_countdown_up_msg['data']['phase']['color']
        self.redgreen_countdown_up_msg['data']['phase']['time'] = redgreen_countdown_up_msg['data']['phase']['time']
        self.redgreen_countdown_up_msg['data']['phase']['direction'] = redgreen_countdown_up_msg['data']['phase']['direction']
                
    def Obu_Up_Callback(self, msg):
        obu_up_msg = json.loads(msg.data)
        self.obu_up_msg['tag'] = obu_up_msg['tag']
        self.obu_up_msg['deviceNo'] = obu_up_msg['deviceNo']
        self.obu_up_msg['timestamp'] = obu_up_msg['timestamptag']
        self.obu_up_msg['tagroadPoints'] = obu_up_msg['taroadPointsg']
        self.obu_up_msg['tagroadPoints']['longitude'] = obu_up_msg['tagroadPoints']['longitude']
        self.obu_up_msg['tagroadPoints']['latitude'] = obu_up_msg['tagroadPoints']['latitude']
        self.obu_up_msg['tagroadPoints']['elevation'] = obu_up_msg['tagroadPoints']['elevation']
        self.obu_up_msg['tagroadPoints']['longitude'] = obu_up_msg['tagroadPoints']['longitude']
        self.obu_up_msg['tagroadPoints']['latitude'] = obu_up_msg['tagroadPoints']['latitude']
        self.obu_up_msg['tagroadPoints']['elevation'] = obu_up_msg['tagroadPoints']['elevation']
        self.obu_up_msg['tagroadPoints']['longitude'] = obu_up_msg['tagroadPoints']['longitude']
        self.obu_up_msg['tagroadPoints']['latitude'] = obu_up_msg['tagroadPoints']['latitude']
        self.obu_up_msg['tagroadPoints']['elevation'] = obu_up_msg['tagroadPoints']['elevation']


    def Perceive_Up_Callback(self, msg):
        perceive_up_msg = json.loads(msg.data)
        self.perceive_up_msg['tag'] = perceive_up_msg['tag']
        self.perceive_up_msg['detail'] = perceive_up_msg['detail']
        self.perceive_up_msg['status'] = perceive_up_msg['status']


    def callback_offset(self, msg):
        self.offset_fromplan = msg.CTE
        self.control_flag = True

    def calculate_dis(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def find_closest_obj(self, obj_list, initial_distance=60):
        closest_distance = initial_distance
        closest_obj = None
        # 如果closet_obj = None , 返回None
        for obj in obj_list:
            if obj["s"] < closest_distance:
                closest_distance = obj["s"]
                closest_obj = obj
        return closest_obj

    def nearest_lidar_obj_select(self, obj_list, ego_content, local_Map_dict, offset_fromplan):
        left_value = 1.8
        right_value = -1.8
        left_left_value = 4.5
        right_right_value = -4.5

        obj_list_final_front = []
        obj_list_final_left_front = []
        obj_list_final_right_front = []
        obj_list_lateral_left = []
        obj_list_lateral_right = []

        obj_final_front_veh = None
        obj_final_left_front_veh = None
        obj_final_right_front_veh = None

        if obj_list is not None:
            for i in range(len(obj_list)):
                if obj_list[i] is not None:
                    # dis_to_ego = math.sqrt(math.pow(obj_list[i]['core_y'], 2) + math.pow(obj_list[i]['core_x'], 2))
                    # theta_to_ego = math.atan2(obj_list[i]['core_y'], obj_list[i]['core_x']) * 180 / math.pi
                    yaw_rad = math.radians(ego_content['yaw'])
                    
                    obj_list[i]['UTM_x'] = ego_content["UTM_x"] + math.cos(yaw_rad) * obj_list[i]['core_x'] - obj_list[i]['core_y'] * math.sin(yaw_rad)
                    obj_list[i]['UTM_y'] = ego_content["UTM_y"] + math.sin(yaw_rad) * obj_list[i]['core_x'] + obj_list[i]['core_y'] * math.cos(yaw_rad)

                    curPos = [ego_content["UTM_x"], ego_content["UTM_y"]]
                    curObj = [obj_list[i]['UTM_x'], obj_list[i]['UTM_y']]

                    trajectory = [list(coord) for coord in zip(local_Map_dict['X_list'], local_Map_dict['Y_list'])]
                    kdtree = KDTree(trajectory)
                    disobj, indices2 = kdtree.query([curObj], k=1)
                    index_obj_l = indices2[0][0]
                    disego, indices1 = kdtree.query([curPos], k=1)
                    index_ego_l = indices1[0][0]

                    obj_list[i]['s'] = (index_obj_l - index_ego_l) * 0.1

                    # vector_1 = [(obj_list[i]['UTM_x'] - local_Map_dict['X_list'][index_obj_l]), (obj_list[i]['UTM_y'] - local_Map_dict['Y_list'][index_obj_l])]
                    # print(type(local_Map_dict['X_list'][min(index_obj_l + 1, len(local_Map_dict['X_list'])-1)]))
                    # vector_2 = [(obj_list[i]['UTM_x'] - local_Map_dict['X_list'][min(index_obj_l + 1, len(local_Map_dict['X_list'])-1)]), (obj_list[i]['UTM_y'] - local_Map_dict['Y_list'][min(index_obj_l + 1, len(local_Map_dict['X_list'] - 1))])]
                    # vector_2 = [(obj_list[i]['UTM_x'] - local_Map_dict['X_list'][index_obj_l + 1]), (obj_list[i]['UTM_y'] - local_Map_dict['Y_list'][index_obj_l + 1])]
                    vector_1 = [(obj_list[i]['UTM_x'] - local_Map_dict['X_list'][max(index_obj_l-1,0)]), (obj_list[i]['UTM_y'] - local_Map_dict['Y_list'][max(index_obj_l-1,0)])]
                    vector_2 = [(obj_list[i]['UTM_x'] - local_Map_dict['X_list'][min(index_obj_l + 1, len(local_Map_dict['X_list'])-1)]), (obj_list[i]['UTM_y'] - local_Map_dict['Y_list'][min(index_obj_l + 1, len(local_Map_dict['Y_list'])-1)])]

                    nearest_distance_lateral = (vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0]) / \
                                               (math.sqrt((local_Map_dict['X_list'][max(index_obj_l-1,0)] - local_Map_dict['X_list'][min(index_obj_l + 1, len(local_Map_dict['X_list'])-1)]) ** 2 + (
                                                       local_Map_dict['Y_list'][max(index_obj_l-1,0)] - local_Map_dict['Y_list'][min(index_obj_l + 1, len(local_Map_dict['Y_list'])-1)]) ** 2))

                    if obj_list[i]['core_y'] < 0:
                        obj_list[i]['l'] = -abs(nearest_distance_lateral)
                    else:
                        obj_list[i]['l'] = abs(nearest_distance_lateral)

                    if 0 < obj_list[i]['s'] < 60:
                        if (obj_list[i]["l"] - offset_fromplan < left_value) & (obj_list[i]["l"] - offset_fromplan > right_value):
                            obj_list_final_front.append(obj_list[i])
                        if (obj_list[i]["l"] - offset_fromplan < left_left_value) & (obj_list[i]["l"] - offset_fromplan >= left_value):    
                            obj_list_final_left_front.append(obj_list[i])
                        if (obj_list[i]["l"] - offset_fromplan <= right_value) & (obj_list[i]["l"] - offset_fromplan >= right_right_value):
                            obj_list_final_right_front.append(obj_list[i])

                    # if -15 < obj_list[i]['s'] < 0:
                    #     if (obj_list[i]["l"] - offset_fromplan < left_left_value) & (obj_list[i]["l"] - offset_fromplan > left_value):
                    #         obj_list_lateral_right.append(obj_list[i])
                    #     if (obj_list[i]["l"] - offset_fromplan > right_right_value) & (obj_list[i]["l"] - offset_fromplan < right_value):
                    #         obj_list_lateral_left.append(obj_list[i])

                # 继续从三条道路中筛选出各自最短的s。
                obj_final_front_veh = self.find_closest_obj(obj_list_final_front)
                obj_final_left_front_veh = self.find_closest_obj(obj_list_final_left_front)
                obj_final_right_front_veh = self.find_closest_obj(obj_list_final_right_front)

        return obj_final_front_veh, obj_final_left_front_veh, obj_final_right_front_veh


    def lane_change_request_to_decision_Callback(self, msg):
        lane_change_request_to_decision_msg = json.loads(msg.data)
        self.lane_change_request_to_decision_msg['tag'] = lane_change_request_to_decision_msg['tag']
        self.lane_change_request_to_decision_msg['deviceNo'] = lane_change_request_to_decision_msg['deviceNo']
        self.lane_change_request_to_decision_msg['remoteLon'] = lane_change_request_to_decision_msg['remoteLon']        
        self.lane_change_request_to_decision_msg['remoteLat'] = lane_change_request_to_decision_msg['remoteLat']
        self.lane_change_request_to_decision_msg['remoteSpeed'] = lane_change_request_to_decision_msg['remoteSpeed']
        self.lane_change_request_to_decision_msg['remoteHeading'] = lane_change_request_to_decision_msg['remoteHeading']
        self.lane_change_request_to_decision_msg['timestamp'] = lane_change_request_to_decision_msg['timestamp']
        self.dspeed -= 2
        print("对方变道，本车减速")


    def ROI_objs_select_from_camera_and_lidar(self):
        while not self.gps_flag or not self.control_flag or not self.local_map_flag and not rospy.is_shutdown():
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            start_time = time.time()

            if len(self.local_Map_dict['X_list']) != 0:
                print("111", len(self.local_Map_dict['X_list']))
                lidar_know_final_front_veh, lidar_know_final_left_front_veh, lidar_know_final_right_front_veh, lidar_know_lateral_left_veh, lidar_know_lateral_right_veh = \
                    self.nearest_lidar_obj_select(self.lidar_know_objs_list, self.ego_content, self.local_Map_dict, self.offset_fromplan)

                if lidar_know_final_front_veh is not None:
                    self.front_obj = lidar_know_final_front_veh
                else:
                    self.front_obj = None

                if lidar_know_final_left_front_veh is not None:
                    self.left_front_obj = lidar_know_final_left_front_veh
                else:
                    self.left_front_obj = None

                if lidar_know_final_right_front_veh is not None:
                    self.right_front_obj = lidar_know_final_right_front_veh
                else:
                    self.right_front_obj = None

                self.ROI_obj_dict = {
                    'front_obj': self.front_obj,
                    'left_front_obj': self.left_front_obj,
                    'right_front_obj': self.right_front_obj,
                }

                if self.front_obj is not None:
                   # 将UTM坐标转换为JSON格式
                   data = {'UTM_x': self.front_obj['UTM_x'], 'UTM_y': self.front_obj['UTM_y']}
                   msg_data = json.dumps(data)
                   # 将JSON格式的UTM坐标转换为String类型
                #    UTM_msg = String()
                #    UTM_msg.data = UTM_data
                   self.topic_1001.publish(msg_data)


                if self.front_obj is not None and self.front_obj['s'] < 8.5:
                    Emergency_brake_dict = {"Emergency_brake_decision": True}
                    brake_record = True
                    Brake_En = 1
                    Target_Travel = 10
                    print(self.front_obj['s'])

                else:
                    Emergency_brake_dict = {"Emergency_brake_decision": False}
                    brake_record = False
                    Brake_En = 0
                    Target_Travel = 0
                print(Emergency_brake_dict)

                msg_send_to_vcu = DecisionInterface()
                msg_send_to_vcu.Brake_En = Brake_En
                msg_send_to_vcu.Target_Travel = Target_Travel

                self.brake_pub.publish(msg_send_to_vcu)
                # self.lidar_know_flag = False

                end_time = time.time()
                process_time = end_time - start_time

    def Perceive_Up_Callback(self, msg):
        perceive_up_msg = json.loads(msg.data)
        self.perceive_up_msg['tag'] = perceive_up_msg['tag']
        self.perceive_up_msg['detail'] = perceive_up_msg['detail']
        self.perceive_up_msg['status'] = perceive_up_msg['status']
    def plan_Callback_category(self, msg):
        self.dspeed = msg.dspeed
        self.cte_offset = msg.cte_offset
        self.Brake_Enable = msg.Brake_Enable
        self.brake_pedel = msg.brake_pedel

    def camera_Callback(self, msg):
        self.is_fog = msg.is_fog
        self.lap = msg.lap
        # print("雾天检测")

    def main(self):
        # 信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        # 发布话题
        self.Emergency_brake_pub = rospy.Publisher('/emergency_brake_from_decision', String, queue_size=10)
        self.decision_pub = rospy.Publisher('/decision_data', DecisionInterface, queue_size=10)
        self.topic_1001 = rospy.Publisher('/topic_1001', String, queue_size=10)

        # 订阅话题
        rospy.Subscriber('/path_speed_tracking_data', PathSpeedCtrlInterface, self.callback_offset)
        rospy.Subscriber('/object_array_json', String, self.call_back_lidar_msg)
        rospy.Subscriber('/gps_imu', GpsImuInterface, self.call_back_ego_vehicle_data)
        rospy.Subscriber("/global_path_planning_data", GlobalPathPlanningInterface, self.global_path_planning_data_callback)
        
        
        rospy.Subscriber("/heart_up_v2x", String, self.Heart_Up_Callback)
        rospy.Subscriber("/redgreen_up_v2x", String, self.RedGreen_Up_Callback)
        rospy.Subscriber("/redgreen_countdown_up_v2x", String, self.RedGreen_Countdown_Up_Callback)
        rospy.Subscriber("/obu_up_v2x", String, self.Obu_Up_Callback)
        rospy.Subscriber("/perceive_up_v2x", String, self.Perceive_Up_Callback)      

        rospy.Subscriber("/lane_change_request_to_decision", String, self.lane_change_request_to_decision_Callback)      
        rospy.Subscriber("/perceive_up_v2x", String, self.Perceive_Up_Callback)
        rospy.Subscriber("/plan_to_decision", ChangeMapPlanInterface, self.plan_Callback_category)      

        rospy.Subscriber("/decision_data_from_camera", CameraInterface, self.camera_Callback)      


        # 打印日志
        rospy.logwarn('decision start Success')
        rospy.logwarn('请下发map信息')
        rospy.logwarn('等待gps消息')
        rospy.logwarn('请下发lidar信息')

        # # 启动线程
        # self.obj_select_thread = threading.Thread(target=self.ROI_objs_select_from_camera_and_lidar)
        # self.obj_select_thread.start()


        while not self.gps_flag or not self.plan_flag or not self.control_flag:
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            start_time = time.time()
            # print( self.gps_flag ,self.plan_flag , self.control_flag)
            # x_list_global = []
            if self.current_map != self.mapfile:
            # if self.flag_map == 0:
                Global_Map_Dict, x_list_global, y_list_global, head_list_global = readMap.readMap(self.mapfile)
                trajectory = [list(coord) for coord in zip(x_list_global, y_list_global)]
                self.kdtree = KDTree(trajectory)
                self.flag_map = 1
                self.current_map = self.mapfile

            if len(x_list_global) != 0:
                curPos = [self.ego_content['UTM_x'], self.ego_content['UTM_y']]
                distances, indices = self.kdtree.query([curPos], k=1)
                mark = indices[0][0]

                x_list_local = x_list_global[max(0, mark - 20): min(len(x_list_global), mark + 350)]
                y_list_local = y_list_global[max(0, mark - 20): min(len(y_list_global), mark + 350)]
                head_list_local = head_list_global[max(0, mark - 50): min(len(head_list_global), mark + 200)]
                
                self.local_Map_dict = {
                    "X_list": x_list_local,
                    "Y_list": y_list_local,
                    "heading_list": head_list_local
                }
                self.local_map_flag = True

            
            if self.local_Map_dict and self.lidar_know_flag:
                # start = time.time()
                # print("111", len(self.local_Map_dict['X_list']))
                lidar_know_final_front_veh, lidar_know_final_left_front_veh, lidar_know_final_right_front_veh = \
                    self.nearest_lidar_obj_select(self.lidar_know_objs_list, self.ego_content, self.local_Map_dict, self.offset_fromplan)

                if lidar_know_final_front_veh is not None:
                    self.front_obj = lidar_know_final_front_veh
                else:
                    self.front_obj = None

                if lidar_know_final_left_front_veh is not None:
                    self.left_front_obj = lidar_know_final_left_front_veh
                else:
                    self.left_front_obj = None

                if lidar_know_final_right_front_veh is not None:
                    self.right_front_obj = lidar_know_final_right_front_veh
                else:
                    self.right_front_obj = None

                self.ROI_obj_dict = {
                    'front_obj': self.front_obj,
                    'left_front_obj': self.left_front_obj,
                    'right_front_obj': self.right_front_obj,
                }

                # if self.front_obj is not None and 0<self.front_obj['s'] < 8.5:
                #     Emergency_brake_dict = {"Emergency_brake_decision": True}
                #     Emergency_brake = True
                #     self.Brake_En = 1
                #     self.Target_Travel = 15
                #     print(self.front_obj['s'])
                # elif self.front_obj is not None:
                #     print(type(self.front_obj))
                #     Emergency_brake = False
                #     Emergency_brake_dict = {"Emergency_brake_decision": False}
                #     self.Brake_En = 0
                #     self.Target_Travel = 0
                # 保持检测效果[
                # print("self.front_obj['s']",self.front_obj)
                # 换道：
                if self.front_obj is not None and 0<self.front_obj['s'] < 20.5:#and self.laneid == 0:
                    Emergency_brake_dict = {"Emergency_brake_decision": True}
                    self.Change_Line = 1
                    # 减速
                    self.dspeed = -1
                    print("self.front_obj['s']",self.front_obj['s'])
                    print(self.front_obj['l']-self.offset_fromplan,"lllll")
                else:
                    Emergency_brake_dict = {"Emergency_brake_decision": False}
                    self.Change_Line = 0
                    # 减速
                    self.dspeed = -0
                    self.laneid = 0
                if len(self.obj_list) < 15:
                    self.obj_list.append(Emergency_brake_dict["Emergency_brake_decision"])

                else:
                    self.obj_list.pop(0)
                    self.obj_list.append(Emergency_brake_dict["Emergency_brake_decision"])
                self.current_Emergency = 0
                for i in range(len(self.obj_list)):
                    if self.obj_list[i]  == True:
                        self.current_Emergency += 1
                    if self.current_Emergency >= 5:
                        flag_true = True
                    else:
                        flag_true = False
                        
                print("flag_true", flag_true)
                print("self.obj_list", self.obj_list)
                msg_send_to_vcu = DecisionInterface()
                if flag_true:
                    self.Change_Line = 1
                if self.Change_Line == 1:
                    msg_send_to_vcu.Change_Line = self.Change_Line
                    msg_send_to_vcu.dspeed = -2.5

                if self.is_fog == 1:
                    msg_send_to_vcu.dspeed = -2.5

                print("self.obj_list", self.obj_list)

                self.lidar_know_flag = False
                # print(self.Change_Line)
                end = time.time()
                # print(1/(end - start), "hz")

                # msg_send_to_vcu = DecisionInterface()
                msg_send_to_vcu.Brake_En = self.Brake_En
                msg_send_to_vcu.Target_Travel = self.Target_Travel
                
                self.decision_pub.publish(msg_send_to_vcu)
                # self.lidar_know_flag = False
                # rospy.Rate(100).sleep()
                end_time = time.time()
                process_time = end_time - start_time
                # print("time", process_time)


if __name__ == "__main__":
    decision_node = DecisionNode()
    decision_node.main()
    rospy.spin()
