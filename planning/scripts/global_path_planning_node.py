#!/usr/bin/python3
# -*- coding: utf-8 -*-
# cyun 2023 12
#!/usr/bin/env python3
#coding=utf-8
# 改为class的形式

import os
import sys
import time
import math
import signal
import glob
import rospy
from sklearn.neighbors import KDTree
from car_interfaces.msg import GlobalPathPlanningInterface, GpsImuInterface, CarOriInterface, ChangeMapPlanInterface, V2xInterface
import prcolor
import numpy as np
import threading
from tf.transformations import quaternion_from_euler
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
import pyproj,json

from std_msgs.msg import String



class GlobalPathPlanningNode:
    def __init__(self):
        self.now_pos_x = 0.0
        self.now_pos_y = 0.0
        self.now_pos_head = 0.0
        self.now_speed = 0.0
        self.insec_point = []
        self.station_point = []
        self.hmi_flag = False
        self.node_flag = False
        self.gps_flag = False
        self.ego_content = {}
        self.map_files = []
        self.mapfile = ""
        self.current_mapfile = None


        self.start_point = [] # v2x的起点

        self.end_point = []

        self.turning_point1 = []
        self.turning_point2 = []

        self.dis_1 = 0
        self.dis_2 = 0
        self.num  = 0
        self.mark = 0
        self.dis_turn = 0

        # 可视化：
        self.x_list = []
        self.y_list = []
        self.x_list_l = []
        self.y_list_l = []
        # map的起点
        self.startpoint = []

        self.v2x_turn_flag = False
        # self.end_point = []
        # self.turning_point1 = []
        # self.turning_point2 = []

    def readMap(self, mapfile):

        class Road:
            def __init__(self):
                self.id = -1
                self.lane = []
                pass

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
                        # seg_new = []
                        vel_values.append(float(seg[5]))
                        # seg_new = from_latlon(float(seg[3]),float(seg[4]))
                        lane.points.append(Pos(float(seg[3]), float(seg[4]), float(seg[2])))
                road.lane.append(lane)
                Map.append(road)
            for pos in lane.points:
                # print(f"Pos: x={pos.x}, y={pos.y}, head={pos.head}")
                pass
        # print (Map[0].lane[0].points)

        x_values = [pos.x for pos in lane.points]
        y_values = [pos.y for pos in lane.points]
        head_values = [pos.head for pos in lane.points]

        return x_values,y_values,head_values,vel_values,road.id,lane.id



    def load_config(self, path):
        script_path = os.path.dirname(os.path.abspath(__file__))
        directory = path
        abs_path = os.path.join(script_path, directory)
        self.map_files = glob.glob(os.path.join(abs_path, "*.map"))

    def call_back_GPS(self, msg):
        self.now_pos_x = msg.posX
        self.now_pos_y = msg.posY
        self.now_pos_head = msg.AngleHeading
        self.gps_flag = True

        # test:
        # 539202.29592411965,4336586.75334715378

    # def hmi_start_end_point_data_callback(self, msg):
    #     self.roadid = msg.roadid
    #     self.stationid = msg.stationid
    #     self.flag_hmi = msg.flag
    #     self.hmi_flag = True

    # def node_point_data_callback(self, msg):
    #     self.station_point = msg.stationpoint
    #     self.insec_point = msg.incppoint
    #     self.station_point = self.insec_point
    #     self.node_flag = True

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def calculate_k(self, x1, y1, x2, y2):
        return (y2 - y1) / (x2 - x1)

    def calculate_head(self, x, y):
        headings = []
        head_list = []
        for i in range(len(x)):
            if i == 0:
                dx = x[i + 1] - x[i]
                dy = y[i + 1] - x[i]
            elif i == len(x) - 1:
                dx = x[i] - x[i - 1]
                dy = y[i] - y[i - 1]
            else:
                dx = x[i + 1] - x[i]
                dy = y[i + 1] - y[i]
            heading = np.arctan2(dy, dx)
            headings_angle = math.degrees(heading)
            headings.append(heading)
            head_list.append(headings_angle)
        return head_list

    def cal_road(self):
        min_distance = float('inf')
        min_distance_map = ""
        min_distance_txt = ""
        for map_file in self.map_files:
            x_list, y_list, head_list, vel_list, gear, action = self.readMap(map_file)
            trajectory = [list(coord) for coord in zip(x_list, y_list)]
            kdtree = KDTree(trajectory)

            curPos1 = [self.turning_point1[0], self.turning_point1[1]]
            curPos2 = [self.turning_point2[0], self.turning_point2[1]]

            distances1, indices = kdtree.query([curPos1], k=1)
            distances2, indices = kdtree.query([curPos2], k=1)

            current_distance = distances1 + distances2

            mark = indices[0][0]
            # mark_index = mark
            # txt_file = os.path.splitext(map_file)[0] + ".txt"
            if current_distance < min_distance:
                min_distance = current_distance
                min_distance_map = map_file
            #     min_distance_txt = txt_file
        # return min_distance_map, min_distance_txt, min_distance
        return min_distance_map, min_distance

    def caculate_point(self, txt_path):
        data_list = []
        with open(txt_path, 'r') as file:
            for line in file:
                values = line.strip().split(';')
                x = float(values[1])
                y = float(values[2])
                data_list.extend([x, y])
        return data_list

    # def wire_data_callback(self, msg):
    #     if msg.msg_from_wire_to_plan != 0:
    #         self.msg_from_wire_to_plan = msg.msg_from_wire_to_plan


    # 已知经纬度求zone和utm坐标
    def convert_to_utm(self, latitude, longitude):
        wgs84 = pyproj.CRS('EPSG:4326')  # WGS84
        utm_zone = (math.floor((longitude + 180) / 6) % 60) + 1
        utm_epsg_code = 32600 + utm_zone  # Assuming northern hemisphere; for southern use 32700 + zone
        utm = pyproj.CRS(f'EPSG:{utm_epsg_code}')
        transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)
        utm_x, utm_y = transformer.transform(longitude, latitude)
        return utm_x, utm_y, utm_zone
    
    # 已知zone和utm坐标求经纬度
    def convert_from_utm(self, utm_x, utm_y, utm_zone, northern_hemisphere=True):
        utm_epsg_code = 32600 + utm_zone if northern_hemisphere else 32700 + utm_zone
        utm = pyproj.CRS(f'EPSG:{utm_epsg_code}')
        wgs84 = pyproj.CRS('EPSG:4326')  # WGS84
        transformer = pyproj.Transformer.from_crs(utm, wgs84, always_xy=True)
        longitude, latitude = transformer.transform(utm_x, utm_y)
        return latitude, longitude


    def v2x_data_callback(self, msg):
        data = json.loads(msg.data)
        roadpoints = data["roadPoints"] 

        self.end_point = [self.convert_to_utm(roadpoints[0]['latitude'],roadpoints[0]['longitude'])[0],self.convert_to_utm(roadpoints[0]['latitude'],roadpoints[0]['longitude'])[1]]
        self.turning_point1 = [self.convert_to_utm(roadpoints[1]['latitude'],roadpoints[1]['longitude'])[0],self.convert_to_utm(roadpoints[1]['latitude'],roadpoints[1]['longitude'])[1]]
        self.turning_point2 = [self.convert_to_utm(roadpoints[2]['latitude'],roadpoints[2]['longitude'])[0],self.convert_to_utm(roadpoints[2]['latitude'],roadpoints[2]['longitude'])[1]]

        self.v2x_turn_flag = True


    # def control_data_callback(self, msg):
    #     self.speed_map = msg.Target_velocity

    def path_publisher(self, x_list, y_list, x_list_pub, y_list_pub, startpoint):
        pub_path = rospy.Publisher("path_data", GlobalPathPlanningInterface, queue_size=10)
        rate = rospy.Rate(10)  # 1 Hz
        while not rospy.is_shutdown() :

            if len(self.x_list) != 0:

                # 直接发布tf
                br = tf.TransformBroadcaster()
                vehicle_position = (self.now_pos_x-self.startpoint[0], self.now_pos_y-self.startpoint[1], 0.0)
                heading = self.convert_deg_to_rad(self.now_pos_head)
                quaternion = quaternion_from_euler(0, 0, heading)
                br.sendTransform(vehicle_position,
                                            quaternion,
                                            rospy.Time.now(),
                                            "base_link",
                                            "map")

                msg = GlobalPathPlanningInterface()
                print("aaaaaaaaaaaaa")
                msg.x_list = self.x_list
                msg.y_list = self.y_list
                msg.x_list_l = self.x_list_l
                msg.y_list_l = self.y_list_l
                msg.startpoint = self.startpoint
                pub_path.publish(msg)
                rate.sleep()

    def convert_deg_to_rad(self, degrees):
        return (degrees + 180) * (2 * math.pi / 360)

    def main(self):
        signal.signal(signal.SIGINT, self.quit)
        rospy.init_node('global_path_planning')
        rospy.logwarn("plan start success")
        rospy.logwarn("**********需要先打开gps**********")
        pub = rospy.Publisher("global_path_planning_data", GlobalPathPlanningInterface, queue_size=10)
        rospy.Subscriber('gps_imu', GpsImuInterface, self.call_back_GPS)
        # rospy.Subscriber("car_wirecontrol", CarOriInterface, self.wire_data_callback)
        #TODO 接受v2x的信息选择全局路径

        rospy.Subscriber("obu_event_message", String, self.v2x_data_callback)
        pub_from_plan_to_decision = rospy.Publisher("plan_to_decision", ChangeMapPlanInterface, queue_size=10)

        # 可视化路径
        # pub_path = rospy.Publisher("path_data", GlobalPathPlanningInterface, queue_size=10)
        # msg = GlobalPathPlanningInterface()
        # # 可视化局部路径
        # msg.x_list = x_list
        # msg.y_list = y_list

        # msg.x_list_l = x_list_pub
        # msg.y_list_l = y_list_pub

        # pub_path.publish(msg)

        # 启动单独的线程用于路径发布
        # 发送可视化路径对的线程（主要是为了降低发送频率减小带宽）
        path_thread = threading.Thread(target=self.path_publisher, args=(self.x_list, self.y_list, self.x_list_l, self.y_list_l, self.startpoint))
        path_thread.start()

        msg = GlobalPathPlanningInterface()

        # self.mapfile = ""
        while not rospy.is_shutdown() and not self.gps_flag:
            rospy.sleep(0.1)

        while not rospy.is_shutdown():

            # 路径重置
            # x_list = []
            # y_list = []
            # head_list = []
            # curva_list = []
            # 有些不能重置，因为他们只计算一次。
            routedata = []
            # vel_list_pub = []
            # curva_list_pub = []
            # head_list_pub = []
            # startpoint = []
            x_list_pub = []
            y_list_pub = []

            start_time = time.time()

            # TODO map直接进行遍历寻找 okk
            # TODO 解析路径只有mapfile改变才改变 okk——控制频率
            # TODO 怎么从一条路切换到下一条路 这里不换挡还简单一些
            # mapfile = "/home/nvidia/sensor_ws/src/hmi/config/5.map"
            # mapfile = "/home/nvidia/sensor_ws/src/hmi/config/5.map"
            list1 = [0, 2, 4]
            list2 = [1, 3, 5]

            self.end_point = [self.convert_to_utm(39.17745732,117.45353548)[0],self.convert_to_utm(39.17745732,117.45353548)[1]]
            self.turning_point1 = [self.convert_to_utm(39.17768892,117.45540107000001)[0],self.convert_to_utm(39.17768892,117.45540107000001)[1]]
            self.turning_point2 = [self.convert_to_utm(39.18193462,117.44041031)[0],self.convert_to_utm(39.18193462,117.44041031)[1]]

            if self.current_mapfile != self.mapfile and self.num in list1:
                self.load_config("../../hmi/config/whole/")
                # self.load_config("../../hmi/config/map1/")
                # self.load_config("../../hmi/config/map2/")
                self.mapfile, min_distance = self.cal_road()

                # data_list = self.caculate_point(min_distance_txt)  # 格式：[[],[],[]]
                # msg.incppoint = data_list
                #test:
                # self.mapfile = "/home/nvidia/sensor_ws/src/hmi/config/test1.map"
                self.mapfile = "/home/nvidia/sensor_ws/src/hmi/config/618_1.map"

                x_list, y_list, head_list, vel_list, gear, action = self.readMap(self.mapfile)

                self.current_mapfile = self.mapfile

                # self.straight_end = [x_list[-1], y_list[-1]]

                dis = {}
                trajectory = [list(coord) for coord in zip(x_list, y_list)]
                try:
                    kdtree = KDTree(trajectory)
                except:
                    continue
                # curPos = [self.now_pos_x, self.now_pos_y]
                distances, indices = kdtree.query([self.end_point], k=1)
                mark_end = indices[0][0]
                mark_index = mark_end

                # 构造vector:
                
                vector_1 = [(self.end_point[0] - x_list[max(mark_index-1,0)]), (self.end_point[1] - y_list[max(mark_index-1,0)])]
                vector_2 = [(self.end_point[0] - x_list[min(mark_index + 1, len(x_list)-1)]), (self.end_point[1] - y_list[min(mark_index + 1, len(y_list)-1)])]

                # nearest_distance_lateral = (vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0]) / \
                #                             (math.sqrt((x_list[max(mark_index-1,0)] - x_list[min(mark_index + 1, len(x_list)-1)]) ** 2 + (
                #                                     y_list[max(mark_index-1,0)] - y_list[min(mark_index + 1, len(y_list)-1)]) ** 2))
                nearest_distance_lateral = distances[0][0]
                dis['s'] = (mark_index - self.mark) * 0.1
                dis['l'] = nearest_distance_lateral

            msg_plan_to_decision = ChangeMapPlanInterface()
            if dis['s'] < 15:
                # plan_to_decision
                msg_plan_to_decision.dspeed = -1.8
                msg_plan_to_decision.cte_offset = dis['l']
            if dis['s'] < 0.45:
                msg_plan_to_decision.Brake_Enable = 1
                msg_plan_to_decision.brake_pedel = 5

            pub_from_plan_to_decision.publish(msg_plan_to_decision)
            print("end_point","s", dis['s'], "l", dis['l'])


            # s l 值
            # s到某个值后
            # 然后把l给到cte。


            # if (self.end_road != 0):
            #     dis_end_road = self.euclidean_distance(self.end_road[0], self.end_road[1], self.now_pos_x, self.now_pos_y)
            #     if dis_end_road < 2:
            #         count1 = 1

            # if (self.straight_end != 0):
            #     dis_end_road = self.euclidean_distance(self.straight_end[0], self.straight_end[1], self.now_pos_x, self.now_pos_y)
            #     if dis_end_road < 150:
            #         count1 = 1


            # # 最小的距离
            # # 减速 and 换道（map自带换道）  map 自带减速
            # if self.dis_turn < 15 and self.num in list1 and :
            #     # msg_plan_to_decision = ChangeMapPlanInterface()
            #     # # 换道并减速。发给决策的plan层
            #     # msg_plan_to_decision.dspeed = -0
            #     # pub_from_plan_to_decision.publish(msg_plan_to_decision)
            #     self.num += 1
            #     self.mapfile = None
            #     # self.count2 = 1
            # # else:
            # #     msg_plan_to_decision.dspeed = 0
            # #     pub_from_plan_to_decision.publish(msg_plan_to_decision)

            # if self.dis_turn < 15 and self.num in list2 and count1 == 1:
            #     # msg_plan_to_decision = ChangeMapPlanInterface()
            #     # 换道并减速。发给决策的plan层
            #     # msg_plan_to_decision.dspeed = -0
            #     # pub_from_plan_to_decision.publish(msg_plan_to_decision)
            #     self.num += 1
            #     self.mapfile = None
            #     # self.count1 = 1
            # # else:
            # #     msg_plan_to_decision.dspeed = 0
            # #     pub_from_plan_to_decision.publish(msg_plan_to_decision)

            

            # mapfile = "/home/cyun/my_project/WIDC/bus_ws/src/hmi/config/5.map"
            print("mapfile",self.mapfile)
            print("min_distance", min_distance[0][0], type(min_distance))
            msg.roadid = self.mapfile
            # x_list, y_list, head_list, vel_list, gear, action = self.readMap(mapfile)
            
            curva_list = [0 for _ in x_list]
            if not x_list or not y_list:
                rospy.logerr("Map data is empty. Exiting.")
                return
            startpoint = [x_list[0], y_list[0]]
            head_list_pub = head_list
            vel_list_pub = vel_list
            curva_list_pub = curva_list
            msg.startpoint = startpoint
            self.start_point = startpoint
            self.startpoint = startpoint

            # x_list_pub = [0] * len(x_list)
            # y_list_pub = [0] * len(x_list)
            # for i in range(len(x_list)):
            #     x_list_pub[i] = x_list[i] - startpoint[0] # 每一个map的第一个点作为起点
            #     y_list_pub[i] = y_list[i] - startpoint[1]

            trajectory = [list(coord) for coord in zip(x_list, y_list)]
            try:
                kdtree = KDTree(trajectory)
            except:
                continue
            curPos = [self.now_pos_x, self.now_pos_y]
            distances, indices = kdtree.query([curPos], k=1)
            self.mark = indices[0][0]
            mark_index = self.mark
            
            # 发布的局部路径
            x_list_1 = x_list[max(0, mark_index - 20):min(len(x_list), mark_index + 200)]
            y_list_1 = y_list[max(0, mark_index - 20):min(len(x_list), mark_index + 200)]
            head_list_pub_1 = head_list_pub[max(0, mark_index - 20):min(len(x_list), mark_index + 200)]
            vel_list_pub_1 = vel_list_pub[max(0, mark_index - 20):min(len(x_list), mark_index + 200)]
            curva_list_pub_1 = curva_list_pub[max(0, mark_index - 20):min(len(x_list), mark_index + 200)]
            
            x_list_pub = [0] * len(x_list_1)
            y_list_pub = [0] * len(y_list_1)
            for i in range(len(x_list_1)):
                x_list_pub[i] = x_list_1[i] #- startpoint[0] # 每一个map的第一个点作为起点
                y_list_pub[i] = y_list_1[i] #- startpoint[1]

            for i in range(len(x_list_1)):
                routedata.append(x_list_1[i])
                routedata.append(y_list_1[i])
                routedata.append(head_list_pub_1[i])
                routedata.append(vel_list_pub_1[i])
                routedata.append(curva_list_pub_1[i])
            routedata = [str(num) for num in routedata]  # 用字符串保证小数不丢失

            # 可视化局部路径
            # msg.x_list = x_list
            # msg.y_list = y_list

            # msg.x_list_l = x_list_pub
            # msg.y_list_l = y_list_pub
            self.x_list = x_list
            self.y_list = y_list
            self.x_list_l = x_list_pub
            self.y_list_l = y_list_pub

            msg.routedata = routedata
            msg.startpoint = startpoint
            msg.endpoint = [x_list[-1], y_list[-1]]
            pub.publish(msg)
            # point = [x_list[mark_index], y_list[mark_index]] # mark点
            end_time = time.time()
            process_time = end_time - start_time
            print(int(1 / process_time), " hz")
            # rospy.Rate(200).sleep()

    def quit(self, signal, frame):
        rospy.signal_shutdown("Quit")

if __name__ == "__main__":
    node = GlobalPathPlanningNode()
    node.main()
