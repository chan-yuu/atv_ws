#!/usr/bin/env python3
#coding=utf-8


from scipy.spatial import cKDTree
from sklearn.neighbors import KDTree

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

import numpy as np
import math
import json

import rospy
from car_interfaces.msg import GlobalPathPlanningInterface
from car_interfaces.msg import  NetStartEndPointInterface,GpsImuInterface
from car_interfaces.msg import HmiStartEndPointInterface
from hmi.msg import NodePointsInterface


import time
import matplotlib.patches as patches

import pyproj
import signal
import threading
import matplotlib


import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import sys
import rospy
from queue import Queue
import sys
import threading
import random
import matplotlib.pyplot as plt
import warnings
from matplotlib import MatplotlibDeprecationWarning
warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)
from PyQt5.QtGui import QIcon, QPixmap
import matplotlib.pyplot as plt
import threading
import os

import threading
from tqdm import tqdm
import time

import yaml
import std_msgs.msg

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

def readMap(mapfile):
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

    return x_values,y_values,head_values,vel_values