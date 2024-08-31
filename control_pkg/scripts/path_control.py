import time
import math
import os,sys,yaml

script_directory = os.path.dirname(os.path.abspath(__file__))
image_relative_path = 'lib'
icon_path = os.path.join(script_directory, image_relative_path)

sys.path.append(icon_path)
import prcolor
import prcolor

class PathControl:
    def __init__(self,yaml_data, L=3.8):
        self.integral_CTE = 0.0  # 积分误差的初始值
        self.last_time = None  # 上次调用时间的初始值
        self.last_CTE = 0
        self.last_dHead = 0
        self.yaml_data = yaml_data
        self.L = L

    def path_control(self, CTE, dHead, kappa, now_speed,yaml_data, dt=None):
        # 读取yaml参数
        CTE_Kp = yaml_data['CTE_Kp']
        CTE_Kd = yaml_data['CTE_Kd']
        Head_Kp = yaml_data['Head_Kp']
        Head_Kd = yaml_data['Head_Kd']

        if dt is None:
            current_time = time.time()
            if self.last_time is None:
                dt = 0.01  # 初始调用间隔
            else:
                dt = current_time - self.last_time
            self.last_time = current_time

        if dt is not None and dt > 0:
            diffCTE = (CTE - self.last_CTE) / dt
            diffdHead = (dHead - self.last_dHead) / dt
        else:
            diffCTE = 0.0
            diffdHead = 0.0
        self.last_CTE = CTE
        self.last_dHead = dHead
        
        # 微分限制
        if diffCTE > 0.2:
            diffCTE = 0.2
        if diffCTE < -0.2:
            diffCTE = -0.2

        # 微分限制
        if diffdHead > 0.2:
            diffdHead = 0.2
        if diffdHead < -0.2:
            diffdHead = -0.2

        # 车轮转角或者方向盘转角都可
        wheelAngle = int(CTE * CTE_Kp + diffCTE * CTE_Kd - dHead * Head_Kp - diffdHead * Head_Kd)
        # print("cte_angle",CTE * CTE_Kp + diffCTE * CTE_Kd,"dhead_angle",- dHead * Head_Kp - diffdHead * Head_Kd)

        if wheelAngle > 355:
            wheelAngle = 355
        if wheelAngle < -355:
            wheelAngle = -355
        # prcolor.prRed("dt",dt)
        # prcolor.prBlue("CTE={},dHead={},wheelAngle={}".format("{:.4f}".format(CTE), "{:.4f}".format(dHead), "{:.4f}".format(wheelAngle)))

        return wheelAngle
