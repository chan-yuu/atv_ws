import time

class SpeedControl:
    def __init__(self, yaml_data):
        self.integral_speed = 0.0  # 积分误差的初始值
        self.last_time = None  # 上次调用时间的初始值
        self.previous_speed_error = 0.0  # 上次速度误差的初始值
        self.yaml_data = yaml_data
        self.max_torque = 0

    def speed_control(self,wheel_angle, now_speed, speed_ctrl_to_wire,yaml_data, dt=None):
        # 刹车参数（高速）
        Speed_Kp_b_high = self.yaml_data['Speed_Kp_b_high']
        Speed_Kd_b_high = self.yaml_data['Speed_Kd_b_high']
        Speed_Ki_b_high = self.yaml_data['Speed_Ki_b_high']

        # 刹车参数（低速）
        Speed_Kp_b_low = self.yaml_data['Speed_Kp_b_low']
        Speed_Kd_b_low = self.yaml_data['Speed_Kd_b_low']
        Speed_Ki_b_low = self.yaml_data['Speed_Ki_b_low']

        # 油门参数（高速）
        Speed_Kp_high = self.yaml_data['Speed_Kp_high']
        Speed_Kd_high = self.yaml_data['Speed_Kd_high']
        Speed_Ki_high = self.yaml_data['Speed_Ki_high']

        # 油门参数（低速）
        Speed_Kp_low = self.yaml_data['Speed_Kp_low']
        Speed_Kd_low = self.yaml_data['Speed_Kd_low']
        Speed_Ki_low = self.yaml_data['Speed_Ki_low']

        # 根据speed_ctrl_to_wire的值判断使用高速还是低速参数
        if speed_ctrl_to_wire >= 4.9:
            Speed_Kp = Speed_Kp_high
            Speed_Kd = Speed_Kd_high
            Speed_Ki = Speed_Ki_high
            Speed_Kp_b = Speed_Kp_b_high
            Speed_Kd_b = Speed_Kd_b_high
            Speed_Ki_b = Speed_Ki_b_high
        else:
            Speed_Kp = Speed_Kp_low
            Speed_Kd = Speed_Kd_low
            Speed_Ki = Speed_Ki_low
            Speed_Kp_b = Speed_Kp_b_low
            Speed_Kd_b = Speed_Kd_b_low
            Speed_Ki_b = Speed_Ki_b_low

        diffspeed = now_speed - speed_ctrl_to_wire
        # print("now_speed",now_speed,"speed_ctrl_to_wire",speed_ctrl_to_wire)
        # 时间间隔的计算
        if dt is None:
            current_time = time.time()
            if self.last_time is None:
                dt = 0.01  # 初始调用间隔
            else:
                dt = current_time - self.last_time
            self.last_time = current_time

        # 积分项计算
        if now_speed > 0.1:
            self.integral_speed += diffspeed * dt

        # 微分项计算
        derivative_speed_error = (diffspeed - self.previous_speed_error) / dt
        self.previous_speed_error = diffspeed  # 更新上一次的速度误差
        derivative_limit = 6
        derivative_speed_error = max(min(derivative_speed_error, derivative_limit), -derivative_limit)

        # 积分限制
        integral_limit = 6
        self.integral_speed = max(min(self.integral_speed, integral_limit), -integral_limit)



        speedBrake = 0
        # 输出计算

        # 高速
        if speed_ctrl_to_wire >= 4.9:
            if diffspeed < 0.3 and diffspeed > -0.1:
                diffspeed = -0.1
                speedBrake = 0
                speedTorque = 10 - int(diffspeed * Speed_Kp + self.integral_speed * Speed_Ki + derivative_speed_error * Speed_Kd)
                #speedTorque = 0
            elif diffspeed > 0.3:
                speedBrake = 0# int(diffspeed * 4)
                speedTorque = 0

            elif diffspeed < 0:
                speedBrake = 0
                speedTorque = 12 - int(diffspeed * Speed_Kp + self.integral_speed * Speed_Ki + derivative_speed_error * Speed_Kd)
            # 起步：
            if diffspeed < -3.8:
                speedTorque = 20

        else:
            if diffspeed < 0.3 and diffspeed > -0.1:
                diffspeed = -0.1
                speedBrake = 0
                speedTorque = 12 - int(diffspeed * Speed_Kp + self.integral_speed * Speed_Ki + derivative_speed_error * Speed_Kd)
                #speedTorque = 0
            elif diffspeed > 0.3:
                speedBrake = 0#int(diffspeed * 2)
                speedTorque = 18


            elif diffspeed < 0:
                speedBrake = 0
                speedTorque = 12 - int(diffspeed * Speed_Kp + self.integral_speed * Speed_Ki + derivative_speed_error * Speed_Kd)
            # 起步：
            if diffspeed < -1.9:
                speedTorque = 20
            print("diffpeed ", diffspeed)
            print("speedTorque",speedTorque)
        if wheel_angle > 80:
            speedTorque += 2
        if wheel_angle > 160:
            speedTorque += 2
        if wheel_angle > 300:
            speedTorque += 2

        # 限制输出范围
        speedTorque = max(min(speedTorque, 30), 0)
        speedBrake = max(min(speedBrake, 60), 0)
        # print("speedTorque",speedTorque)
        # print("diffspeed",diffspeed)
        return speedTorque, speedBrake
