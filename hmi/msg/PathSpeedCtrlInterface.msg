float64  timestamp         # 数据帧时间戳
float32  Target_velocity    # 车辆速度m/s
float32  Target_steering_angle    # 车辆转角，方向盘转角 
uint8  Target_gear   # 01：驻车；02：空挡（N）；03：前进（D）；04：后退（R）；05：无效；
uint8  Target_primary_gear   # 01：低速，02：中速；03：高速；
uint8  Target_EngineSpeedLevel   # 01：低速；02：中速；03：高速；
float32  Target_speedlever_percentage   #车速手柄（采棉机）开度（-1000-1000）   对应【-100%, 100%】，负数为后退，正数为前进。
float32  Target_Torque_Nm   #目标驱动扭矩（单位：Nm，没有则给0）。
uint8  Target_braking_percentage   #制动行程请求，最大行程点125，最小行程点为0，单位为个（当前将行程分成125个点）
float32[] NearestPoint_Car2Path  # 反馈当前车辆位置距离规划的局部路径的最近点，包括：
# float   longitude        # 途径点经度（东向坐标，单位为：m）
# float   latitude         # 途径点纬度（北向坐标，单位为：m）
# float   speed         # 途径点速度（单位：m/s）


# add cyun 8.1
int8 Cotton_Bin_Ctrl #棉线动作使能


# add cyun 8.21
float32 CTE
float32 dHead

# add cyun 8.30
float32 brake_enable  #刹车使能
float32 value_brake   #刹车量

# add cyun 8.31
float32[]  flag # 标志位，控制给线控，表明发送成功


float32 raw_control # 


float32 kappa #采棉时转弯，提采头的判断

float64 x_trajectory
float64 y_trajectory

int8 gear # 档位 0 1-倒退 2 3-前进
int8 action # 0-关 1-取货 2-卸货