std_msgs/Header header       # ros时间戳
int64  gps_time              # 时间戳
float64  AngRateRawX         # 角速度x
float64  AngRateRawY         # 角速度y
float64  AngRateRawZ         # 角速度z
float64  AccelRawX             # x轴加速度
float64  AccelRawY             # y轴加速度
float64  AccelRawZ             # z轴加速度
float64 PosESigma           #东向位置标准差
float64 PosNSigma          #北向位置标准差
float64 PosUSigma          #天向位置标准差
float64  VelESigma         # 东向速度标准差
float64  VelNSigma        # 北向速度标准差
float64  VelUSigma        # 天向速度标准差
float64  VelSigma          # 车辆速度标准差
float64  AccelX         # 车辆坐标系x轴加速度
float64  AccelY         # 车辆坐标系y轴加速度
float64  AccelZ         # 车辆坐标系z轴加速度
float64  AngleHeadingSigma         # 偏航角标准差
float64  AnglePitchSigma        # 俯仰角标准差
float64  AngleRollSigma         # 横滚角标准差
float64  AngRateX                  # 车辆坐标系x轴角速度
float64  AngRateY                  # 车辆坐标系y轴角速度
float64  AngRateZ                  # 车辆坐标系z轴角速度
int32  GpsNumSats2Used      #辅天线使用卫星数
int32  GpsNumSats                #主天线搜星数
int32  GpsNumSats2              #副天线搜星数
float64 Rear_posX                  #后轴中心点UTM坐标x
float64 Rear_posY                  #后轴中心点UTM坐标y
float64 Rear_posZ                  #后轴中心点UTM坐标z
float32  process_time    # 进程处理时间

