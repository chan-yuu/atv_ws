string nav_flag #定位无效-0  单点解-1  差分定位-2  RTK固定解-4  RTK浮点解-5 递推-6

float64 x_acc
float64 y_acc
float64 x_gyro
float64 y_gyro
float64 z_acc
float64 z_gyro

float64 acc
float64 gps_week
float64 gps_ms

string nav_flag_g #无效解	0
# #单点解	1
# #差分解	2
# #浮点解	5
# #窄固定解（narrow int）	4
# #其它固定解	7
# #ID:0X51B gnss航向
# float64 heading_g

std_msgs/Header header       # ros时间戳
int64  gps_time              #时间戳

float64  PosLon        # 经度
float64  PosLat         # 纬度
float64  PosAlt         # 高度

float64 lat_g
float64 lon_g

float64 east_speed_g
float64 north_speed_g
float64 up_speed_g
float64 alt_g
float64 heading_g

float64  VelE         # 东向速度
float64  VelN        # 北向速度
float64  VelU        # 天向速度
float64  Vel          # 车辆速度

float64  AngleHeading         # 偏航角（-180-180）  #需要这个范围
float64  AnglePitch        # 俯仰角（-90-90）
float64  AngleRoll         # 横滚角（-180-180）

int32  system_state                #惯导模式（0-初始化；1-卫导模式；2-组合导航模式；3-纯惯导模式）
int32  GpsNumSatsUsed         #主天线使用卫星数
int32  satellite_status              #卫星状态（0-不定位不定向；1-单点定位定向；   2-伪距差分定位定向；3-组合推算；4-RTK稳定解定位定向；5-RTK浮点解定位定向；6-单点定位不定向； 7-伪距差分定位不定向；8-RTK稳定解定位不定向；9-RTK浮点解定位不定向）
float64  GpsAge                     #差分延时
bool  state                      #设备状态（0-状态异常；1-状态正常）
float64 posX                    #定位点UTM坐标x
float64 posY                    #定位点UTM坐标y
float64 posZ # 高程
float64  process_time        #进程处理时间
bool Gps_Cfault  #惯导通信故障


float64 x
float64 y
float64 z
float64 pitch
float64 roll
float64 yaw
float64 vel
float64 lat
float64 lon
bool state_ndt
