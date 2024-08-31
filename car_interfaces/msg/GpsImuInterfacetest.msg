string nav_flag #定位无效-0  单点解-1  差分定位-2  RTK固定解-4  RTK浮点解-5 递推-6

float32 x_acc #车辆坐标系
float32 y_acc 
float32 x_gyro #车辆坐标系
float32 y_gyro
float32 z_acc
float32 z_gyro

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
# float32 heading_g

std_msgs/Header header       # ros时间戳
float64  gps_time              #时间戳

#float64 lat
#float64 lon

float64 east_speed
float64 north_speed
float64 up_speed
float64 alt
float64 heading

float64 east_speed_g
float64 north_speed_g
float64 up_speed_g
float64 alt_g
float64 heading_g

int32  system_state                #惯导模式（0-初始化；1-卫导模式；2-组合导航模式；3-纯惯导模式）
int32  GpsNumSatsUsed         #主天线使用卫星数
int32  satellite_status              #卫星状态（0-不定位不定向；1-单点定位定向；   2-伪距差分定位定向；3-组合推算；4-RTK稳定解定位定向；5-RTK浮点解定位定向；6-单点定位不定向； 7-伪距差分定位不定向；8-RTK稳定解定位不定向；9-RTK浮点解定位不定向）
float64  GpsAge                     #差分延时
bool  state                      #设备状态（0-状态异常；1-状态正常）
float32  process_time        #进程处理时间
bool Gps_Cfault  #惯导通信故障


float32 posX
float32 posY
float32 posZ
float32 pitch
float32 roll
float32 yaw
float32 vel
float32 lat
float32 lon
bool state_ndt
