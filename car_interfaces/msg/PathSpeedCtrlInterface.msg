uint8 Parking_Req   # 驻车请求  （0：驻车  1：释放） 
uint8 IPC_En   # 使能信号 （0：未使能   1：使能）
uint8  Target_Gear    # 车辆档位信号（0x00： P 档；  0x01：倒档R；  0x02：N 档；  0x03：D 档/L档）
int16  TargetPedalOpen  #油门踏板开度： 取值0～100，最小计量单元1%
float32  Target_Angle    # 角度旋转到当前数值对应的角度 (-390°~+390°前轮方向盘转，正为左转)，0°为对应中点位置
uint8 Brake_En   # 刹车使能 （0：未使能   1：使能）
float32 Target_Travel   # 行程值 （有效值：0-60，制动为行程控制）
uint8  DippedLamp    # 近光灯 （0：关闭，1：打开）
uint8  TurnLamp    # 转向灯控制 （0x00：归位；  0x01：左转向；  0x02：右转向）
uint8  FarLamp   # 远光灯 （0：关闭，1：打开）
uint8  Horn     # 喇叭 （0：关闭，1：打开）
uint8  Car_OFF    # 关机 （0：关闭，1：打开）
uint8  AlarmLamp    # 双闪灯（APP接入） （0：关闭，1：打开）
uint8  OutLineLamp   # 轮廓灯 （0：关闭，1：打开）

float32 CTE
float32 dHead

float32  Target_velocity    # 车辆速度m/s
float32  now_speed    # 车辆速度m/s
