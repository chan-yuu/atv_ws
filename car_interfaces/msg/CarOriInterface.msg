# warn cyun 8.24 协议和msg的名称最好对应，不然程序写起来比较麻烦
# 另外，最好是根据协议来写msg，不然直接写msg，有些数据类型是有问题的

uint8  Drive_Mode    # 驾驶模式 (0：手动控制模式（油门踏板+挡位） 1：自动模式（线控）   2：遥控器调试模式)
uint8  Gear   # 档位 （0x00： P 档；0x01：倒档R；0x02：N 档；0x03：D 档）
uint8  EPOSts   # 急停触发状态 （0X00：未触发，0x01：遥控器触发；0x02：硬线触发；）
uint8  YK_H   # 复位键H （0：未触发  1：触发）
uint8  YK_F   # 遥控F键 （0：未触发  1：触发）
float32  Angle   # 当前角度 
uint8  EPB_SystemState   # EPB开关状态 （0:已释放 1：已施压 2：释放中 3：故障（停止或不确定状态） 4：施压中 5：脱离）
float32  Car_Speed   # 车速 （有效值范围：0～2200（表示0 km/h～220 km/h），最小计量单元：0.1km/h，“0xFF,0xFE”表示异常，“0xFF,0xFF”表示无效）
float32  Motor_Torque   # 发动机扭矩

uint16 Fault1   # 故障代码1（见故障列表,共150项）
uint16 Fault2   # 故障代码2
uint16 Fault3   # 故障代码3
uint16 Fault4   # 故障代码4
float32 Mileage   # 累计里程（有效值范围：0～9999999（表示0km～999999.9km），最小计量单元：0.1km。）

float32  Brake_Pressure   # 制动压力采样值 （精度为0.05MPa,偏移量为0，范围为0~10MPa）
float32  LR_WheelSpeed   # 左后轮速度
float32  RR_WheelSpeed # 右后轮速度
uint8  SOC   # 电池当前SOC（0-100）
uint8  CarSts1   # 车辆状态1 （0x00： 正常 ；0x01：一级报警  ；0x02：二级报警  ；0x03：三级报警）
uint8  CarSts2   # 车辆状态2 （0x00： READY ；0x01：急停状态 ；）

uint16  Fault   # 整车故障（见故障列表）
uint16  VCU_Sts   # VCU状态 （见VCU列表）
uint8  CarStartState   # 车辆启动状态 （0未知；1启动；2关机）
float32  VCU_Service_Voltage   # VCU供电电压 （VCU供电电压，单位mv）

uint8  LV_Main   # 主版本（样例V1.2.1）
uint8  LV_Sun   # 子版本
uint8  LV_zCode   # 修订版本
uint32  Time   # 日期 （样例：23020801  表示23年2月8日第1版程序）

