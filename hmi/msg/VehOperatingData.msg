# float64  timestamp  # 数据帧时间戳
# float32  cotton_picker_fan_Enabler    #  使能采头与风机，1-使能；0-保持不变；
# float32  cotton_bin_upper    # 升棉箱指令，1-提升；0-保持不变；
# uint8  cotton_bin_lower   # 降棉箱指令，1-降低；0-保持不变；
# uint8  cotton_bin_tilting   #  棉箱倾斜指令（需要持续作用，长按），1-倾斜；0-保持不变；
# uint8  cotton_bin_restore # 棉箱回正指令（需要持续作用，长按），1-回正；0-保持不变；
# uint8  cotton_unload  # 卸棉花指令（传送带），1-卸载；0-保持不变；
# float32  process_time    # 进程处理时间

# int8 operate# 0-不使能 1-使能
# int8 unloading # 0-不使能 1-使能

int32 target # 上升下降高度
int8 FY
int8 UpDown
int32 FY_h

# 10.17 cyun
int8 cotton_box_control   #0-不动作 1-棉箱倾斜 2-无动作 3-无动作 4-棉箱倾斜回正 
# int8 cotton_picker_enable       # 采头风机使能 0-不使能 1-使能 
int8 cotton_unloading         # 卸棉传送带  0-关闭 1-启动
# auto_enable= 1           #enable_single;
# speed_range= 0                  
# throttle_opening=0  #-100~100       
int8 Head_control    #1-采头下降 2-左采头上升（需要使能）3-右采头上升（需要使能） 4-采头同升（需要使能） 
int8 Engine_power     # 1-低速 2-中速 3-高速               
int8 Target_Fan_gear  # 采头风机 0-不接通 1-风机 2-采头风机
# Reserve_1=0       # 驻车信号 0-驻车 1-取消
# Target_steering_angle = 0 #方向盘转角
# brake_enable=0  #刹车使能
# brake_pressure_request=0 #刹车行程
# Reserve_2=0
# Target_turn_light=0
int8 Door_control    #门控 0-停止 1-开门 2-关门
# Reserve_3=0

int8 flag # 发送到线控成功

# operate = 0 # 采棉动作流程
# unloading = 0# 卸棉流程