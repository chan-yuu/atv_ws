# warn cyun 8.24 协议和msg的名称最好对应，不然程序写起来比较麻烦
# 另外，最好是根据协议来写msg，不然直接写msg，有些数据类型是有问题的

float64  timestamp  #时间戳
uint8  id   # 车辆ID
uint8 auto_enable #无人驾驶使能 0-不使能   1-使能
float32  carspeed    # 车辆速度，单位为m/s
float32  steerangle    # 车辆转角，左转为正，右转为负
float32 current_angle # 转角？车轮缺
int8  gearpos    # 车辆档位信号（01：驻车，02：空挡（N），03：前进（D），04：后退（R），05：无效）
float32  brake_pressure_request    # 制动行程请求（0-125）
uint8  brake_enable    # 制动使能（0：不使能  1：制动使能）
uint8  current_braking_percentage  #制动踏板开度： 取值0～125
uint8  soc    # 电池当前SOC（0-100）
uint8  batteryvol    # 电池电压（0-55V）
uint16  batterydischargecur    # 电池放电电流（0-100A）
uint8  car_run_mode   #车辆运行模式： 0： ACU控制  1：自动驾驶  2： 急停模式
int16  throttle_percentage  #油门踏板开度： 取值0～100
uint8  left_light    #左转向灯状态：0：关闭，1：打开
uint8  right_light    #右转向灯状态：0：关闭，1：打开
uint8  reversing_light   #倒车灯状态：0：关闭，1：打开
uint8  speaker     #喇叭状态：0：关闭，1：打开
uint8  start_button     #启动按钮状态：0：按键无效，1：按键有效
uint8  stop_button     #急停按钮状态：0：按键无效，1：按键有效
uint8  state          # 设备状态，0：状态正常，1：电池箱报警；2：电机控制器报警
uint8  error          # 错误码；电池箱报警：1：单体过压或欠压，2：放电电流异常，3：电压报警，4：电池温度报警，5：电池SOC过低。电机控制器报警：1：转向电机控制器故障，2：驱动电机控制器故障
float32  cotton_picker_fan_Enabler_act    #  使能采头与风机，1-使能；0-保持不变；
float32  cotton_bin_upper_act    # 升棉箱实际状态，1-提升；0-保持不变；
uint8  cotton_bin_lower_act   # 降棉箱实际状态，1-降低；0-保持不变；
uint8  cotton_bin_tilting_act   #  棉箱倾斜实际状态，1-倾斜；0-保持不变；
uint8  cotton_bin_restore_act # 棉箱回正实际状态，1-回正；0-保持不变；
uint8  cotton_unload_act  # 卸棉花实际状态（传送带），1-卸载；0-保持不变；
float32  cotton_picker_head_height  # 棉花采头高度，单位：m；
uint8  Target_primary_gear_act   # 01：低速，02：中速；03：高速；
uint8  Target_EngineSpeedLevel_act   # 01：低速；02：中速；03：高速；
int16  Target_speedlever_percentage_act   #车速手柄开度（-100%-100%）   取值【-100, 100】，负数为后退，正数为前进。
float32  process_time    # 进程处理时间


float32 raw_controlled


int8 wait_wirectrl# 0-等待 1-等待结束 发送给hmi界面
