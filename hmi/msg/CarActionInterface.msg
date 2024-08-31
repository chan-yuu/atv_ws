# topic car_action_data
int8 flag                       #标志位
int8 operate #0-不使能 1-使能
# 采头下降
# 风机打开 
# 中速档
# 风机采头打开  
# 高速档

# 卸棉动作（全体）启动按钮
int8 unloading # 0-不使能 1-使能
# 棉线倾斜
# 传送带  停留一段时间（按照装满来说是30s-1min吧）


# 其余动作按钮
int8 cotton_box_control 		 #0-不动作 1-棉箱倾斜 2-无动作 3-无动作 4-棉箱倾斜回正 
# int8 cotton_picker_enable      	 # 采头风机使能 0-不使能 1-使能 
int8 cotton_unloading                    # 卸棉传送带  0-关闭 1-启动
int8 Head_control    			 # 0-不动作 1-采头下降 2-左采头上升（需要使能）3-右采头上升（需要使能） 4-采头同升（需要使能）
int8 Engine_power     			 # 0-不动作 1-低速 2-中速 3-高速
int8 Target_Fan_gear 			 # 采头风机 0-不接通 1-风机 2-采头风机
int8 Door_control    			 #门控 0-停止 1-开门 2-关门
int8 wheel_steer_control                 #方向盘控制 0-关闭 1-开
