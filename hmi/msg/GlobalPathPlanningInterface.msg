float64  timestamp  #时间戳
float64[]  startpoint   #起点位置
float64[]   endpoint    #终点位置
string[]   routedata   #路径集合（所有途径点的集合）
float32  process_time    # 进程处理时间
int32 Global_Path_Segment_Index # 从1开始累加

int32 len_path # 全局路径的长度

#int8 Global_Path_End_Flag #终点置1，其他0
float32[]   last_point

int8 plan_over # gear 1-倒 3-前
int8 action # 取货动作 0-关 1-取货 2-卸货


float64[] x_list
float64[] y_list

string roadid #全局路径的地址，主要是给decision的
float64[]  incppoint  #路口信息集地址
