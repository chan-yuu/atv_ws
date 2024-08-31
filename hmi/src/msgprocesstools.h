#ifndef MSGPROCESSTOOLS_H
#define MSGPROCESSTOOLS_H
#include <iostream>
#include "hmi/HmiStartEndPointInterface.h"
//#include "hmi/ParamOptimizeInterface.h"
//#include "hmi/FaultDiagnosisInterface.h"

const int font_size =9;
const int default_value = 0;
/***********************************指令下发****************************************/
struct HmiStartEndPointEvent
{
    double speed;                           //作业车速
    double point1_line_lo;                  //直线采集点1
    double point1_line_la;
    double point2_line_lo;                  //直线采集点2
    double point2_line_la;
    double point3_line_lo;                  //直线采集点3
    double point3_line_la;
    double point4_line_lo;                  //直线采集点4
    double point4_line_la;
    double point5_line_lo;                  //直线采集点5
    double point5_line_la;
    double point6_line_lo;                  //直线采集点6
    double point6_line_la;
    double point7_line_lo;                  //直线采集点7
    double point7_line_la;
    double point8_line_lo;                  //直线采集点8
    double point8_line_la;
    double point1_turn_lo;                  //掉头采集点1
    double point1_turn_la;
    double point2_turn_lo;                  //掉头采集点2
    double point2_turn_la;
    int left_rectify;                       //向左纠偏
    int right_rectify;                      //向右纠偏
    int left_direction;                     //航向向左
    int right_direction;                    //航向向右
    int planning_line;                      //规划直线
    int planning_turn;                      //规划掉头

    int restart;                            //界面重启
    int exp_start;                          //实验测试按钮
    int emergency_stop;                     //紧急停车按钮

};

double CaculateLongitudeDMS2Double(const int degree,const int minute,const double second);
std::vector<double> CaculateLongitudeDouble2DMS(const double longitude);
double CaculateLatitudeDMS2Double(const int degree,const int minute,const double second);
std::vector<double> CaculateLatitudeDouble2DMS(const double latitude);
hmi::HmiStartEndPointInterface CreateHmiStartEndPointMsg(const struct HmiStartEndPointEvent hmiStartEndPointEvent);
/**********************************************************************************/

/***********************************参数优化****************************************/
struct ParamOptimizeEvent
{
    double kp_v;                 //循迹控制中的反馈通道的比例系数
    double kp_kapa;              //循迹控制中的曲率前馈通道的比例系数
    double b0_spdcrl;            //车速控制中的控制输入增益
    double wc_spdcrl;            //车速控制中的控制器带宽
};
//hmi::ParamOptimizeInterface CreateParamOptimizeMsg(const struct ParamOptimizeEvent paramOptimizeEvent);
/**********************************************************************************/

/***********************************故障检测****************************************/

/**********************************************************************************/
#endif // MSGPROCESSTOOLS_H
