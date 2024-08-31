// PathSpeedControlWP.h
#ifndef PATH_SPEED_CONTROL_WP_H
#define PATH_SPEED_CONTROL_WP_H

#include "ros/ros.h"
#include "std_msgs/String.h" // 根据实际需要包含适当的消息类型
#include "path_control.h"
#include "speed_control.h"
// #include <common_msgs/Control_Test.h>
// #include <perception_msgs/PerceptionLocalization.h>
// #include <perception_msgs/Trajectory.h>
#include <vector>
#include <cmath>
#include <algorithm>

#include <control_pkg/pid_controller.h>
#include "control_pkg/interpolation_1d.h"
#include "control_pkg/digital_filter.h"

#include <car_interfaces/GpsImuInterface.h>
#include <car_interfaces/PathSpeedCtrlInterface.h>
#include <nav_msgs/Path.h>

#include <control_pkg/find_throttle_brake.h>
#include <car_interfaces/GlobalPathPlanningInterface.h>


class PathItem {
public:
    PathItem(double x, double y, double head, double speed, double curva)
        : x(x), y(y), head(head), speed(speed), curva(curva) {}

    double x;
    double y;
    double head;   // [deg]
    double speed;
    double curva;
};

struct PathSearchOut {
    std::vector<PathItem> path_list_obj;
    int index;
    double distance;
    double now_speed;
    double now_speed_ins;
    double acc;
    double dis_end;
};

struct OutCaWheel {
    double wheelAngle;
    double throttle;
    double brake;
    double speedPlan;

    double speedPlanPre;
    double CTE;
    double kappa;
    double dHead;
    double curdis;
    double accPlan;


};

class PathSpeedControlWP {
public:
    PathSpeedControlWP(ros::NodeHandle* nodehandle);
    void Run();
private:
    ros::NodeHandle nh_;
    ros::Publisher control_pub_; // 控制命令的发布者
    ros::Subscriber path_sub_; // 路径的订阅者
    ros::Subscriber location_sub_;
    ros::Subscriber global_path_sub_;

    ros::Publisher control_init_; // 

    std::shared_ptr<PathControl> pathControl_;
    // SpeedControl speedControl_;
    // std::unique_ptr<SpeedControl> speedControl_;
    std::shared_ptr<SpeedControl> speedControl_;
    
    // 订阅话题
    void pathCallback(const nav_msgs::Path::ConstPtr& msg); // 根据实际消息类型进行更改
    void GpsCallback(const car_interfaces::GpsImuInterface::ConstPtr& msg); // 根据实际消息类型进行更改
    void WaitMsg();

    void globalpathCallback(const car_interfaces::GlobalPathPlanningInterface::ConstPtr& msg); 


    PathSearchOut SearchPathMark(car_interfaces::GpsImuInterface EgoContent, nav_msgs::Path PathContent);
    
    // 发布话题
    void PublishControlCommand(car_interfaces::PathSpeedCtrlInterface msg);
    
    OutCaWheel calculate_wheel_angle_and_speed(
    PathSearchOut output_search,
    car_interfaces::GpsImuInterface EgoContent
    );

    bool path_msg = false;
    bool gps_msg = false;
    car_interfaces::GpsImuInterface EgoContent;
    nav_msgs::Path PathContent;
    car_interfaces::GlobalPathPlanningInterface globalPathContent;

    // New member variables for SpeedControl parameters
    double Speed_Kp_t;
    double Speed_Ki_t;
    double Speed_Kd_t;
    double Speed_Kp_b;
    double Speed_Ki_b;
    double Speed_Kd_b;

    double Acc_Kp_t;
    double Acc_Ki_t;
    double Acc_Kd_t;
    double Acc_Kp_b;
    double Acc_Ki_b;
    double Acc_Kd_b;

    double Kv, Kd, L;
    double WheelToSteering, Ki, maxWheelAngle;

    bool end_Brake;
    bool end_Brake_1;
    double end_dis;
    double dis_end;

    control::PIDController speed_pid_controller_;
    control::PidConf speed_pid_low_conf_;
    control::PidConf speed_pid_high_conf_;

    control::Interpolation1D interpolation_throttle;
	control::Interpolation1D interpolation_brake;
    control::DigitalFilter digital_filter_pitch_angle_;

    // speedcontrol寻找插值的开度
    ThrottleBrakeLookup throttleLookup;
    ThrottleBrakeLookup brakeLookup;

    //calculate acc: remove3.28. cyun
    double previous_velocity_x_;
    double previous_velocity_y_;
    ros::Time previous_time_;
    double acceleration;

};

#endif // PATH_SPEED_CONTROL_WP_H
