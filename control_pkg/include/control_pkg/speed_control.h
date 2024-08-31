#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <yaml-cpp/yaml.h>
#include <chrono>

struct WheelThroBrake{
    double speedTorque;
    double speedBrake;
};


class SpeedControl {
public:
    SpeedControl(double Speed_Kp_t, double Speed_Ki_t, double Speed_Kd_t,
                double Speed_Kp_b, double Speed_Ki_b, double Speed_Kd_b,
                double Acc_Kp_t, double Acc_Ki_t, double Acc_Kd_t,
                double Acc_Kp_b, double Acc_Ki_b, double Acc_Kd_b
                );
    WheelThroBrake calculateSpeedControl(double now_acc, double acc_from_plan_pre, double now_speed, double speed_ctrl_to_wire);

private:
    double integral_speed;
    double previous_speed_error;
    double integral_acc;
    double previous_acc_error;

    std::chrono::steady_clock::time_point last_time;
    bool first_call;
    
    // PID parameters for throttle and brake
    double Speed_Kp_t, Speed_Ki_t, Speed_Kd_t;
    double Speed_Kp_b, Speed_Ki_b, Speed_Kd_b;
    // ros::NodeHandle nh;
    double Acc_Kp_t, Acc_Ki_t, Acc_Kd_t;
    double Acc_Kp_b, Acc_Ki_b, Acc_Kd_b;

};

#endif // SPEED_CONTROL_H
