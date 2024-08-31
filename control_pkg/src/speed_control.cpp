#include "control_pkg/speed_control.h"
#include "ros/ros.h"
#include <control_pkg/color_print.h>

SpeedControl::SpeedControl(double Speed_Kp_t, double Speed_Ki_t, double Speed_Kd_t,
                           double Speed_Kp_b, double Speed_Ki_b, double Speed_Kd_b,
                           double Acc_Kp_t, double Acc_Ki_t, double Acc_Kd_t,
                           double Acc_Kp_b, double Acc_Ki_b, double Acc_Kd_b)
    : integral_speed(0.0), previous_speed_error(0.0), first_call(true), integral_acc(0.0),previous_acc_error(0.0),
    Speed_Kp_t(Speed_Kp_t), Speed_Ki_t(Speed_Ki_t), Speed_Kd_t(Speed_Kd_t),
    Speed_Kp_b(Speed_Kp_b), Speed_Ki_b(Speed_Ki_b), Speed_Kd_b(Speed_Kd_b), 
    Acc_Kp_t(Acc_Kp_t), Acc_Ki_t(Acc_Ki_t), Acc_Kd_t(Acc_Kd_t),
    Acc_Kp_b(Acc_Kp_b), Acc_Ki_b(Acc_Ki_b), Acc_Kd_b(Acc_Kd_b)  {}

WheelThroBrake SpeedControl::calculateSpeedControl(double now_acc, double acc_from_plan_pre, double now_speed, double speed_ctrl_to_wire) {
    WheelThroBrake wheelThrottleBrake;

    double max_Throttle = 100; //yaml_data["max_Throttle"].as<double>();
    double max_Brake = 100; //yaml_data["max_Brake"].as<double>();
    double min_Throttle = 0; //yaml_data["min_Throttle"].as<double>();
    double min_Brake = 0; //yaml_data["min_Brake"].as<double>();

    double diffspeed = now_speed - speed_ctrl_to_wire;
    // if (diffspeed<=0.2){diffspeed=0}
    diffspeed = std::max(std::min(diffspeed, 0.2), -0.2);


    double diffacc = now_acc - acc_from_plan_pre;
    double dt;
    double speedTorque;
    double speedBrake;


    if (first_call) {
        dt = 0.01; // 初始调用间隔
        first_call = false;
    } else {
        auto current_time = std::chrono::steady_clock::now();
        dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() / 1000.0;
    }
    last_time = std::chrono::steady_clock::now();

    // 只有运动时才有积分
    if (now_acc != 0) {
        integral_acc += diffacc * dt;
    }

    if (now_speed != 0) {
        integral_speed += diffspeed * dt;
    }

    // 计算速度误差的微分
    double derivative_speed_error = (diffspeed - previous_speed_error) / dt;
    previous_speed_error = diffspeed; // 更新上一次的速度误差

    double derivative_acc_error = (diffacc - previous_acc_error) / dt;
    previous_acc_error = diffacc; // 更新上一次的速度误差


    double integral_limit = 2; // 积分限制
    integral_speed = std::max(std::min(integral_speed, integral_limit), -integral_limit);

    // double integral_limit_ = 2; // 积分限制
    integral_acc = std::max(std::min(integral_acc, integral_limit), -integral_limit);

    // color_print::prBlue(diffacc, diffspeed);

    bool use_acc;
    use_acc = false;

    if (diffacc >= 0 && acc_from_plan_pre != 0&& use_acc) {
        speedBrake = (diffacc * Acc_Kp_b + integral_acc * Acc_Ki_b + derivative_acc_error * Acc_Kd_b);
        speedTorque = 0;
    }

    else if(diffacc < 0 && acc_from_plan_pre != 0&& use_acc){
        speedBrake = 0;
        speedTorque = -(diffacc * Acc_Kp_t + integral_acc * Acc_Ki_t + derivative_acc_error * Acc_Kd_t);
    }

    else {
        if (diffspeed > 0) {
            speedBrake = (diffspeed * Speed_Kp_b + integral_speed * Speed_Ki_b + derivative_speed_error * Speed_Kd_b);
            speedTorque = 0;
        } else {
            speedBrake = 0;
            // 初始有10
            speedTorque = 10-(diffspeed * Speed_Kp_t + integral_speed * Speed_Ki_t + derivative_speed_error * Speed_Kd_t);
        }
        
    } 


    // 限制输出值
    if (speedTorque >= max_Throttle) speedTorque = max_Throttle;
    if (speedTorque <= min_Throttle) speedTorque = min_Throttle;
    
    if (speedBrake >= max_Brake) speedBrake = max_Brake;
    if (speedBrake <= min_Brake) speedBrake = min_Brake;
    wheelThrottleBrake.speedBrake = speedBrake;
    wheelThrottleBrake.speedTorque = speedTorque;

    // color_print::prBlue("speedTorque",speedTorque);
    return wheelThrottleBrake;
}
