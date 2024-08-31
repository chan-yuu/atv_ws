#include "control_pkg/path_control.h"
#include <control_pkg/color_print.h>


PathControl::PathControl(double Kv, double Ki, double Kd,
                           double L, double maxWheelAngle, double WheelToSteering) 
: integral_CTE(0.0), previous_CTE(0.0), first_call(true),
Kv(Kv), Ki(Ki), Kd(Kd), L(L), maxWheelAngle(maxWheelAngle), WheelToSteering(WheelToSteering) {}


double PathControl::calculateSteeringAngle(double CTE, double dHead, double kappa, double now_speed) {
    
    // double Kv = yaml_data["Kv"].as<double>();
    // double Ki = yaml_data["Ki"].as<double>();
    // double L = yaml_data["L"].as<double>();
    // double integral_limit =  yaml_data["integral_limit"].as<double>();

    // double Kv = 1.8; //yaml_data["Kv"].as<double>();
    // double Ki = 0.15; //yaml_data["Ki"].as<double>();
    // double Kd = 0.08; // 微分增益
    

    // double L = 3.8; //yaml_data["L"].as<double>();
    double integral_limit = 5; //yaml_data["integral_limit"].as<double>();
    double integral_limit2 = 1;
    double sita = atan(L * kappa);
    sita = 0;
    double dt;

    if (first_call) {
        dt = 0.01; // 初始调用间隔
        first_call = false;
    } else {
        auto current_time = std::chrono::steady_clock::now();
        dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() / 1000.0;
    }
    last_time = std::chrono::steady_clock::now();

    if (now_speed != 0) {
        integral_CTE += CTE * dt;
    }
    // double integral_limit = 2; // 积分限制
    integral_CTE = std::fmax(std::fmin(integral_CTE, integral_limit), -integral_limit);
    
    // 计算微分项 如果不动微分项不会积累
    double derivative_CTE = (CTE - previous_CTE) / dt;
    previous_CTE = CTE; // 更新上一次的横向误差

    previous_CTE = std::fmax(std::fmin(previous_CTE, integral_limit2), -integral_limit2);
    double Target_wheel_angle = ((-dHead / 180 * M_PI) + atan(CTE * Kv / std::fmax((now_speed + 0.0001), 1.0) + sita) + Ki * integral_CTE +  Kd * derivative_CTE) * 180 / M_PI;

    // 根据需要调整角度限制
    double max_wheel_angle = maxWheelAngle;//18.62; //yaml_data["WheelAngle"].as<double>();
    if (Target_wheel_angle > max_wheel_angle) Target_wheel_angle = max_wheel_angle;
    if (Target_wheel_angle < -max_wheel_angle) Target_wheel_angle = -max_wheel_angle;
    
    double wheel_to_steering_angle = WheelToSteering;//29; //yaml_data["WheelToSteering"].as<double>();
    double Target_steering_angle = Target_wheel_angle * wheel_to_steering_angle; //* 29; // 根据实际比例调整
    color_print::prGreen("CTE_angle", atan(CTE * Kv / std::fmax((now_speed + 0.0001), 1.0) + std::abs(sita))* 180 / M_PI, "dHead_angle", -(dHead), "integ",(Ki * integral_CTE)* 180 / M_PI,"\n");
    color_print::prBlue("kappa", kappa, "CTE", CTE, "dHead", dHead, "Target_wheel_angle",Target_steering_angle,"\n");
    return Target_steering_angle;
}
