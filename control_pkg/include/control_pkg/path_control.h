#ifndef PATH_CONTROL_H
#define PATH_CONTROL_H

#include <math.h>
#include <chrono>

class PathControl {
public:
    // PathControl();
    PathControl(double Kv, double Ki, double Kd,
                           double L, double maxWheelAngle, double WheelToSteering);
    double calculateSteeringAngle(double CTE, double dHead, double kappa, double now_speed);

private:
    double integral_CTE;
    double previous_CTE;
    std::chrono::steady_clock::time_point last_time;
    bool first_call;

    double Kv, Kd, L;
    double WheelToSteering, Ki, maxWheelAngle;
};

#endif // PATH_CONTROL_H
