// main.cpp
#include "ros/ros.h"
#include "control_pkg/PathSpeedControlWP.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    PathSpeedControlWP pathSpeedControlWP(&nh);
    while (ros::ok()) {
        pathSpeedControlWP.Run();

        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown(); // 进入循环，等待回调函数
    return 0;
}
