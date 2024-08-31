// time:3.28 
// cyun control from CICV2024, python->c++

// PathSpeedControlWP.cpp
// #include <control_pkg/speed_control.h>
#include "control_pkg/PathSpeedControlWP.h"
// #include <common_msgs/Control_Test.h>
#include <cmath>
#include <control_pkg/color_print.h>
#include "control_pkg/KDTree.h"
#include <ros/ros.h>
#include "control_pkg/NearestPointFinder.h"
#include <car_interfaces/PathSpeedCtrlInterface.h>
#include "control_pkg/KDTree.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


template <typename T>
T Clamp(const T value, T bound1, T bound2)
{
    if (bound1 > bound2) 
    {
        std::swap(bound1, bound2);
    }

    if (value < bound1) 
    {
        return bound1;
    } 
    else if (value > bound2) 
    {
        return bound2;
    }
    return value;
}


PathSpeedControlWP::PathSpeedControlWP(ros::NodeHandle* nodehandle): nh_(*nodehandle) {

    // 车辆相关信息
    end_dis = nh_.param("path_control/end_dis", 1.0);
    Kv = nh_.param("path_control/Kv", 2.0); //NOTE must double for default
    Ki = nh_.param("path_control/Ki", 1.0);
    Kd = nh_.param("path_control/Kd", 1.0);
    L = nh_.param("path_control/L", 1.6);
    maxWheelAngle = nh_.param("path_control/maxWheelAngle", 8.0);
    WheelToSteering = nh_.param("path_control/WheelToSteering", 1.05);
    
    // method1 速度控制接口，分为油门和制动的
    Speed_Kp_t = nh_.param("speed_control/Speed_Kp_t", 100);
    Speed_Ki_t = nh_.param("speed_control/Speed_Ki_t", 10);
    Speed_Kd_t = nh_.param("speed_control/Speed_Kd_t", 0);

    Speed_Kp_b = nh_.param("speed_control/Speed_Kp_b", 100);
    Speed_Ki_b = nh_.param("speed_control/Speed_Ki_b", 10);
    Speed_Kd_b = nh_.param("speed_control/Speed_Kd_b", 0);


    double Acc_Kp_t = nh_.param("acc_control/Acc_Kp_t", 1);
    double Acc_Ki_t = nh_.param("acc_control/Acc_Ki_t", 0.02);
    double Acc_Kd_t = nh_.param("acc_control/Acc_Kd_t", 0);

    double Acc_Kp_b = nh_.param("acc_control/Acc_Kp_b", 1);
    double Acc_Ki_b = nh_.param("acc_control/Acc_Ki_b", 0.02);
    double Acc_Kd_b = nh_.param("acc_control/Acc_Kd_b", 0);

    // method2 速度控制接口信息
    speed_pid_low_conf_.integrator_enable           = nh_.param("speed_pid_low_conf/integrator_enable", true);
    speed_pid_low_conf_.integrator_saturation_level = nh_.param("speed_pid_low_conf/integrator_saturation_level", 0.1);
    speed_pid_low_conf_.output_saturation_level     = nh_.param("speed_pid_low_conf/output_saturation_level", 0.25);
    speed_pid_low_conf_.kp  = nh_.param("speed_pid_low_conf/kp", 0.3);
    speed_pid_low_conf_.ki  = nh_.param("speed_pid_low_conf/ki", 0.1);
    speed_pid_low_conf_.kd  = nh_.param("speed_pid_low_conf/kd", 0.0);
    speed_pid_low_conf_.kaw = nh_.param("speed_pid_low_conf/kaw", 0.0);
    speed_pid_low_conf_.nts = nh_.param("speed_pid_low_conf/ts", 0.01);

    speed_pid_high_conf_.integrator_enable           = nh_.param("speed_pid_high_conf/integrator_enable", true);
    speed_pid_high_conf_.integrator_saturation_level = nh_.param("speed_pid_high_conf/integrator_saturation_level", 0.3);
    speed_pid_high_conf_.output_saturation_level     = nh_.param("speed_pid_high_conf/output_saturation_level", 0.0);
    speed_pid_high_conf_.kp  = nh_.param("speed_pid_high_conf/kp", 0.25);
    speed_pid_high_conf_.ki  = nh_.param("speed_pid_high_conf/ki", 0.3);
    speed_pid_high_conf_.kd  = nh_.param("speed_pid_high_conf/kd", 0.0);
    speed_pid_high_conf_.kaw = nh_.param("speed_pid_high_conf/kaw", 0.0);
    speed_pid_high_conf_.nts = nh_.param("speed_pid_high_conf/ts", 0.01);


    speedControl_ = std::make_shared<SpeedControl>(Speed_Kp_t, Speed_Ki_t, Speed_Kd_t, Speed_Kp_b, Speed_Ki_b, Speed_Kd_b, 
                                                    Acc_Kp_t, Acc_Ki_t, Acc_Kd_t, Acc_Kp_b, Acc_Ki_b, Acc_Kd_b);

    pathControl_ = std::make_shared<PathControl>(Kv, Ki, Kd,
                           L, maxWheelAngle, WheelToSteering);

    control_init_ = nh_.advertise<car_interfaces::PathSpeedCtrlInterface>("path_speed_tracking_data", 10); // 

    //2024 CICV
    // control_pub_ = nh_.advertise<common_msgs::Control_Test>("control_test", 10); //          
    // path_sub_ = nh_.subscribe("cicv_amr_trajectory", 10, &PathSpeedControlWP::pathCallback, this); // 
    // location_sub_ = nh_.subscribe("cicv_location", 10, &PathSpeedControlWP::GpsCallback, this); // 
    
    path_sub_ = nh_.subscribe("/run_hybrid_astar/searched_path", 10, &PathSpeedControlWP::pathCallback, this); // 混合A星不间断发送路径才行
    location_sub_ = nh_.subscribe("gps_imu", 10, &PathSpeedControlWP::GpsCallback, this); // 同上
    // location_sub_ = nh_.subscribe("gps_imu", 10, &PathSpeedControlWP::GpsCallback, this); // 同上
    global_path_sub_ = nh_.subscribe("/global_path_planning_data", 10, &PathSpeedControlWP::globalpathCallback, this);
    end_Brake = false;
    end_Brake_1 = false;
}


void PathSpeedControlWP::pathCallback(const nav_msgs::Path::ConstPtr& msg) 
{
    PathContent = *msg;
    path_msg = true;

    std::vector<PathItem> path_list_obj;
    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> heading_list;

    std::vector<double> routedata;
    if (!PathContent.poses.empty()) {
        for (const auto& pose : PathContent.poses) {
            // 提取路径点的姿态信息
            geometry_msgs::Quaternion orientation = pose.pose.orientation;

            // // 将四元数转换为欧拉角
            // tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
            // tf2::Matrix3x3 mat(quat);
            // double roll, pitch, yaw;
            // mat.getRPY(roll, pitch, yaw);

            tf::Quaternion quaternion;
            tf::quaternionMsgToTF(pose.pose.orientation, quaternion);
            double roll, pitch, yaw;
            tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 提取roll, pitch, yaw
            
            // 存储到routedata中
            routedata.push_back(pose.pose.position.x);
            routedata.push_back(pose.pose.position.y);
            x_list_.push_back(pose.pose.position.x);
            y_list_.push_back(pose.pose.position.y);
            heading_list.push_back(yaw);
            routedata.push_back(yaw);

            routedata.push_back(1); // 速度如何规划
            routedata.push_back(0);  // 曲率如何计算
            // routedata.push_back(0);  // 加速度如何规划

            // PathItem path_item(pose.pose.position.x, pose.pose.position.y, yaw, velocity, curvature);
            // path_list_obj.emplace_back(path_item);
        }
    // 转换标准搜索格式
    const int group_size = 5;
    for (size_t i = 0; i < routedata.size() / group_size; i++) {
        size_t start_index = i * group_size;
        size_t end_index = start_index + group_size;
        PathItem path_item(routedata[start_index], routedata[start_index + 1],
                        routedata[start_index + 2], routedata[start_index + 3],
                        routedata[start_index + 4]);
        path_list_obj.emplace_back(path_item);
    }
    }
    std::cout<<x_list_.size()<<std::endl;
}

void PathSpeedControlWP::globalpathCallback(const car_interfaces::GlobalPathPlanningInterface::ConstPtr& msg) 
{
    globalPathContent = *msg;
    path_msg = true;
}

void PathSpeedControlWP::GpsCallback(const car_interfaces::GpsImuInterface::ConstPtr& msg) 
{
    EgoContent = *msg;
    gps_msg = true;
}


void PathSpeedControlWP::Run(){

    WaitMsg();
    }


void PathSpeedControlWP::WaitMsg(){
    ros::Rate loopRate(100);
    //等待消息的接收
    if (!path_msg || !gps_msg) // and !camera_flag   
    {
        ROS_WARN("control start success");
        ROS_WARN("********waiting for plan********");
        ROS_WARN("*********waiting for gps*********");
        // color_print::prRed(path_msg, "\n");
        ros::Duration(0.1).sleep();
    }
    else
    {

        PathSearchOut output_search = SearchPathMark(EgoContent, PathContent);
        OutCaWheel CtrlWheelSpeed = calculate_wheel_angle_and_speed(output_search, EgoContent);
        //2024CICV
        // common_msgs::Control_Test msg_;
        // msg_.SteeringAngle = CtrlWheelSpeed.wheelAngle;
        // msg_.BrakePedal = CtrlWheelSpeed.brake;
        // msg_.ThrottlePedal = CtrlWheelSpeed.throttle;
        // //应对终点的转向问题：  remove3.27
        // // if (end_Brake_1)
        // // {
        // //     msg_.SteeringAngle = 0;//CtrlWheelSpeed.wheelAngle;
        // // }

        // // 应对终点的横纵向问题：
        // if (end_Brake)
        // {
        // msg_.BrakePedal = 100;
        // msg_.ThrottlePedal = 0;//CtrlWheelSpeed.throttle;
        // // msg_.SteeringAngle = 0;//CtrlWheelSpeed.wheelAngle;
        // }//CtrlWheelSpeed.brake;}
        // msg_.Gear = 4;
        
        // control_pub_.publish(msg_);
        // 发布控制话题数据
        car_interfaces::PathSpeedCtrlInterface msg_control;

        msg_control.CTE = CtrlWheelSpeed.CTE;
        msg_control.dHead = CtrlWheelSpeed.dHead;
        msg_control.Target_Torque_Nm = CtrlWheelSpeed.speedPlan;
        msg_control.Target_velocity = 0.6;//output_search.now_speed;
        //ins 给出的当前速度： remove
        // msg_control.x_trajectory = output_search.now_speed_ins;
        // 加速度偏差：
        // msg_control.x_list_ = CtrlWheelSpeed.accPlan;  //目标加速度
        // msg_control.y_list_ = output_search.acc;  //反馈加速度
        // 终点偏差：
        msg_control.Target_steering_angle = CtrlWheelSpeed.wheelAngle;
        msg_control.kappa = output_search.dis_end; // 到达终点信息
        control_init_.publish(msg_control);
        loopRate.sleep();
    }
}


PathSearchOut PathSpeedControlWP::SearchPathMark(car_interfaces::GpsImuInterface EgoContent, nav_msgs::Path PathContent) {
    // 
    PathSearchOut output_search_mark;
    // ROS_INFO("Received GPS data");

    //2024cicv
    // // Extract TrajectoryInfo
    // const perception_msgs::TrajectoryInfo& trajectory_info = PathContent.trajectoryinfo;
    // int32_t path_id = trajectory_info.path_id;
    // float total_path_length = trajectory_info.total_path_length;
    // float total_path_time = trajectory_info.total_path_time;
    // int8_t decision_type = trajectory_info.decision_type;
    // int8_t light_type = trajectory_info.light_type;
    // const std::vector<std::string>& lane_ids = trajectory_info.lane_ids;

    // // Extract TrajectoryPoints
    // const std::vector<perception_msgs::TrajectoryPoint>& trajectory_points = trajectory_info.trajectorypoints;
    // for (const auto& point : trajectory_points) {
    //     const perception_msgs::Point2D& position = point.position;
    //     float velocity = point.velocity;
    //     float heading = point.heading * 180 / M_PI; // + 180 lujin -180-180

    //     float curvature = point.curvature;
    //     float s = point.s;
    //     float a = point.a;
    //     float t = point.t;
    //     int8_t point_type = point.point_type;

    //     x_list_.push_back(position.x);
    //     y_list_.push_back(position.y);
    //     routedata.push_back(position.x);
    //     routedata.push_back(position.y);
    //     routedata.push_back(heading);
    //     routedata.push_back(velocity);
    //     routedata.push_back(curvature);
    //     routedata.push_back(a);

    //     PathItem path_item(position.x, position.y, heading, velocity, curvature, a);
    //     path_list_obj.emplace_back(path_item);
    // }


    // 处理混合A*接口
    std::vector<PathItem> path_list_obj;
    std::vector<double> x_list_;
    std::vector<double> y_list_;
    std::vector<double> routedata;
    if (!PathContent.poses.empty()) {
        for (const auto& pose : PathContent.poses) {
            // 提取路径点的姿态信息
            geometry_msgs::Quaternion orientation = pose.pose.orientation;

            // 将四元数转换为欧拉角
            // tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
            // tf2::Matrix3x3 mat(quat);
            // double roll, pitch, yaw;
            // mat.getRPY(roll, pitch, yaw);

            tf::Quaternion quaternion;
            tf::quaternionMsgToTF(pose.pose.orientation, quaternion);
            double roll, pitch, yaw;
            tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 提取roll, pitch, yaw
            
            // 存储到routedata中
            routedata.push_back(pose.pose.position.x);
            routedata.push_back(pose.pose.position.y);
            x_list_.push_back(pose.pose.position.x);
            y_list_.push_back(pose.pose.position.y);
            routedata.push_back(yaw);

            routedata.push_back(1); // 速度如何规划
            routedata.push_back(0);  // 曲率如何计算
            // routedata.push_back(0);  // 加速度如何规划

            // PathItem path_item(position.x, position.y, heading, velocity, curvature);
            // path_list_obj.emplace_back(path_item);
        }
    // 转换标准搜索格式
    const int group_size = 5;
    for (size_t i = 0; i < routedata.size() / group_size; i++) {
        size_t start_index = i * group_size;
        size_t end_index = start_index + group_size;
        PathItem path_item(routedata[start_index], routedata[start_index + 1],
                        routedata[start_index + 2], routedata[start_index + 3],
                        routedata[start_index + 4]);
        path_list_obj.emplace_back(path_item);
    }


    // //处理原采图规划接口：
    // std::vector<PathItem> path_list_obj;
    // float end_point;
    // bool flag_con;
    // bool mark;
    // std::vector<std::vector<float>> trajectory;
    // bool plan_flag;
    // // float insec_point;

    // std::vector<float> path_list; //= globalPathContent.routedata;
    // for (const std::string& num : globalPathContent.routedata) {
    // path_list.push_back(std::stof(num));
    // }
    // // path_list = std::vector<float>(path_list.begin(), path_list.end());
    // // color_print::prBlue("path_list", path_list.size());
    // // insec_point = globalPathContent.incppoint;
    // int gear_from_plan = globalPathContent.plan_over;
    // int action_from_plan = globalPathContent.action;
    // std::vector<PathItem> path_list_obj1;
    // int group_size = 5;

    // for (int i = 0; i < path_list.size() / group_size; ++i) {
    //     int start_index = i * group_size;
    //     int end_index = start_index + group_size;
    //     std::vector<float> group(path_list.begin() + start_index, path_list.begin() + end_index);
    //     PathItem path_item{
    //         group[0],
    //         group[1],
    //         group[2],
    //         group[3],
    //         group[4]
    //     };
    //     path_list_obj1.push_back(path_item);
    // }
    // // end_point = globalPathContent.endpoint;
    // path_list_obj = path_list_obj1;

    // std::vector<double> x_list_;
    // std::vector<double> y_list_;

    // for (const auto& point : path_list_obj) {
    //     //     const perception_msgs::Point2D& position = point.position;
    //     //     float velocity = point.velocity;
    //     //     float heading = point.heading * 180 / M_PI; // + 180 lujin -180-180

    //     //     float curvature = point.curvature;
    //     //     float s = point.s;
    //     //     float a = point.a;
    //     //     float t = point.t;
    //     //     int8_t point_type = point.point_type;

    //         x_list_.push_back(point.x);
    //         y_list_.push_back(point.y);}


    // // x_list_ = globalPathContent.x_list;
    // // y_list_ = globalPathContent.y_list;
    // // for (int i = 0; i < path_list.size(); i += group_size) {
    // //     x_list_.push_back(path_list[i]);
    // //     y_list_.push_back(path_list[i + 1]);
    // // }

    // trajectory.clear();
    // for (int i = 0; i < x_list_.size(); ++i) {
    //     trajectory.push_back({ x_list_[i], y_list_[i] });
    // }

    color_print::prRed("path_list_obj", path_list_obj.size());

    bool plan_flag = true;

    output_search_mark.path_list_obj = path_list_obj;
    // Construct the trajectory data structure
    // std::vector<std::vector<double>> trajectory;
    // for (size_t i = 0; i < x_list_.size(); i++) {
    //     trajectory.emplace_back(std::vector<double>{x_list_[i], y_list_[i]});
    // }
    KDTree tree(x_list_, y_list_);
    auto result = tree.findNearest(EgoContent.posX, EgoContent.posY);
    // std::cout << "Nearest index: " << result.first << ", Distance: " << result.second << std::endl;
    

    // 精准停车服务
    output_search_mark.dis_end = std::sqrt(std::pow((EgoContent.posX-456926.6232), 2) + std::pow((EgoContent.posY-4399561.805), 2));
    dis_end = output_search_mark.dis_end;
    
    if (output_search_mark.dis_end <= end_dis) end_Brake = true;
    if (output_search_mark.dis_end <= 1.5) end_Brake_1 = true;


    double velocity_x = EgoContent.VelE;
    double velocity_y = EgoContent.VelN;
    double now_speed = std::sqrt(std::pow(velocity_x, 2) + std::pow(velocity_y, 2));

    double accel_x = EgoContent.x_acc;
    double accel_y = EgoContent.y_acc;
    double now_acc = std::sqrt(std::pow(accel_x, 2) + std::pow(accel_y, 2));

    double wheel_angle;
    double speed_ctrl_to_wire;
    color_print::prRed("result.first", result.first, x_list_.size(), "\n");

    output_search_mark.index = result.first;
    output_search_mark.distance = result.second;
    output_search_mark.now_speed = now_speed;
    output_search_mark.acc = now_acc;

    return output_search_mark;
}
}

class Point {
public:
    Point(double x, double y, double head) : x(x), y(y), head(head) {}
    double x, y, head;
};


OutCaWheel PathSpeedControlWP::calculate_wheel_angle_and_speed(
    PathSearchOut output_search,
    car_interfaces::GpsImuInterface EgoContent
    ) {
    OutCaWheel outCaWheel;

    Point cur_pos(EgoContent.posX, EgoContent.posY, EgoContent.AngleHeading);  // 航向系统统一使用-180~180[deg] 然后转到ros中的-pi~pi[red]中   

    double now_speed = output_search.now_speed;
    int mark = static_cast<int>(output_search.index);
    double cur_dis = output_search.distance;

    Point mark_cood(output_search.path_list_obj[mark].x, output_search.path_list_obj[mark].y, output_search.path_list_obj[mark].head);
    std::tuple<double, double, double> point_m = {output_search.path_list_obj[mark].x, output_search.path_list_obj[mark].y, output_search.path_list_obj[mark].head};

    if (cur_dis > 99) {
        ROS_WARN("curDis is %f, please check map or gps", cur_dis);
    }

    Point ptB(0,0,0);
    int next_mark = std::min(mark + 2, static_cast<int>(output_search.path_list_obj.size() - 1));
    ptB = Point(output_search.path_list_obj[next_mark].x,
                output_search.path_list_obj[next_mark].y,
                output_search.path_list_obj[next_mark].head);

    double cur_pos_head = cur_pos.head;
    double path_head = output_search.path_list_obj[mark].head;
    // color_print::prRed("head", path_head,"\n",output_search.path_list_obj[mark].x, output_search.path_list_obj[mark].y, output_search.path_list_obj[mark].head, "\n");
    color_print::prRed("EgoContent", EgoContent.posX, EgoContent.posY, "B", output_search.path_list_obj[next_mark].x,
                output_search.path_list_obj[next_mark].y,
                "A", output_search.path_list_obj[mark].x,
                output_search.path_list_obj[mark].y, "\n");
    int perception_mark = 5;
    // 根据速度信息确定预瞄点
    if (now_speed <= 3.5) perception_mark = static_cast<int>(now_speed * 2.5);
    if (now_speed >= 7) perception_mark = static_cast<int>(now_speed * 1.2);
    if (now_speed >= 8) perception_mark = 13;
    if (now_speed >= 10.5) perception_mark = 9;
    
    size_t perception_index = std::min(static_cast<size_t>(mark + perception_mark), output_search.path_list_obj.size() - 1);

    double path_head_p = output_search.path_list_obj[perception_index].head;
    // double path_head_p = output_search.path_list_obj[std::min(mark + perception_mark, static_cast<size_t>(output_search.path_list_obj.size() - 1))].head;
    double dHead = cur_pos_head - path_head_p;
    // color_print::prRed("head", cur_pos_head,path_head_p);
    if (dHead < -180) {
        dHead += 360;
    } else if (dHead > 180) {
        dHead -= 360;
    }
    
    double dx, dy, Rx, Ry;
    dx = mark_cood.x - ptB.x;
    dy = mark_cood.y - ptB.y;
    Rx = cur_pos.x - ptB.x;
    Ry = cur_pos.y - ptB.y;
    double CTE = (Ry * dx - Rx * dy) / (std::sqrt((dx * dx + dy * dy)) + 0.0000001); //加上一个值防止除0
    color_print::prRed("CTE111", CTE);

    double kappa;//= 0.0; // 曲率:
    // 三点法计算曲率 
    double x1, x2, x3, y1, y2, y3;
    x1, y1 = output_search.path_list_obj[mark].x, output_search.path_list_obj[mark].y;
    if (mark+2 < output_search.path_list_obj.size()-1) {
        x2 = output_search.path_list_obj[mark+2].x;
        y2 = output_search.path_list_obj[mark+2].y;
    }
    else {
        x2 = output_search.path_list_obj[mark].x;
        y2 = output_search.path_list_obj[mark].y;    }

    if (mark+4 <= output_search.path_list_obj.size()-1) {
        x3 = output_search.path_list_obj[mark+4].x;
        y3 = output_search.path_list_obj[mark+4].y;
    }
    else {
        x3 = output_search.path_list_obj[mark].x;
        y3 = output_search.path_list_obj[mark].y;
    }


    // Calculate the center of the circle
    double denominator = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

    double x_center = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / denominator;
    double y_center = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / denominator;
    // Calculate the radius
    double r = std::sqrt((x1 - x_center) * (x1 - x_center) + (y1 - y_center) * (y1 - y_center));
    // Calculate curvature
    double curvature = 1 / r;
    // Determine the sign of the curvature
    double vector1[2] = {x2 - x1, y2 - y1};
    double vector2[2] = {x3 - x2, y3 - y2};
    double cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0];
    if (cross_product > 0) {
        curvature *= 1; // Curvature is negative
    } else {
        curvature *= -1; // Curvature is positive
    }

    kappa = curvature;

    int perception_mark_s = 3;
    if (now_speed>=7)
    {
        perception_mark_s = 5;
    }
    if (now_speed>=10.8)
    {
        perception_mark_s = 10;
    }
    else{perception_mark_s = 3;}


    double speed_ctrl_to_wire = output_search.path_list_obj[mark].speed; //当前目标速度
    size_t perception_index_v = std::min(static_cast<size_t>(mark + perception_mark_s), output_search.path_list_obj.size() - 1); 
    double speedPlanPre = output_search.path_list_obj[mark+perception_index_v].speed;// 预瞄目标速度
    double acc_from_plan = 0;//output_search.path_list_obj[mark].a;
    outCaWheel.accPlan = acc_from_plan;

    outCaWheel.speedPlanPre = speedPlanPre; //带预瞄
    // 预瞄加速度：
    double acc_from_plan_pre = 0;//output_search.path_list_obj[mark+3].a;
    double pitch_ = EgoContent.pitch;

    // color_print::prPink(end_dis, end_Brake, "\n");
    outCaWheel.speedPlan = speed_ctrl_to_wire;
    
    outCaWheel.wheelAngle = pathControl_->calculateSteeringAngle(CTE, dHead, kappa, now_speed);
    outCaWheel.CTE = CTE;
    outCaWheel.kappa = kappa;
    outCaWheel.dHead = dHead;

    // color_print::prBlue("CTE", CTE, "dHead", dHead, "now_speed", now_speed, "yaw_now", path_head_p, "\n");
    double accel_x = EgoContent.x_acc;
    double accel_y = EgoContent.y_acc;
    double now_acc = std::sqrt(std::pow(accel_x, 2) + std::pow(accel_y, 2));

    
    WheelThroBrake throttle_brake = speedControl_->calculateSpeedControl(now_acc, acc_from_plan_pre, now_speed, outCaWheel.speedPlanPre);
    outCaWheel.throttle = throttle_brake.speedTorque;
    outCaWheel.brake = throttle_brake.speedBrake;

    // 第二种查表方式计算：
    // method 2: start
    // double switch_speed = 5.0;
    // double v_erro_deadzone = 0.2;
    
    // if (now_speed <= switch_speed) 
    // {
    //     speed_pid_controller_.Init(speed_pid_low_conf_);
    // } else {
    //     speed_pid_controller_.Init(speed_pid_high_conf_);
    // }

    // double speed_controller_input = 0.0;
    // double speed_controller_input_limit = 2.0; // yaml
    // double speed_controller_input_limited = 0.0;

    // speed_controller_input_limited = speed_ctrl_to_wire - now_speed; //当前点的速度误差
    
    // // color_print::prRed(speed_controller_input_limited, "s", "\n");
    // // speed_controller_input_limited = Clamp(speed_controller_input, -speed_controller_input_limit,
    // //                                        speed_controller_input_limit);
    // // speed_controller_input_limited = Clamp(speed_controller_input_limited, -2, 2);

    // // 速度PID输出结果为加速度补偿
    // double acceleration_cmd_closeloop = 0.0;
    
    // // 给速度误差增加死区限制，防止油门刹车交替触发
    // if (abs(speed_controller_input_limited) < v_erro_deadzone)
    // {
    //     speed_controller_input_limited = 0;
    // }
    // // color_print::prRed(speed_controller_input_limited, "s0", "\n");

    // acceleration_cmd_closeloop = speed_pid_controller_.Control(speed_controller_input_limited);
    
    // // color_print::prRed(acceleration_cmd_closeloop, "s1", "\n");

    // double slope_offset_compensation = digital_filter_pitch_angle_.Filter(9.8 * std::sin(pitch_));

    // double acceleration_cmd = acceleration_cmd_closeloop + acc_from_plan_pre +   // 参考点加速度
    //                           slope_offset_compensation;
    // color_print::prGreen(acceleration_cmd_closeloop,acc_from_plan_pre,slope_offset_compensation);//slope_offset_compensation

    // // std::vector<std::pair<double, double>> accel{{0.3,10},{0.5,15},{0.8,20},{1,25},{1.3,30},{1.5,35},{1.7,40},{0.7,50},{0.7,60},{0.7,65},{0.7,70},{0.7,75},{0.7,80},{0.7,85},{0.7,90},{0.7,100}};	
    // std::vector<std::pair<double, double>> brk{{-5.714,100},{-4,80},{-2.857,50},{-2,40},{-1.42,33},{-1,28},{-0.66,25},{-0.5,20},{-0.33,13},{-0.2,9},{-0.1,5},{0,0}};
    // std::vector<std::pair<double, double>> accel{{0,0},{0.3,10},{0.5,15},{0.8,20},{1,25},{1.3,30},{1.5,35},{1.7,40},{1.7,45},{2.2,50},{2.75,60},{2.99,65},{3.23,70},{3.47,75},{3.71,80},{3.95,85},{4.19,90},{4.43,95},{4.67,100}};	

    // interpolation_throttle.Init(accel);
    // interpolation_brake.Init(brk);


    // throttleLookup.Init(accel);
    // brakeLookup.Init(brk);

    // double acceleration_lookup = acceleration_cmd;


    // // 油门、制动指令初始化
    // double brake_cmd = 0.0;
    // double throttle_cmd = 0.0;
    // //油门开度 限制 
    // int throttle_max = 100;
    // int start_up_throttle_ = 8;
    // // 查表
    // if(acceleration_lookup >= 0)
    // {
    //     int calculaate_throttle_ = throttleLookup.Lookup(acceleration_lookup) + start_up_throttle_;
    //     if(calculaate_throttle_ >= throttle_max)
    //     {
    //         calculaate_throttle_ = throttle_max;
    //     }
    //     if(calculaate_throttle_ <= 0)
    //     {
    //         calculaate_throttle_ = 0;
    //     }

    //     throttle_cmd = calculaate_throttle_;
    //     brake_cmd    = 0;
    // }
    // else
    // {
    //     throttle_cmd = 0;
    //     brake_cmd    = brakeLookup.Lookup(acceleration_lookup) - 10;

    //     if(brake_cmd >= 100)
    //     {
    //         brake_cmd = 100;
    //     }
    //     if(brake_cmd <= 0)
    //     {
    //         brake_cmd = 0;
    //     }
    // }

    // outCaWheel.throttle = throttle_cmd;
    // outCaWheel.brake = brake_cmd;
    //method 2 end;
    

    // 额外处理：
    // if (now_speed>=9.8)
    // {
    //     outCaWheel.throttle=outCaWheel.throttle + 20;
    //     outCaWheel.brake = outCaWheel.brake - 10;
    // }
    // else{
    // outCaWheel.throttle = throttle_brake.speedTorque;
    // outCaWheel.brake = throttle_brake.speedBrake;
    // }
    // // 即将到达终点：
    // if (dis_end <= 3) 
    // {
    // outCaWheel.brake = 0;
    // outCaWheel.throttle = 5;
    // }

    // color_print::prRed(speed_controller_input_limited,acceleration_lookup, "throttle:", outCaWheel.throttle, "brake:", outCaWheel.brake,"\n");
    outCaWheel.curdis = cur_dis;
    return outCaWheel;
}