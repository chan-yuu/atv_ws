// description:接收组合惯导消息，发布odom和imu数据
// time:2024.3.10
// cyun
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <car_interfaces/GpsImuInterface.h>
#include <color_print.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
double pos_x;
double pos_y;
double pos_z;
double yaw, AngleHeading;
double roll;
double pitch;
double vel_x;
double vel_y;
double vel_z;
double acc_x;
double acc_y;
double acc_z;
double x_gyro;
double y_gyro;
double z_gyro;


double DegToRad(double deg)
{
  double rad;
  double deg_new;
  if(deg < 0)
  {
    deg_new = -deg - 180;
    rad = deg_new * M_PI / 180;
  }
  else{
    deg_new = deg - 180;
  rad = (deg_new* M_PI / 180.0 );}
  return rad;
}

ros::Publisher odom_pub;
ros::Time current_time, last_time;

void gpsCallback(const car_interfaces::GpsImuInterface::ConstPtr &msg)
{
  pos_x = msg->posX;
  pos_y = msg->posY;
  // pos_z = msg->posZ;
  AngleHeading = msg->AngleHeading;
  yaw = DegToRad(AngleHeading);
  vel_x = msg->VelE;
  vel_y = msg->VelN;
  vel_z = msg->VelU;

  pitch = msg->pitch;
  roll = msg->roll;

  acc_x = msg->x_acc;
  acc_y = msg->y_acc;
  acc_z = msg->z_acc;
  x_gyro = msg->x_gyro;
  y_gyro = msg->y_gyro;
  z_gyro = msg->z_gyro;
  current_time = msg->header.stamp;
  // std::cout<<pos_x<<std::endl;
  // color_print::prRed(x_gyro,y_gyro,z_gyro, "\n");
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Subscriber gps_sub = n.subscribe("/gps_imu", 5, gpsCallback);

  // ros::Rate r(100);

  while (ros::ok())
  {
    ros::spinOnce();  // 先处理回调函数
    // current_time = ros::Time::now();
    if(pos_x != 0)
    {
      double process_time = last_time.toSec();
    
    // map odom 
    static tf2_ros::TransformBroadcaster tf_broadcaster_m;
    geometry_msgs::TransformStamped transformStamped_m;
    transformStamped_m.header.stamp = current_time;//ros::Time::now();
    transformStamped_m.header.frame_id = "map";
    transformStamped_m.child_frame_id = "odom";
    transformStamped_m.transform.translation.x = 0;
    transformStamped_m.transform.translation.y = 0;
    transformStamped_m.transform.translation.z = 0;
    tf2::Quaternion q_m;
    q_m.setRPY(0, 0, 0);
    transformStamped_m.transform.rotation = tf2::toMsg(q_m);
    tf_broadcaster_m.sendTransform(transformStamped_m);

    // odom base_footprint
    static tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = current_time;//ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = 604270 - pos_x;
    transformStamped.transform.translation.y = 4085089 - pos_y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation = tf2::toMsg(q);
    tf_broadcaster.sendTransform(transformStamped);

    // base_footprint base_link
    tf2_ros::StaticTransformBroadcaster static_broadcaster1;
    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.stamp = current_time;//ros::Time::now();
    transformStamped1.header.frame_id = "base_footprint";
    transformStamped1.child_frame_id = "base_link";
    transformStamped1.transform.translation.x = 0;
    transformStamped1.transform.translation.y = 0;
    transformStamped1.transform.translation.z = 0;
    tf2::Quaternion q1;
    q1.setRPY(0, 0, 0);
    transformStamped1.transform.rotation = tf2::toMsg(q1);
    static_broadcaster1.sendTransform(transformStamped1);

    // base_link rslidar
    tf2_ros::StaticTransformBroadcaster static_broadcaster2;
    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.stamp = current_time;//ros::Time::now();
    transformStamped2.header.frame_id = "base_link";
    transformStamped2.child_frame_id = "rslidar";
    transformStamped2.transform.translation.x = 0;
    transformStamped2.transform.translation.y = 0;
    transformStamped2.transform.translation.z = 2;
    tf2::Quaternion q2;
    q2.setRPY(0, 0, 0);
    transformStamped2.transform.rotation = tf2::toMsg(q2);
    static_broadcaster2.sendTransform(transformStamped2);

    // base_link imu
    tf2_ros::StaticTransformBroadcaster static_broadcaster3;
    geometry_msgs::TransformStamped transformStamped3;
    transformStamped3.header.stamp = current_time;//ros::Time::now();
    transformStamped3.header.frame_id = "base_link";
    transformStamped3.child_frame_id = "imu";
    transformStamped3.transform.translation.x = 0;
    transformStamped3.transform.translation.y = 0;
    transformStamped3.transform.translation.z = 1.2;

    transformStamped3.transform.rotation = tf2::toMsg(q1);
    static_broadcaster3.sendTransform(transformStamped3);

    // 发布imu
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu";
    imu.linear_acceleration.x = acc_x; // acc[0] * (-9.8);
    imu.linear_acceleration.y = acc_y; // acc[1] * (-9.8);
    imu.linear_acceleration.z = acc_z; // acc[2] * (-9.8);
    imu.angular_velocity.x = x_gyro; // gyo[0] * 3.1415926 / 180.0;
    imu.angular_velocity.y = y_gyro; // gyo[1] * 3.1415926 / 180.0;
    imu.angular_velocity.z = z_gyro; // yo[2] * 3.1415926 / 180.0;
    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    // imu_pub.publish(imu);
    // 发布odom数据

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time; // ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x=  604270 - pos_x;
    odom.pose.pose.position.y=  4085089 - pos_y;
    odom.pose.pose.position.z = 0.0;
    tf::Quaternion qua_o;
    qua_o.setRPY(0,0,yaw); // yaw的方向严重不对
    odom.pose.pose.orientation.x=qua_o.x();
    odom.pose.pose.orientation.y=qua_o.y();
    odom.pose.pose.orientation.z=qua_o.z();
    odom.pose.pose.orientation.w=qua_o.w();
    odom_pub.publish(odom);
    last_time = ros::Time::now();
    }
    // double process_time = (last_time-current_time).toSec();
    // color_print::prYellow("process_time", process_time, "\n");
    // color_print::prGreen("call gps success","\n");
    // r.sleep();
  }
  // ros::spin();
  return 0;
}