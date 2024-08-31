#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>
#include <iostream>

// 标定参数结构体
struct CalibrationParams
{
  double x_offset;
  double y_offset;
  double z_offset;
  double roll_offset;
  double pitch_offset;
  double yaw_offset;
};

ros::Publisher calibrated_pub;
CalibrationParams calibration_params;

void loadCalibrationParams(const ros::NodeHandle& nh)
{
  nh.getParam("calibration/x_offset", calibration_params.x_offset);
  nh.getParam("calibration/y_offset", calibration_params.y_offset);
  nh.getParam("calibration/z_offset", calibration_params.z_offset);
  nh.getParam("calibration/roll_offset", calibration_params.roll_offset);
  nh.getParam("calibration/pitch_offset", calibration_params.pitch_offset);
  nh.getParam("calibration/yaw_offset", calibration_params.yaw_offset);

  // 打印加载的参数以确认
  ROS_INFO("Loaded calibration parameters:");
  ROS_INFO("x_offset: %f", calibration_params.x_offset);
  ROS_INFO("y_offset: %f", calibration_params.y_offset);
  ROS_INFO("z_offset: %f", calibration_params.z_offset);
  ROS_INFO("roll_offset: %f", calibration_params.roll_offset);
  ROS_INFO("pitch_offset: %f", calibration_params.pitch_offset);
  ROS_INFO("yaw_offset: %f", calibration_params.yaw_offset);
}

void laserCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Create transformation matrix
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << calibration_params.x_offset, calibration_params.y_offset, calibration_params.z_offset;
  Eigen::AngleAxisf rollAngle(calibration_params.roll_offset, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(calibration_params.pitch_offset, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(calibration_params.yaw_offset, Eigen::Vector3f::UnitZ());
  transform.rotate(yawAngle * pitchAngle * rollAngle);

  // 打印转换矩阵以确认
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << transform.matrix() << std::endl;

  // Transform the point cloud
  sensor_msgs::PointCloud2 calibrated_cloud_msg;
  pcl_ros::transformPointCloud(transform.matrix(), *cloud_msg, calibrated_cloud_msg);

  // Publish the calibrated point cloud
  calibrated_cloud_msg.header = cloud_msg->header;
  calibrated_pub.publish(calibrated_cloud_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_calibration_node");
  ros::NodeHandle nh;

  // 获取参数服务器中的参数
  std::string input_topic;
  std::string output_topic;
  int queue_size;

  nh.param<std::string>("input_topic", input_topic, "/lslidar_point_cloud");
  nh.param<std::string>("output_topic", output_topic, "/fusion_points");
  nh.param<int>("queue_size", queue_size, 10);

  // 加载标定参数
  loadCalibrationParams(nh);

  // 订阅输入点云话题
  ros::Subscriber laser_sub = nh.subscribe(input_topic, queue_size, laserCallback);

  // 发布标定后的点云话题
  calibrated_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, queue_size);

  ros::spin();
  return 0;
}
