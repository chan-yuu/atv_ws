#include "datapublisherthread.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>

DataPublisherThread::DataPublisherThread(QObject* parent)
    :QObject(parent)
    ,hmiStartEndPointPub()
    ,nodePointsPub()
{

}

DataPublisherThread::~DataPublisherThread()
{
    ros::shutdown();
}


void DataPublisherThread::SetHmiStaratEndPointMsg(const hmi::HmiStartEndPointInterface msg)
{
    hmiStartEndPointMsg = msg;
}

void DataPublisherThread::SetNodePointsMsg(const hmi::NodePointsInterface msg)
{
    nodePointsMsg = msg;
}

void DataPublisherThread::SlotPublisherHmiStartEndPointMsg()
{
    if(hmiStartEndPointPub==ros::Publisher())
    {
        nh= ros::NodeHandle();
        hmiStartEndPointPub= nh.advertise<hmi::HmiStartEndPointInterface>("hmi_start_end_point_data", 1000);
    }

    // 定义循环发送消息的逻辑
    ros::Rate loop_rate(10);  // 假设每秒发送10条消息
    while (ros::ok())
    {

         // 发布消息
         hmiStartEndPointPub.publish(hmiStartEndPointMsg);

         // 处理ROS回调
         ros::spinOnce();

         // 休眠一段时间，以控制发送频率
         loop_rate.sleep();
    }
}

void DataPublisherThread::SlotPublisherNodePointsMsg()
{
    if(nodePointsPub==ros::Publisher())
    {
         nh= ros::NodeHandle();
         nodePointsPub= nh.advertise<hmi::NodePointsInterface>("node_points_data", 1000);
    }

    // 定义循环发送消息的逻辑
    ros::Rate loop_rate(10);  // 假设每秒发送10条消息
    while (ros::ok())
    {

         // 发布消息
         nodePointsPub.publish(nodePointsMsg);

         // 处理ROS回调
         ros::spinOnce();

         // 休眠一段时间，以控制发送频率
         loop_rate.sleep();
    }
}
