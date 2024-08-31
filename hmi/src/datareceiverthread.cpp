#include "datareceiverthread.h"
#include <QDebug>
#include <QSettings>
#include <QApplication>

#include <iostream>
DataReceiverThread::DataReceiverThread()
{

}

DataReceiverThread::~DataReceiverThread()
{

}

void DataReceiverThread::run()
{
    //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
    QString currentNodeExeDirPath = QApplication::applicationDirPath();
    QString iniFilePath;
    QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
    //node当前文件夹  lib  devel
    strListCurrentNodeExeDir.removeLast();
    strListCurrentNodeExeDir.removeLast();
    strListCurrentNodeExeDir.removeLast();
    iniFilePath = strListCurrentNodeExeDir.join("/");
    iniFilePath = iniFilePath+"/src/hmi/config/topics.ini";
    QSettings *settings = new QSettings(iniFilePath,QSettings::IniFormat);
    settings->beginGroup("Subscriber");
    std::string faultDiagnosisSubTopic =settings->value("faultDiagnosisSub").toString().toStdString();
    std::string selfCheckDataSubTopic =settings->value("selfCheckDataSub").toString().toStdString();
    std::string gpsImuSubTopic =settings->value("gpsImuSub").toString().toStdString();
    std::string globalPathPlanningSubTopic =settings->value("globalPathPlanningSub").toString().toStdString();
    std::string pathSpeedCtrlSubTopic =settings->value("pathSpeedCtrlSub").toString().toStdString();
    std::string cameraImageSubTopic =settings->value("cameraImageSub").toString().toStdString();
    std::string carOriSubTopic =settings->value("carOriSub").toString().toStdString();
    settings->endGroup();

    // Initialize ROS node
    ros::NodeHandle nh;

    // Create ROS subscribers
    ros::Subscriber faultDiagnosisSub = nh.subscribe(faultDiagnosisSubTopic, 10, &DataReceiverThread::faultDiagnosisCallback, this);
    ros::Subscriber selfCheckDataSub = nh.subscribe(selfCheckDataSubTopic, 10, &DataReceiverThread::SelfCheckDataCallback, this);
    ros::Subscriber hmiSub = nh.subscribe("hmi_start_end_point_data", 10, &DataReceiverThread::hmiCallback, this);
    ros::Subscriber gpsImuSub = nh.subscribe(gpsImuSubTopic, 10, &DataReceiverThread::gpsImuCallback, this);
    ros::Subscriber globalPathPlanningSub = nh.subscribe(globalPathPlanningSubTopic, 10, &DataReceiverThread::globalPathPlanningCallback, this);
    ros::Subscriber pathSpeedCtrlSub = nh.subscribe(pathSpeedCtrlSubTopic, 10, &DataReceiverThread::pathSpeedCtrlCallback, this);
    //ros::Subscriber cameraImageSub = nh.subscribe("camera/image", 10, &DataReceiverThread::CameraImageCallback, this);
    ros::Subscriber cameraImageSub = nh.subscribe(cameraImageSubTopic, 10, &DataReceiverThread::CameraImageCallback, this);
    ros::Subscriber carOriSub = nh.subscribe(carOriSubTopic, 10, &DataReceiverThread::carOriCallback, this);
    // Spin ROS node
    ros::spin();

}

void DataReceiverThread::faultDiagnosisCallback(const hmi::FaultDiagnosisInterface msg)
{
    //优化收取同样的数据刷新界面一次
//    if(faultDiagnosisMsg == msg)
//    {
//        return;
//    }
//    else
//    {
//        faultDiagnosisMsg = msg;
//    }
    emit SIGFaultDiagnosisDataReceived(msg);
}

void DataReceiverThread::SelfCheckDataCallback(const std_msgs::String msg)
{
    emit SIGSelfCheckDataDataReceived(msg);
}

void DataReceiverThread::hmiCallback(const hmi::HmiStartEndPointInterface msg)
{
    emit SIGHmiDataReceived(msg);
}

void DataReceiverThread::gpsImuCallback(const hmi::GpsImuInterface msg)
{
    emit SIGGpsImuDataReceived(msg);
}

void DataReceiverThread::globalPathPlanningCallback(const hmi::GlobalPathPlanningInterface msg)
{
    emit SIGGlobalPathPlanningDataReceived(msg);
}

void DataReceiverThread::pathSpeedCtrlCallback(const hmi::PathSpeedCtrlInterface msg)
{

    emit SIGPathSpeedCtrlDataReceived(msg);
}

void DataReceiverThread::CameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    emit SIGCameraImageReceived(msg);
}

void DataReceiverThread::carOriCallback(const hmi::CarOriInterface msg)
{
    emit SIGCarOriDataReceived(msg);
}
