#ifndef DATARECEIVERTHREAD_H
#define DATARECEIVERTHREAD_H
#include <QCoreApplication>
#include <QThread>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include "hmi/FaultDiagnosisInterface.h"
#include "hmi/HmiStartEndPointInterface.h"
#include "hmi/GpsImuInterface.h"
#include "hmi/GlobalPathPlanningInterface.h"
#include "hmi/PathSpeedCtrlInterface.h"
#include "hmi/CarOriInterface.h"
class DataReceiverThread : public QThread
{
    Q_OBJECT
public:
    DataReceiverThread();
    ~DataReceiverThread();

signals:
    void SIGFaultDiagnosisDataReceived(const hmi::FaultDiagnosisInterface);
    void SIGSelfCheckDataDataReceived(const std_msgs::String);
    void SIGHmiDataReceived(const hmi::HmiStartEndPointInterface);
    void SIGGpsImuDataReceived(const hmi::GpsImuInterface);
    void SIGGlobalPathPlanningDataReceived(const hmi::GlobalPathPlanningInterface);
    void SIGPathSpeedCtrlDataReceived(const hmi::PathSpeedCtrlInterface);
    void SIGCameraImageReceived(const sensor_msgs::ImageConstPtr&);
    void SIGCarOriDataReceived(const hmi::CarOriInterface);

protected:
    void run() override;


private slots:
    void faultDiagnosisCallback(const hmi::FaultDiagnosisInterface msg);
    void SelfCheckDataCallback(const std_msgs::String msg);
    void hmiCallback(const hmi::HmiStartEndPointInterface msg);
    void gpsImuCallback(const hmi::GpsImuInterface msg);
    void globalPathPlanningCallback(const hmi::GlobalPathPlanningInterface msg);
    void pathSpeedCtrlCallback(const hmi::PathSpeedCtrlInterface msg);
    void CameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void carOriCallback(const hmi::CarOriInterface msg);

private:
    hmi::FaultDiagnosisInterface faultDiagnosisMsg;
};
#endif // DATARECEIVERTHREAD_H
