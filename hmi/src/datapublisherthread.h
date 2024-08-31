
#ifndef DATAPUBLISHERTHREAD_H
#define DATAPUBLISHERTHREAD_H

#include <ros/ros.h>
#include <QString>
#include <QObject>

#include "hmi/HmiStartEndPointInterface.h"
#include "hmi/NodePointsInterface.h"
class DataPublisherThread : public QObject
{
    Q_OBJECT

public:
    explicit DataPublisherThread(QObject* parent = nullptr);
    ~DataPublisherThread();
public:
    void SetHmiStaratEndPointMsg(const hmi::HmiStartEndPointInterface msg);
    void SetNodePointsMsg(const hmi::NodePointsInterface msg);
public slots:
    void SlotPublisherHmiStartEndPointMsg();
    void SlotPublisherNodePointsMsg();
protected:
private:

private:
    ros::NodeHandle nh;

    ros::Publisher hmiStartEndPointPub;
    hmi::HmiStartEndPointInterface hmiStartEndPointMsg;

    ros::Publisher nodePointsPub;
    hmi::NodePointsInterface nodePointsMsg;
};

#endif // DATAPUBLISHERTHREAD_H

