#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QDockWidget>
#include <QLabel>
#include <QThread>
#include <QMetaType>
#include <QPaintEvent>
#include <sensor_msgs/Image.h>

#include "hmi/HmiStartEndPointInterface.h"
#include "hmi/NodePointsInterface.h"
#include "hmi/FaultDiagnosisInterface.h"
#include "std_msgs/String.h"
#include "hmi/PathSpeedCtrlInterface.h"
#include "hmi/GpsImuInterface.h"
#include "hmi/GlobalPathPlanningInterface.h"
//#include "hmi/CarOriInterface.h"
class InstructionDeliverWidget;
class MapCollectWidget;
class CarMoniterWidget;
class FaultDetectWidget;
class PerceivedResultWidget;
class DataPublisherThread;
class DataReceiverThread;
class RvizModuleWidget;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget* parents = nullptr, Qt::WindowFlags flags = 0);
    ~MainWindow();
public:
    void Init();
protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void InitMember();
    void InitView();
    void InitSlot();
    void LayoutUI();

private slots:
    void SlotFunctionDeliverInstructionMsg(const hmi::HmiStartEndPointInterface hmiStartEndPointMsg);
    void SlotFunctionDeliverNodeData(const hmi::NodePointsInterface nodePointsMsg);

    void SlotFunctionReceiveFaultDetectMsg(const hmi::FaultDiagnosisInterface faultDetectMsg);
//    void SlotFunctionReceiveCarOriMsg(const hmi::CarOriInterface carOrinMsg);

private:
    void ResizsDock();
    DataReceiverThread* dataReceiverThread;

    DataPublisherThread* dataPublisherThreadInstruction;
    QThread* threadPublisherHmiStartEndPointMsg;

    DataPublisherThread* dataPublisherThreadNode;
    QThread* threadPublisherNodePointsMsg;

//    DataPublisherThread* dataPublisherThreadCarAction;
//    QThread* threadPublisherCarActionMsg;
    int deskCount;
    int LidarState;
    bool GpsState;
    bool CanState;

    struct UIMainWindow
    {

        /*-----Layouts-------- */

        QDockWidget* dockTopLogo;
        //主界面指令下发
        QDockWidget* dockInstructionDeliver;
        //主界面车辆作业监控
        QDockWidget* dockCarMoniter;
        //主界面感知结果
        QDockWidget* dockPerceivedResults;
        //主界面参数优化
        QDockWidget* dockParamOptimize;
        //主界面故障检测
        QDockWidget* dockFaultDetect;

        QDockWidget* dockCarAction;

        QDockWidget* dockMapCollect;


        /*-----Widgets-------- */

        QLabel* labelTJUNLogo;
        QLabel* labelTJUNName;

        InstructionDeliverWidget* instructionDeliverWidget;
        MapCollectWidget* mapCollectWidget;
        CarMoniterWidget* carMoniterWidget;
        //结果树图
        PerceivedResultWidget* perceivedResultWidget;

        FaultDetectWidget* faultDetectWidget;

        RvizModuleWidget* rvizModuleWidget;
        QMessageBox* messageBoxWaitSelfCheck;

        QWidget* perceiveWidget;
    };


    UIMainWindow ui;
};
#endif // MAINWINDOW_H
