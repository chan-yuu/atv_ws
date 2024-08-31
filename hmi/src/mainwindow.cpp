#include "mainwindow.h"
#include "instructiondeliverwidget.h"
#include "mapcollectwidget.h"
#include "nodemanagewidget.h"
#include "carmoniterwidget.h"
#include "faultdetectwidget.h"
#include "perceivedresultwidget.h"
#include "rvizmodulewidget.h"
#include "datapublisherthread.h"
#include "datareceiverthread.h"
#include "msgprocesstools.h"

#include <iostream>

#include <QPixmap>
#include <QLayout>
#include <QDebug>
#include <QApplication>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QSettings>
using namespace std;
MainWindow::MainWindow(int argc, char** argv, QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow(parent)
{
    Init();
}

MainWindow::~MainWindow()
{
    ros::shutdown();
    if (dataReceiverThread)
    {
        dataReceiverThread->quit();
        dataReceiverThread->wait();
        delete dataReceiverThread;
        dataReceiverThread=nullptr;
    }
    if (threadPublisherHmiStartEndPointMsg)
    {
        threadPublisherHmiStartEndPointMsg->quit();
        threadPublisherHmiStartEndPointMsg->wait();
        delete threadPublisherHmiStartEndPointMsg;
        threadPublisherHmiStartEndPointMsg=nullptr;
    }
    if (threadPublisherNodePointsMsg)
    {
        threadPublisherNodePointsMsg->quit();
        threadPublisherNodePointsMsg->wait();
        delete threadPublisherNodePointsMsg;
        threadPublisherNodePointsMsg=nullptr;
    }
//    if (threadPublisherCarActionMsg)
//    {
//        threadPublisherCarActionMsg->quit();
//        threadPublisherCarActionMsg->wait();
//        delete threadPublisherCarActionMsg;
//        threadPublisherCarActionMsg=nullptr;
//    }

//    //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
//    QString currentNodeExeDirPath = QApplication::applicationDirPath();
//    QString iniFilePath;
//    QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
//    //node当前文件夹  lib  devel
//    strListCurrentNodeExeDir.removeLast();
//    strListCurrentNodeExeDir.removeLast();
//    strListCurrentNodeExeDir.removeLast();
//    iniFilePath = strListCurrentNodeExeDir.join("/");
//    iniFilePath = iniFilePath+"/src/hmi/config/topics.ini";
//    QSettings *settings = new QSettings(iniFilePath,QSettings::IniFormat);
//    // 写入第一组数据
//    settings->beginGroup("Subscriber");
//    settings->setValue("faultDiagnosisSub","fault_diagnosis_data");
//    settings->setValue("selfCheckDataSub","from_vcu_self_check");
//    settings->setValue("gpsImuSub","gps_imu");
//    settings->setValue("globalPathPlanningSub","global_path_planning_data");
//    settings->setValue("pathSpeedCtrlSub","path_speed_tracking_data");
//    settings->setValue("cameraImageSub","usb_cam/image_raw");
//    settings->setValue("carOriSub","car_ori_data");
//    settings->endGroup();

//    delete settings;
//    settings =nullptr;
}

void MainWindow::paintEvent(QPaintEvent* event)
{
    /*if(!LidarState
        || !GpsState
        || !CanState)
    {
        if(!ui.messageBoxWaitSelfCheck->isVisible())
        {
            ui.messageBoxWaitSelfCheck->show();
        }
    }

    if(LidarState
        && GpsState
        && CanState)
    {
        if(ui.messageBoxWaitSelfCheck->isVisible())
        {
            ui.messageBoxWaitSelfCheck->close();
        }
    }*/
    QMainWindow::paintEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    ResizsDock();
    QMainWindow::resizeEvent(event);
}

void MainWindow::Init()
{
    setObjectName("AIConnect");
    setWindowIcon(QIcon("://source/1.png"));
    setWindowTitle("天津大学无人驾驶地图采集人机交互界面");
    setDockNestingEnabled(true);

    InitMember();
    InitView();
    LayoutUI();
    InitSlot();

    setWindowFlags(Qt::Window | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
}

void MainWindow::InitMember()
{
  dataPublisherThreadInstruction = new DataPublisherThread();
  threadPublisherHmiStartEndPointMsg = new QThread();
  dataPublisherThreadInstruction->moveToThread(threadPublisherHmiStartEndPointMsg);

  dataPublisherThreadNode = new DataPublisherThread();
  threadPublisherNodePointsMsg = new QThread;
  dataPublisherThreadNode->moveToThread(threadPublisherNodePointsMsg);

//  dataPublisherThreadCarAction = new DataPublisherThread();
//  threadPublisherCarActionMsg = new QThread;
//  dataPublisherThreadCarAction->moveToThread(threadPublisherCarActionMsg);

  dataReceiverThread = new DataReceiverThread();
  dataReceiverThread->start();

  LidarState = 0;
  GpsState = false;
  CanState = false;

  ui.labelTJUNLogo = new QLabel();

  ui.labelTJUNName = new QLabel();

  ui.instructionDeliverWidget = new InstructionDeliverWidget();
  ui.instructionDeliverWidget->Init();

  ui.mapCollectWidget = new MapCollectWidget();
  ui.mapCollectWidget->Init();

  ui.carMoniterWidget = new CarMoniterWidget();
  ui.carMoniterWidget->Init();

  ui.perceivedResultWidget = new PerceivedResultWidget();
  ui.perceivedResultWidget->Init();


  ui.faultDetectWidget = new FaultDetectWidget();
  ui.faultDetectWidget->Init();

  ui.rvizModuleWidget = new RvizModuleWidget();
  ui.rvizModuleWidget->Init();

  ui.messageBoxWaitSelfCheck = new QMessageBox();
  ui.messageBoxWaitSelfCheck->setWindowTitle("自检");
  ui.messageBoxWaitSelfCheck->setText("等待自检......");
  ui.messageBoxWaitSelfCheck->setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);
  ui.messageBoxWaitSelfCheck->addButton(QMessageBox::Ok);
  ui.messageBoxWaitSelfCheck->button(QMessageBox::Ok)->hide();
  ui.messageBoxWaitSelfCheck->setModal(true);
}

void MainWindow::InitView()
{
  QDesktopWidget* desktopWidget = QApplication::desktop();
  deskCount = desktopWidget->screenCount();
  if(deskCount == 1)
  {
    QRect screenRect = desktopWidget->screenGeometry();
    setGeometry(screenRect);
  }

  ui.perceiveWidget = new QWidget();
  QHBoxLayout* hboxLayoutPerceiveWidget = new QHBoxLayout(ui.perceiveWidget);
  hboxLayoutPerceiveWidget->addWidget(ui.perceivedResultWidget);
  hboxLayoutPerceiveWidget->addWidget(ui.rvizModuleWidget);
  hboxLayoutPerceiveWidget->setStretch(0,1);
  hboxLayoutPerceiveWidget->setStretch(1,1);

//  QVBoxLayout* vboxLayoutPerceiveWidget = new QVBoxLayout(ui.perceiveWidget);
//  vboxLayoutPerceiveWidget->addWidget(ui.perceivedResultWidget);
//  vboxLayoutPerceiveWidget->addWidget(ui.rvizModuleWidget);
//  vboxLayoutPerceiveWidget->setStretch(0,1);
//  vboxLayoutPerceiveWidget->setStretch(1,1);
  if(deskCount == 2)
  {
    //主屏
    QRect screenRect1 = desktopWidget->screenGeometry(0);
    setGeometry(screenRect1);
    //分屏
    QRect screenRect2 = desktopWidget->screenGeometry(1);
    ui.perceiveWidget->setGeometry(screenRect2);
    ui.perceiveWidget->setWindowTitle("感知结果");
    ui.perceiveWidget->show();
  }
  QWidget* widgetTopLogo = new QWidget();

  {
      QPixmap pixmapTJUNLogo;
      pixmapTJUNLogo.load("://source/1.png");
      pixmapTJUNLogo = pixmapTJUNLogo.scaled(25, 25);
      ui.labelTJUNLogo->setPixmap(pixmapTJUNLogo);
      ui.labelTJUNLogo->setAlignment(Qt::AlignHCenter);
      ui.labelTJUNLogo->setStyleSheet("border:0xp;");

      ui.labelTJUNName->setWordWrap(true);
      ui.labelTJUNName->setAlignment(Qt::AlignTop | Qt::AlignHCenter);
      QString TJUNName = "天津大学无人驾驶地图采集人机交互界面";
      ui.labelTJUNName->setText(TJUNName.split("", QString::SkipEmptyParts).join("\n"));
      QFont fontTJUNName;
      fontTJUNName.setBold(true);
      fontTJUNName.setPointSize(font_size+1);
      ui.labelTJUNName->setFont(fontTJUNName);
      ui.labelTJUNName->setStyleSheet("border:0xp;");
  }

  //dockwigdet initview
  {
      QString styleSheet = "border: 1px solid gray;";
      ui.dockTopLogo = new QDockWidget(this);
      ui.dockTopLogo->setTitleBarWidget(NULL);
      ui.dockTopLogo->setStyleSheet(styleSheet);
      QVBoxLayout* vboxLayoutTopLogo = new QVBoxLayout(widgetTopLogo);
      vboxLayoutTopLogo->addWidget(ui.labelTJUNLogo);
      vboxLayoutTopLogo->addWidget(ui.labelTJUNName);
      vboxLayoutTopLogo->setStretch(0, 1);
      vboxLayoutTopLogo->setStretch(1, 8);
      ui.dockTopLogo->setWidget(widgetTopLogo);

      ui.dockMapCollect = new QDockWidget(tr("地图采集"), this);
      ui.dockMapCollect->setWidget(ui.mapCollectWidget);

      ui.dockInstructionDeliver = new QDockWidget(tr("指令下发"), this);
      ui.dockInstructionDeliver->setWidget(ui.instructionDeliverWidget);

      ui.dockCarMoniter = new QDockWidget(tr("车辆作业检测"), this);
      ui.dockCarMoniter->setWidget(ui.carMoniterWidget);

      ui.dockPerceivedResults = new QDockWidget(tr("感知结果"), this);
      if(deskCount == 1)
      {
        ui.dockPerceivedResults->setWidget(ui.perceiveWidget);
      }

      //ui.dockParamOptimize = new QDockWidget(tr("参数优化"), this);
      //ui.dockParamOptimize->setWidget(ui.paramOptimizeWidget);

      ui.dockFaultDetect = new QDockWidget(tr("故障检测"), this);
      ui.dockFaultDetect->setWidget(ui.faultDetectWidget);

      //ui.dockCarAction = new QDockWidget(tr("车辆控制"),this);
      //ui.dockCarAction->setWidget(ui.carActionDeliverWidget);

  }


}

void MainWindow::InitSlot()
{
  //告知QObject
  qRegisterMetaType<hmi::HmiStartEndPointInterface>("hmi::HmiStartEndPointInterface");
  qRegisterMetaType<hmi::FaultDiagnosisInterface>("hmi::FaultDiagnosisInterface");
  qRegisterMetaType<hmi::PathSpeedCtrlInterface>("hmi::PathSpeedCtrlInterface");
  qRegisterMetaType<std_msgs::String>("std_msgs::String");
  qRegisterMetaType<hmi::GpsImuInterface>("hmi::GpsImuInterface");
  qRegisterMetaType<hmi::GlobalPathPlanningInterface>("hmi::GlobalPathPlanningInterface");
  qRegisterMetaType<sensor_msgs::ImageConstPtr>(" sensor_msgs::ImageConstPtr");
  qRegisterMetaType<hmi::NodePointsInterface>("hmi::NodePointsInterface");

  connect(ui.instructionDeliverWidget,&InstructionDeliverWidget::SIGInstructionDeliverMsg,this,&MainWindow::SlotFunctionDeliverInstructionMsg);
  connect(ui.mapCollectWidget,&MapCollectWidget::SIGNodeDataDeliver,this,&MainWindow::SlotFunctionDeliverNodeData);

  connect(dataReceiverThread,&DataReceiverThread::SIGFaultDiagnosisDataReceived,ui.faultDetectWidget,&FaultDetectWidget::SlotFunctionReceiveFaultDetectMsg);
  connect(dataReceiverThread,&DataReceiverThread::SIGSelfCheckDataDataReceived,ui.faultDetectWidget,&FaultDetectWidget::SlotFunctionReceiveSelfCheckMsg);
  connect(dataReceiverThread,&DataReceiverThread::SIGGpsImuDataReceived,ui.carMoniterWidget,&CarMoniterWidget::SlotFunctionReceiveGpsImuMsg);
  connect(dataReceiverThread,&DataReceiverThread::SIGGpsImuDataReceived,ui.mapCollectWidget,&MapCollectWidget::SlotFunctionReceiveGPSImuMsg);
  connect(dataReceiverThread,&DataReceiverThread::SIGGpsImuDataReceived,ui.mapCollectWidget->GetNodeManagetWidget(),&NodeManageWidget::SlotFunctionReceiveGPSImuMsg);
  //connect(dataReceiverThread,&DataReceiverThread::SIGGlobalPathPlanningDataReceived,ui.carMoniterWidget,&CarMoniterWidget::SlotFunctionReceiveGlobalPathPlanningMsg);
  connect(dataReceiverThread,&DataReceiverThread::SIGPathSpeedCtrlDataReceived,ui.carMoniterWidget,&CarMoniterWidget::SlotFunctionReceivePathSpeedCtrlDeliverMsg);
  connect(dataReceiverThread,&DataReceiverThread::SIGCameraImageReceived,ui.perceivedResultWidget,&PerceivedResultWidget::SlotFunctionReceiveCameraImageMsg);

  connect(dataReceiverThread,&DataReceiverThread::SIGFaultDiagnosisDataReceived,this,&MainWindow::SlotFunctionReceiveFaultDetectMsg);

  connect(ui.instructionDeliverWidget,&InstructionDeliverWidget::SIGMapFileToPathDrawWidget,ui.carMoniterWidget,&CarMoniterWidget::SlotFunctionReceiveMapFilePath);
}

void MainWindow::LayoutUI()
{
  //清除自带centeralwidget
  QWidget* p = takeCentralWidget();
  if (p)
  {
    delete p;
    p = nullptr;
  }
  //布局docketwidget
  addDockWidget(Qt::TopDockWidgetArea, ui.dockTopLogo);
  addDockWidget(Qt::TopDockWidgetArea, ui.dockInstructionDeliver);
  addDockWidget(Qt::TopDockWidgetArea, ui.dockCarMoniter);
  if(deskCount == 1)
  {
    addDockWidget(Qt::TopDockWidgetArea, ui.dockPerceivedResults);
  }
  //tabifyDockWidget(ui.dockPerceivedResults, ui.dockCarMoniter);
  //addDockWidget(Qt::BottomDockWidgetArea, ui.dockCarAction);
  //splitDockWidget(ui.dockCarAction, ui.dockFourthParamOptimize, Qt::Horizontal);
  //splitDockWidget(ui.dockFourthParamOptimize, ui.dockFifthFaultDetect, Qt::Horizontal);
  //splitDockWidget(ui.dockInstructionDeliver, ui.dockMapCollect, Qt::Vertical);
  addDockWidget(Qt::BottomDockWidgetArea, ui.dockMapCollect);
  addDockWidget(Qt::BottomDockWidgetArea, ui.dockFaultDetect);
  ui.dockInstructionDeliver->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  ui.dockCarMoniter->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  ui.dockPerceivedResults->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  ui.dockMapCollect->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
//  ui.dockParamOptimize->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
  ui.dockFaultDetect->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);
//  ui.dockCarAction->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable);

  ResizsDock();
}

void MainWindow::SlotFunctionDeliverInstructionMsg(const hmi::HmiStartEndPointInterface hmiStartEndPointMsg)
{
  dataPublisherThreadInstruction->SetHmiStaratEndPointMsg(hmiStartEndPointMsg);
  connect(threadPublisherHmiStartEndPointMsg,&QThread::started,dataPublisherThreadInstruction,&DataPublisherThread::SlotPublisherHmiStartEndPointMsg,Qt::UniqueConnection);
  if(!threadPublisherHmiStartEndPointMsg->isRunning())
  {
    threadPublisherHmiStartEndPointMsg->start();
  }


  if(!threadPublisherHmiStartEndPointMsg->isRunning())
  {
    QMessageBox::warning(this, "指令下发", "指令下发失败，线程未成功启动!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }

  QMessageBox::information(this, "指令下发", "指令下法成功!", QMessageBox::Yes, QMessageBox::Yes);
  return;
}

void MainWindow::SlotFunctionDeliverNodeData(const hmi::NodePointsInterface nodePointsMsg)
{
  dataPublisherThreadNode->SetNodePointsMsg(nodePointsMsg);
  connect(threadPublisherNodePointsMsg,&QThread::started,dataPublisherThreadNode,&DataPublisherThread::SlotPublisherNodePointsMsg,Qt::UniqueConnection);
  if(!threadPublisherNodePointsMsg->isRunning())
  {
    threadPublisherNodePointsMsg->start();
  }
  if(!threadPublisherNodePointsMsg->isRunning())
  {
    QMessageBox::warning(this, "路口与站点", "路口与站点下发失败，线程未成功启动!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
  else
  {
    QMessageBox::information(this, "路口与站点", "路口与站点下发成功!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
}

void MainWindow::SlotFunctionReceiveFaultDetectMsg(const hmi::FaultDiagnosisInterface faultDetectMsg)
{
  LidarState = faultDetectMsg.lidar_start;
  GpsState = faultDetectMsg.Gps_state_fault;
  CanState = faultDetectMsg.can_state;
}

//void MainWindow::SlotFunctionDeliverCarActionMsg(const hmi::CarActionInterface carActionMsg)
//{
//  dataPublisherThreadCarAction->SetCarActionMsg(carActionMsg);
//  connect(threadPublisherCarActionMsg,&QThread::started,dataPublisherThreadCarAction,&DataPublisherThread::SlotPublisherCarActionMsg,Qt::UniqueConnection);
//  if(!threadPublisherCarActionMsg->isRunning())
//  {
//    threadPublisherCarActionMsg->start();
//  }
//  if(!threadPublisherCarActionMsg->isRunning())
//  {
//    QMessageBox::warning(this, "车辆控制", "车辆控制下发失败，线程未成功启动!", QMessageBox::Yes, QMessageBox::Yes);
//  }
//  else
//  {
//    QMessageBox::information(this, "车辆控制", "车辆控制下发成功!", QMessageBox::Yes, QMessageBox::Yes);
//  }
//}

//void MainWindow::SlotFunctionReceiveCarOriMsg(const hmi::CarOriInterface carOrinMsg)
//{
//  waitWireControlState = carOrinMsg.wait_wirectrl;
//  if(waitWireControlState)
//  {
//    ui.messageBoxWaitWireContril->close();
//  }
//}

void MainWindow::ResizsDock()
{
  int width = this->size().width();
  int height = this->size().height();
  QList<QDockWidget*> horizontalFirstRowDockList;
  horizontalFirstRowDockList << ui.dockTopLogo;
  horizontalFirstRowDockList << ui.dockInstructionDeliver;
  //horizontalFirstRowDockList << ui.dockMapCollect;
  horizontalFirstRowDockList << ui.dockCarMoniter;
  if(deskCount == 1)
  {
    horizontalFirstRowDockList << ui.dockPerceivedResults;
  }


  QList<int> horizontalFirstRowSizeList;
  horizontalFirstRowSizeList << static_cast<int>(width * 0.05);
  horizontalFirstRowSizeList << static_cast<int>(width * 0.2);
  //horizontalFirstRowSizeList << static_cast<int>(width * 0.2);
  horizontalFirstRowSizeList << static_cast<int>(width * 0.4);
  if(deskCount == 1)
  {
    horizontalFirstRowSizeList << static_cast<int>(width * 0.35);
  }
  this->resizeDocks(horizontalFirstRowDockList, horizontalFirstRowSizeList, Qt::Horizontal);

  QList<QDockWidget*> horizontalSecondRowDockList;
  horizontalSecondRowDockList << ui.dockMapCollect;
  horizontalSecondRowDockList << ui.dockFaultDetect;

  QList<int> horizontalSecondRowSizeList;
  horizontalSecondRowSizeList << static_cast<int>(width * 0.25);
  horizontalSecondRowSizeList << static_cast<int>(width * 0.75);
  this->resizeDocks(horizontalSecondRowDockList, horizontalSecondRowSizeList, Qt::Horizontal);

  QList<int> verticalSizeList;
  verticalSizeList<<static_cast<int>(height * 0.7);
  verticalSizeList<<static_cast<int>(height * 0.7);
  verticalSizeList<<static_cast<int>(height * 0.7);
  if(deskCount == 1)
  {
    verticalSizeList<<static_cast<int>(height * 0.7);
  }
  verticalSizeList<<static_cast<int>(height * 0.3);
  verticalSizeList<<static_cast<int>(height * 0.3);
  QList<QDockWidget*> verticalDockList;
  verticalDockList<<ui.dockTopLogo;
  verticalDockList<<ui.dockInstructionDeliver;
  verticalDockList<<ui.dockCarMoniter;
  if(deskCount == 1)
  {
    verticalDockList<<ui.dockPerceivedResults;
  }
  verticalDockList<<ui.dockMapCollect;
  verticalDockList<<ui.dockFaultDetect;
  this->resizeDocks(verticalDockList, verticalSizeList, Qt::Vertical);

  if(deskCount == 1)
  {
    ui.perceivedResultWidget->setMaximumSize(QSize(width*0.35,height*0.35));
    ui.rvizModuleWidget->setMaximumSize(QSize(width*0.35,height*0.35));
  }

  if(deskCount == 2)
  {
    width = ui.perceiveWidget->width();
    height = ui.perceiveWidget->height();
    //ui.perceivedResultWidget->setMaximumSize(QSize(width,height*0.5));
    //ui.rvizModuleWidget->setMaximumSize(QSize(width,height*0.5));
    ui.perceivedResultWidget->setMaximumSize(QSize(width*0.5,height));
    ui.rvizModuleWidget->setMaximumSize(QSize(width*0.5,height));
  }
}
