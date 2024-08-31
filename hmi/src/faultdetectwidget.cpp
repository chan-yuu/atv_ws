#include "faultdetectwidget.h"

#include <QLayout>
#include <QDebug>
#include <QPainter>

FaultDetectWidget::FaultDetectWidget(QWidget* parent)
  :QWidget(parent)
{

}

FaultDetectWidget::~FaultDetectWidget()
{

}

void FaultDetectWidget::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  QPen pen;
  pen.setColor(Qt::gray);
  pen.setWidth(1);
  painter.setPen(pen);
  painter.drawRect(this->rect().x(), this->rect().y(), this->rect().width() - 1, this->rect().height() - 1);
  QWidget::paintEvent(event);
}


void FaultDetectWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

void FaultDetectWidget::InitMember()
{
  ui.labelInertialNavigation = new QLabel("惯导状态故障:");
  ui.labelDriveByWire = new QLabel("线控油门故障:");
  ui.labelSearchStars = new QLabel("搜星数量故障:");
  ui.labelBrakeByWire = new QLabel("线控刹车故障:");
  ui.labelSteerByWire = new QLabel("线控转向故障:");
  ui.labelVisionInspection = new QLabel("视觉检测故障:");
  ui.labelMillimeterWaveRadar = new QLabel("毫米雷达故障:");
  ui.labelHeadDrive = new QLabel("采头驱动故障:");
  ui.labelVCUCommunication = new QLabel("VCU通信故障: ");
  ui.labelPublicNetworkCommunication = new QLabel("公网通信故障:");
  ui.labelInertialNavigationMode = new QLabel("惯导模式故障:");
  ui.labelInertialNavigationCommunication =  new QLabel("惯导通信故障:");
  ui.labelMillimeterWaveRaderCommunication = new QLabel("毫米波通信故障:");
  ui.labelSatelliteStatus = new QLabel("卫星状态故障:");

  ui.labelInertialNavigationState = new QLabel();
  ui.labelDriveByWireState = new QLabel();
  ui.labelSearchStarsState = new QLabel();
  ui.labelBrakeByWireState = new QLabel();
  ui.labelSteerByWireState = new QLabel();
  ui.labelVisionInspectionState = new QLabel();
  ui.labelMillimeterWaveRadarState = new QLabel();
  ui.labelHeadDriveState = new QLabel();
  ui.labelVCUCommunicationState = new QLabel();
  ui.labelPublicNetworkCommunicationState = new QLabel();
  ui.labelInertialNavigationModeState= new QLabel();
  ui.labelInertialNavigationCommunicationState= new QLabel();
  ui.labelMillimeterWaveRaderCommunicationState= new QLabel();
  ui.labelSatelliteStatusState= new QLabel();



  ui.labelAutoDriveEnable = new QLabel("自动驾驶状态:");
  ui.labelSteering = new QLabel("驾驶模式:");
  ui.labelEbs = new QLabel("EBS状态:");
  ui.labelBrake = new QLabel("刹车状态:");
  ui.labelFrontDoor = new QLabel("前门状态:");
  ui.labelMiddleDoor = new QLabel("后门状态:");

  ui.labelAutoDriveEnableState = new QLabel();
  ui.labelSteeringState = new QLabel();
  ui.labelEbsState = new QLabel();
  ui.labelBrakeState = new QLabel();
  ui.labelFrontDoorState = new QLabel();
  ui.labelMiddleDoorState = new QLabel();


  ui.labelGps = new QLabel("惯导通讯故障:");
  ui.labelCan = new QLabel("Can通讯故障:");

  ui.labelGpsState = new QLabel();
  ui.labelCanState = new QLabel();


  SetLabelLEDState(ui.labelInertialNavigationState, StateColor::RED);
  SetLabelLEDState(ui.labelDriveByWireState, StateColor::RED);
  SetLabelLEDState(ui.labelSearchStarsState, StateColor::RED);
  SetLabelLEDState(ui.labelBrakeByWireState, StateColor::RED);
  SetLabelLEDState(ui.labelSteerByWireState, StateColor::RED);
  SetLabelLEDState(ui.labelVisionInspectionState, StateColor::RED);
  SetLabelLEDState(ui.labelMillimeterWaveRadarState, StateColor::RED);
  SetLabelLEDState(ui.labelHeadDriveState, StateColor::RED);
  SetLabelLEDState(ui.labelVCUCommunicationState, StateColor::RED);
  SetLabelLEDState(ui.labelPublicNetworkCommunicationState, StateColor::RED);
  SetLabelLEDState(ui.labelInertialNavigationModeState, StateColor::RED);
  SetLabelLEDState(ui.labelInertialNavigationCommunicationState, StateColor::RED);
  SetLabelLEDState(ui.labelMillimeterWaveRaderCommunicationState, StateColor::RED);
  SetLabelLEDState(ui.labelSatelliteStatusState, StateColor::RED);

  SetLabelLEDState(ui.labelAutoDriveEnableState, StateColor::RED);
  SetLabelLEDState(ui.labelSteeringState, StateColor::RED);
  SetLabelLEDState(ui.labelEbsState, StateColor::RED);
  SetLabelLEDState(ui.labelBrakeState, StateColor::RED);
  SetLabelLEDState(ui.labelFrontDoorState, StateColor::RED);
  SetLabelLEDState(ui.labelMiddleDoorState, StateColor::RED);

  SetLabelLEDState(ui.labelGpsState, StateColor::RED);
  SetLabelLEDState(ui.labelCanState, StateColor::RED);
}

void FaultDetectWidget::InitView()
{
  QGridLayout* gridLayout = new QGridLayout(this);

  QHBoxLayout* hboxLayoutFirstRow = new QHBoxLayout();
  QHBoxLayout* hboxLayoutSecondRow = new QHBoxLayout();
//  QHBoxLayout* hboxLayoutThirdRow = new QHBoxLayout();
//  QHBoxLayout* hboxLayoutFourthRow = new QHBoxLayout();

  hboxLayoutFirstRow->addWidget(ui.labelAutoDriveEnable);
  hboxLayoutFirstRow->addWidget(ui.labelAutoDriveEnableState);
  hboxLayoutFirstRow->addStretch();
  //hboxLayoutFirstRow->addWidget(ui.labelSteering);
  //hboxLayoutFirstRow->addWidget(ui.labelSteeringState);
  //hboxLayoutFirstRow->addStretch();
  hboxLayoutFirstRow->addWidget(ui.labelEbs);
  hboxLayoutFirstRow->addWidget(ui.labelEbsState);
  hboxLayoutFirstRow->addStretch();
  hboxLayoutFirstRow->addWidget(ui.labelBrake);
  hboxLayoutFirstRow->addWidget(ui.labelBrakeState);
  hboxLayoutFirstRow->addStretch();
  hboxLayoutFirstRow->addWidget(ui.labelFrontDoor);
  hboxLayoutFirstRow->addWidget(ui.labelFrontDoorState);
  hboxLayoutFirstRow->addStretch();
  hboxLayoutFirstRow->addWidget(ui.labelMiddleDoor);
  hboxLayoutFirstRow->addWidget(ui.labelMiddleDoorState);
  hboxLayoutFirstRow->addStretch();

  hboxLayoutSecondRow->addWidget(ui.labelGps);
  hboxLayoutSecondRow->addWidget(ui.labelGpsState);
  hboxLayoutSecondRow->addStretch();
  hboxLayoutSecondRow->addWidget(ui.labelCan);
  hboxLayoutSecondRow->addWidget(ui.labelCanState);
  //hboxLayoutSecondRow->addStretch();
  //hboxLayoutSecondRow->addStretch();
  //hboxLayoutSecondRow->addStretch();
  hboxLayoutSecondRow->addStretch();
  hboxLayoutSecondRow->addStretch();
  hboxLayoutSecondRow->addStretch();
  hboxLayoutSecondRow->addStretch();
  hboxLayoutSecondRow->addStretch();

//  hboxLayoutSecondRow->addWidget(ui.labelSearchStars);
//  hboxLayoutSecondRow->addWidget(ui.labelSearchStarsState);
//  hboxLayoutSecondRow->addStretch();
//  hboxLayoutSecondRow->addWidget(ui.labelBrakeByWire);
//  hboxLayoutSecondRow->addWidget(ui.labelBrakeByWireState);
//  hboxLayoutSecondRow->addStretch();
//  hboxLayoutSecondRow->addWidget(ui.labelSteerByWire);
//  hboxLayoutSecondRow->addWidget(ui.labelSteerByWireState);
//  hboxLayoutSecondRow->addStretch();
//  hboxLayoutSecondRow->addWidget(ui. );
//  hboxLayoutSecondRow->addWidget(ui.labelVisionInspectionState);
//  hboxLayoutSecondRow->addStretch();

//  hboxLayoutThirdRow->addWidget(ui.labelMillimeterWaveRadar);
//  hboxLayoutThirdRow->addWidget(ui.labelMillimeterWaveRadarState);
//  hboxLayoutThirdRow->addStretch();
//  hboxLayoutThirdRow->addWidget(ui.labelHeadDrive);
//  hboxLayoutThirdRow->addWidget(ui.labelHeadDriveState);
//  hboxLayoutThirdRow->addStretch();
//  hboxLayoutThirdRow->addWidget(ui.labelVCUCommunication);
//  hboxLayoutThirdRow->addWidget(ui.labelVCUCommunicationState);
//  hboxLayoutThirdRow->addStretch();
//  hboxLayoutThirdRow->addWidget(ui.labelPublicNetworkCommunication);
//  hboxLayoutThirdRow->addWidget(ui.labelPublicNetworkCommunicationState);
//  hboxLayoutThirdRow->addStretch();

//  hboxLayoutFourthRow->addWidget(ui.labelInertialNavigationMode);
//  hboxLayoutFourthRow->addWidget(ui.labelInertialNavigationModeState);
//  hboxLayoutFourthRow->addStretch();
//  hboxLayoutFourthRow->addWidget(ui.labelInertialNavigationCommunication);
//  hboxLayoutFourthRow->addWidget(ui.labelInertialNavigationCommunicationState);
//  hboxLayoutFourthRow->addStretch();
//  hboxLayoutFourthRow->addWidget(ui.labelMillimeterWaveRaderCommunication);
//  hboxLayoutFourthRow->addWidget(ui.labelMillimeterWaveRaderCommunicationState);
//  hboxLayoutFourthRow->addStretch();
//  hboxLayoutFourthRow->addWidget(ui.labelSatelliteStatus);
//  hboxLayoutFourthRow->addWidget(ui.labelSatelliteStatusState);
//  hboxLayoutFourthRow->addStretch();

  gridLayout->addLayout(hboxLayoutFirstRow, 0, 0, 1, 1);
  gridLayout->addLayout(hboxLayoutSecondRow, 1, 0, 1, 1);

  QVBoxLayout* vboxLayout = new QVBoxLayout();
  vboxLayout->addStretch();
  gridLayout->addLayout(vboxLayout, 2, 0, 1, 1);

}

void FaultDetectWidget::InitSlots()
{

}

void FaultDetectWidget::SetLabelLEDState(QLabel* label, int color, int size)
{
  // 将label中的文字清空
  label->setText("");
  // 先设置矩形大小
  // 如果ui界面设置的label大小比最小宽度和高度小，矩形将被设置为最小宽度和最小高度；
  // 如果ui界面设置的label大小比最小宽度和高度大，矩形将被设置为最大宽度和最大高度；
  QString min_width = QString("min-width: %1px;").arg(size);              // 最小宽度：size
  QString min_height = QString("min-height: %1px;").arg(size);            // 最小高度：size
  QString max_width = QString("max-width: %1px;").arg(size);              // 最小宽度：size
  QString max_height = QString("max-height: %1px;").arg(size);            // 最小高度：size
  // 再设置边界形状及边框
  QString border_radius = QString("border-radius: %1px;").arg(size / 2);    // 边框是圆角，半径为size/2
  QString border = QString("border:1px solid black;");                    // 边框为1px黑色
  // 最后设置背景颜色
  QString background = "background-color:";
  switch (color) {
  case 0:
    // 灰色
    background += "rgb(190,190,190)";
    break;
  case 1:
    // 红色
    background += "rgb(255,0,0)";
    break;
  case 2:
    // 绿色
    background += "rgb(0,255,0)";
    break;
  case 3:
    // 黄色
    background += "rgb(255,255,0)";
    break;
  default:
    break;
  }

  const QString SheetStyle = min_width + min_height + max_width + max_height + border_radius + border + background;
  label->setStyleSheet(SheetStyle);
  label->setFixedSize(QSize(10,10));
}

void FaultDetectWidget::SlotFunctionReceiveFaultDetectMsg(const hmi::FaultDiagnosisInterface faultDetectMsg)
{
  if(faultDiagnosisMsg == faultDetectMsg)
  {
    return;
  }
  else
  {
    faultDiagnosisMsg = faultDetectMsg;
   }
  {
    if(!faultDetectMsg.Gps_state_fault)
    {
        SetLabelLEDState(ui.labelGpsState,StateColor::RED);
    }
    if(faultDetectMsg.Gps_state_fault)
    {
        SetLabelLEDState(ui.labelGpsState,StateColor::GREEN);
    }

    if(!faultDetectMsg.can_state)
    {
        SetLabelLEDState(ui.labelCanState,StateColor::RED);
    }
    if(faultDetectMsg.can_state)
    {
        SetLabelLEDState(ui.labelCanState,StateColor::GREEN);
    }


//    if(!faultDetectMsg.cotton_box_driver_fault)
//    {
//        SetLabelLEDState(ui.labelCottonBoxDriveState,StateColor::RED);
//    }
//    else
//    {
//        SetLabelLEDState(ui.labelCottonBoxDriveState,StateColor::GREEN);
//    }

//    if(!faultDetectMsg.cotton_box_full_moniter_fault)
//    {
//        SetLabelLEDState(ui.labelCottonBoxFullLoadedState,StateColor::RED);
//    }
//    else
//    {
//        SetLabelLEDState(ui.labelCottonBoxFullLoadedState,StateColor::GREEN);
//    }
  }

  {
//    if(!faultDetectMsg.search_stars_unenough_fault)
//    {
//        SetLabelLEDState(ui.labelSearchStarsState,StateColor::RED);
//    }
//    else
//    {
//        SetLabelLEDState(ui.labelSearchStarsState,StateColor::GREEN);
//    }

//    if(!faultDetectMsg.brake_fault)
//    {
//        SetLabelLEDState(ui.labelBrakeByWireState,StateColor::RED);
//    }
//    else
//    {
//        SetLabelLEDState(ui.labelBrakeByWireState,StateColor::GREEN);
//    }

//    if(!faultDetectMsg.Steering_fault)
//    {
//        SetLabelLEDState(ui.labelSteerByWireState,StateColor::RED);
//    }
//    else
//    {
//        SetLabelLEDState(ui.labelSteerByWireState,StateColor::GREEN);
//    }

//    if(!faultDetectMsg.vision_detect_fault)
//    {
//        SetLabelLEDState(ui.labelVisionInspectionState,StateColor::RED);
//    }
//    else
//    {
//        SetLabelLEDState(ui.labelVisionInspectionState,StateColor::GREEN);
//    }
  }

  //  {
  //    if(!faultDetectMsg.radar_state_fault)
  //    {
  //        SetLabelLEDState(ui.labelMillimeterWaveRadarState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelMillimeterWaveRadarState,StateColor::GREEN);
  //    }

  //    if(!faultDetectMsg.head_driver_fault)
  //    {
  //        SetLabelLEDState(ui.labelHeadDriveState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelHeadDriveState,StateColor::GREEN);
  //    }

  //    if(!faultDetectMsg.vcu_communication_fault)
  //    {
  //        SetLabelLEDState(ui.labelVCUCommunicationState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelVCUCommunicationState,StateColor::GREEN);
  //    }

  //    if(!faultDetectMsg.public_communication_fault)
  //    {
  //        SetLabelLEDState(ui.labelPublicNetworkCommunicationState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelPublicNetworkCommunicationState,StateColor::GREEN);
  //    }
  //  }

  //  {
  //    if(faultDetectMsg.system_state!=2)
  //    {
  //        SetLabelLEDState(ui.labelInertialNavigationModeState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelInertialNavigationModeState,StateColor::GREEN);
  //    }

  //    if(faultDetectMsg.satellite_status==0)
  //    {
  //        SetLabelLEDState(ui.labelSatelliteStatusState,StateColor::RED);
  //    }
  //    else if(faultDetectMsg.satellite_status==4)
  //    {
  //        SetLabelLEDState(ui.labelSatelliteStatusState,StateColor::GREEN);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelSatelliteStatusState,StateColor::YELLOW);
  //    }

  //    if(!faultDetectMsg.gps_communication_fault)
  //    {
  //        SetLabelLEDState(ui.labelInertialNavigationCommunicationState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelInertialNavigationCommunicationState,StateColor::GREEN);
  //    }

  //    if(!faultDetectMsg.radar_communication_fault)
  //    {
  //        SetLabelLEDState(ui.labelMillimeterWaveRaderCommunicationState,StateColor::RED);
  //    }
  //    else
  //    {
  //        SetLabelLEDState(ui.labelMillimeterWaveRaderCommunicationState,StateColor::GREEN);
  //    }
  //  }
}

void FaultDetectWidget::SlotFunctionReceiveSelfCheckMsg(const std_msgs::String selfCheckMsg)
{
  /*"{\"allow_auto\": true, \"detect_state\": {\"steering_state\": 1, \"EBS_state\": 1,\
  \ \"brake_state\": 1, \"front_door_state\": 0, \"middle_door_state\": 0}}"*/
  QString qstrMsg = QString::fromStdString(selfCheckMsg.data);
  qstrMsg = qstrMsg.remove("\"");
  qstrMsg = qstrMsg.remove("{");
  qstrMsg = qstrMsg.remove("}");
  qstrMsg = qstrMsg.remove(",");
  QList<QString> listQstrMsg = qstrMsg.split(":");
//  for(int i=0;i<listQstrMsg.size();++i)
//  {
//    qDebug()<<listQstrMsg[i];
//  }
  bool autoEnable = listQstrMsg[1].split(" ")[0].toInt();
  int steeringState = listQstrMsg[3].split(" ")[0].toInt();
  int ebsState = listQstrMsg[4].split(" ")[0].toInt();
  int brakeState = listQstrMsg[5].split(" ")[0].toInt();
  int frontDoorState = listQstrMsg[6].split(" ")[0].toInt();
  int middleDoorState = listQstrMsg[7].split(" ")[1].toInt();

  {
    if(autoEnable)
    {
        SetLabelLEDState(ui.labelAutoDriveEnableState,StateColor::GREEN);
    }
    if(!autoEnable)
    {
        SetLabelLEDState(ui.labelAutoDriveEnableState,StateColor::RED);
    }
     //1为自动驾驶，4为手动模式，5为人工介入模式，6为警告模式，7为错误，8为力矩叠加
    if(steeringState == 1)
    {
        SetLabelLEDState(ui.labelSteeringState,StateColor::GREEN);
    }
    if(steeringState == 4 || steeringState == 7 || steeringState == 8)
    {
        SetLabelLEDState(ui.labelSteeringState,StateColor::RED);
    }
    if(steeringState == 5 || steeringState == 6)
    {
        SetLabelLEDState(ui.labelSteeringState,StateColor::YELLOW);
    }

    if(ebsState)
    {
        SetLabelLEDState(ui.labelEbsState,StateColor::RED);
    }
    if(!ebsState)
    {
        SetLabelLEDState(ui.labelEbsState,StateColor::GREEN);
    }

    if(brakeState)
    {
        SetLabelLEDState(ui.labelBrakeState,StateColor::RED);
    }
    if(!brakeState)
    {
        SetLabelLEDState(ui.labelBrakeState,StateColor::GREEN);
    }

    if(frontDoorState)
    {
        SetLabelLEDState(ui.labelFrontDoorState,StateColor::RED);
    }
    if(!frontDoorState)
    {
        SetLabelLEDState(ui.labelFrontDoorState,StateColor::GREEN);
    }

    if(middleDoorState)
    {
        SetLabelLEDState(ui.labelMiddleDoorState,StateColor::RED);
    }
    if(!middleDoorState)
    {
        SetLabelLEDState(ui.labelMiddleDoorState,StateColor::GREEN);
    }
  }
}
