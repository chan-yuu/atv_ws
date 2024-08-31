#include "mapcollectwidget.h"
#include "msgprocesstools.h"
#include "nodemanagewidget.h"

#include <QLayout>
#include <QPainter>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <QProcess>
#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QSettings>
#include <QInputDialog>
#include <QDateTime>
#include <math.h>


MapCollectWidget::MapCollectWidget(QWidget* parent)
  :QWidget(parent)
{

}

MapCollectWidget::~MapCollectWidget()
{

}

void MapCollectWidget::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  QPen pen;
  pen.setColor(Qt::gray);
  pen.setWidth(1);
  painter.setPen(pen);
  painter.drawRect(this->rect().x(), this->rect().y(), this->rect().width() - 1, this->rect().height() - 1);
  QWidget::paintEvent(event);
}


void MapCollectWidget::MapCollectWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

NodeManageWidget* MapCollectWidget::GetNodeManagetWidget()
{
  return ui.nodeManageWidget;
}

void MapCollectWidget::MapCollectWidget::InitMember()
{
  startFlag = 0;
  clearFlag = 1;
  QFont font;
  font.setPointSize(font_size);

  ui.labelRoadId = new QLabel("RoadId:");
  ui.labelRoadId ->setFont(font);
  ui.labelRoadId->setAlignment(Qt::AlignCenter);
  ui.lineEditRoadId = new QLineEdit();
  ui.lineEditRoadId ->setFont(font);

  ui.labelLaneId = new QLabel("LaneID:");
  ui.labelLaneId ->setFont(font);
  ui.labelLaneId ->setAlignment(Qt::AlignCenter);
  ui.lineEditLaneId = new QLineEdit();
  ui.lineEditLaneId ->setFont(font);

  ui.labelLaneSelection = new QLabel("车道选择:");
  ui.labelLaneSelection ->setFont(font);
  ui.labelLaneSelection ->setAlignment(Qt::AlignCenter);
  ui.comboBoxLaneSelection = new QComboBox();
  ui.comboBoxLaneSelection ->setFont(font);
  QStringList strListLane;
  strListLane<<"Left"<<"Right";
  ui.comboBoxLaneSelection->addItems(strListLane);
  ui.comboBoxLaneSelection->setCurrentIndex(0);

  ui.labelWorkSpeed = new QLabel("作业车速:");
  ui.labelWorkSpeed ->setFont(font);
  ui.labelWorkSpeed->setAlignment(Qt::AlignCenter);
  ui.doubleSpinBoxWorkSpeed = new QDoubleSpinBox();
  ui.doubleSpinBoxWorkSpeed ->setFont(font);
  ui.doubleSpinBoxWorkSpeed->setDecimals(3);
  ui.doubleSpinBoxWorkSpeed->setSingleStep(0.001);

  ui.labelDistance = new QLabel("已采集距离:");
  ui.labelDistance ->setFont(font);
  ui.labelDistance->setAlignment(Qt::AlignCenter);
  ui.labelDistanceValue = new QLabel("0m");
  ui.labelDistanceValue ->setFont(font);

  ui.pushButtonStartCollectPoints = new QPushButton("开始采集");
  ui.pushButtonStartCollectPoints ->setFont(font);

  ui.pushButtonStopCollectPoints = new QPushButton("停止采集");
  ui.pushButtonStopCollectPoints ->setFont(font);

  ui.pushButtonClearCollectInfo = new QPushButton("清空采集");
  ui.pushButtonClearCollectInfo ->setFont(font);

  ui.pushButtonSaveCollectPoints = new QPushButton("保存采集点");
  ui.pushButtonSaveCollectPoints ->setFont(font);

  ui.pushButtonNodeManage = new QPushButton("路口和站点管理");
  ui.pushButtonNodeManage ->setFont(font);

  ui.pushButtonNodeDataInsectionImport = new QPushButton("路口数据导入");
  ui.pushButtonNodeDataInsectionImport ->setFont(font);

  ui.pushButtonNodeDataSiteImport = new QPushButton("站点数据导入");
  ui.pushButtonNodeDataSiteImport ->setFont(font);

  ui.pushButtonNodeDataDeliver = new QPushButton("路口和站点数据下发");
  ui.pushButtonNodeDataDeliver ->setFont(font);

  ui.nodeManageWidget = new NodeManageWidget();
  ui.nodeManageWidget->Init();
}

void MapCollectWidget::MapCollectWidget::InitView()
{
  QGridLayout* gridLayout = new QGridLayout(this);

  QHBoxLayout* hboxLayoutPushButton = new QHBoxLayout();
  QHBoxLayout* hboxLayoutPushButton1 = new QHBoxLayout();

  hboxLayoutPushButton->addWidget(ui.pushButtonStartCollectPoints);
  hboxLayoutPushButton->addWidget(ui.pushButtonStopCollectPoints);
  hboxLayoutPushButton->addWidget(ui.pushButtonClearCollectInfo);
  hboxLayoutPushButton->addWidget(ui.pushButtonSaveCollectPoints);
  hboxLayoutPushButton1->addWidget(ui.pushButtonNodeManage);
  hboxLayoutPushButton1->addWidget(ui.pushButtonNodeDataInsectionImport);
  hboxLayoutPushButton1->addWidget(ui.pushButtonNodeDataSiteImport);
  hboxLayoutPushButton1->addWidget(ui.pushButtonNodeDataDeliver);
  gridLayout->addWidget(ui.labelRoadId, 0, 0, 1, 1);
  gridLayout->addWidget(ui.lineEditRoadId, 0, 1, 1, 1);

  gridLayout->addWidget(ui.labelLaneId, 1, 0, 1, 1);
  gridLayout->addWidget(ui.lineEditLaneId, 1, 1, 1, 1);

  gridLayout->addWidget(ui.labelLaneSelection, 2, 0, 1, 1);
  gridLayout->addWidget(ui.comboBoxLaneSelection, 2, 1, 1, 1);

  gridLayout->addWidget(ui.labelWorkSpeed, 3, 0, 1, 1);
  gridLayout->addWidget(ui.doubleSpinBoxWorkSpeed, 3, 1, 1, 1);

  gridLayout->addWidget(ui.labelDistance, 4, 0, 1, 1);
  gridLayout->addWidget(ui.labelDistanceValue, 4, 1, 1, 1);

  gridLayout->addLayout(hboxLayoutPushButton, 5, 0, 1, 2);
  gridLayout->addLayout(hboxLayoutPushButton1, 6, 0, 1, 2);

  QVBoxLayout* vboxLayout = new QVBoxLayout();
  vboxLayout->addStretch();
  gridLayout->addLayout(vboxLayout,7,0,1,2);
  gridLayout->setColumnStretch(0, 1);
  gridLayout->setColumnStretch(1, 2);
}

void MapCollectWidget::MapCollectWidget::InitSlots()
{
  connect(ui.pushButtonStartCollectPoints,&QPushButton::clicked,this,&MapCollectWidget::OnActionStartCollectPoints);
  connect(ui.pushButtonStopCollectPoints,&QPushButton::clicked,this,&MapCollectWidget::OnActionStopCollectPoints);
  connect(ui.pushButtonClearCollectInfo,&QPushButton::clicked,this,&MapCollectWidget::OnActionClearCollectInfo);
  connect(ui.pushButtonSaveCollectPoints,&QPushButton::clicked,this,&MapCollectWidget::OnActionSaveCollectPoints);
  connect(ui.pushButtonNodeManage,&QPushButton::clicked,this,&MapCollectWidget::OnActionNodeManage);
  connect(ui.pushButtonNodeDataInsectionImport,&QPushButton::clicked,this,&MapCollectWidget::OnActionNodeDataInsectionImport);
  connect(ui.pushButtonNodeDataSiteImport,&QPushButton::clicked,this,&MapCollectWidget::OnActionNodeDataSiteImport);
  connect(ui.pushButtonNodeDataDeliver,&QPushButton::clicked,this,&MapCollectWidget::OnActionNodeDataDeliver);

  connect(ui.doubleSpinBoxWorkSpeed,&QDoubleSpinBox::editingFinished,this,&MapCollectWidget::OnDoubleSpinBoxSpeedEditFinishing);
}

void MapCollectWidget::SlotFunctionReceiveGPSImuMsg(const hmi::GpsImuInterface gpsImuMsg)
{
  if(!startFlag)
  {
      return;
  }

  if(clearFlag)
  {
    startPoint.setX(gpsImuMsg.x);
    startPoint.setY(gpsImuMsg.y);
    clearFlag = 0;
  }

  double distance = std::sqrt(std::pow(gpsImuMsg.x - startPoint.x(), 2) + std::pow(gpsImuMsg.y - startPoint.y(), 2));

  if(distance >= 0.1)
  {
    vectorPosLo.push_back(gpsImuMsg.lon);
    vectorPosLa.push_back(gpsImuMsg.lat);
    vectorPosHeading.push_back(gpsImuMsg.yaw);
    vectorPosX.push_back(gpsImuMsg.x);
    vectorPosY.push_back(gpsImuMsg.y);
    vectorSpeed.push_back(ui.doubleSpinBoxWorkSpeed->value());
    QString strDistacneValue = ui.labelDistanceValue->text();
    double value = strDistacneValue.split("m")[0].toDouble();
    value = value + distance;
    ui.labelDistanceValue->setText(QString::number(value) + "m");
    qDebug()<<"SSS:"<<startPoint<<"\t"<<"EEEE:"<<gpsImuMsg.x<<"\t"<<gpsImuMsg.y;
    startPoint.setX(gpsImuMsg.x);
    startPoint.setY(gpsImuMsg.y);
  }
}

void MapCollectWidget::OnActionStartCollectPoints()
{
  double speed = ui.doubleSpinBoxWorkSpeed->value();
  if(speed == 0)
  {
    QMessageBox::warning(this, "开始采集", "请输入作业车速!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
  startFlag = 1;
}

void MapCollectWidget::OnActionStopCollectPoints()
{
  startFlag = 0;
}

void  MapCollectWidget::OnActionClearCollectInfo()
{
  clearFlag = 1;
  ui.labelDistanceValue->setText("0m");
  vectorPosLo.clear();
  vectorPosLa.clear();
  vectorPosHeading.clear();
  vectorPosX.clear();
  vectorPosY.clear();
  vectorSpeed.clear();
}

void MapCollectWidget::OnActionSaveCollectPoints()
{
  startFlag = 0;
  //ui.labelDistanceValue->setText("0m");
  QString roadId = ui.lineEditRoadId->text();
  QString laneId = ui.lineEditLaneId->text();
  QString speed = ui.doubleSpinBoxWorkSpeed->text();
  bool ok1,ok2;
  roadId.toInt(&ok1,10);
  laneId.toInt(&ok2,10);
  if(!ok1 && !ok2)
  {
    QMessageBox::warning(this, "地图采集", "输入的RoadId和StationId必须是整数!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
  QString laneType = ui.comboBoxLaneSelection->currentText();
  QString leftEnable = "0";
  QString rightEnable = "0";
  if(laneType == "left")
  {
    leftEnable = "1";
    rightEnable = "0";
  }
  else
  {
    leftEnable = "0";
    rightEnable = "1";
  }
  QString pos = "0";
  QString firsLine = "RoadID\tLaneID\tLeft\tRight\tPos\n";
  QString secondLine = roadId + "\t" + laneId + "\t" + leftEnable + "\t" + rightEnable + "\t" + pos + "\t";

  if(roadId.isEmpty())
  {
    QMessageBox::warning(this, "保存采集点", "请输入RoadID!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
  if(laneId.isEmpty())
  {
    QMessageBox::warning(this, "保存采集点", "请输入LaneID!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }

  if(speed.toDouble() == 0)
  {
    QMessageBox::warning(this, "保存采集点", "请输入作业车速!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }

  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString filePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  filePath = strListCurrentNodeExeDir.join("/");
  filePath = filePath + "/src/hmi/config";
  //QDateTime dateTime = QDateTime::currentDateTime();
  //QString strDataTime = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
  QString fileName = QFileDialog::getSaveFileName(this,tr("保存采集点文件"),filePath,"*.map");
  if(fileName.isEmpty())
  {
    return;
  }

  QFile file(fileName);
  file.open(QIODevice::WriteOnly);
  file.close();
  bool okFileCreate = file.open(QIODevice::ReadWrite);
  if(!okFileCreate)
  {
    QMessageBox::warning(this, "保存采集点", "保存采集点文件创建失败!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
  QTextStream stream(&file);

  stream << firsLine;
  stream << secondLine;
  for (int i = 0; i < vectorPosLo.size(); ++i)
  {
    stream << QString::number(vectorPosLo[i] ,'f', 11) + "," + QString::number(vectorPosLa[i] ,'f', 11)
                  + "," + QString::number(vectorPosHeading[i] ,'f', 11)
                  + "," + QString::number(vectorPosX[i] ,'f', 11)
                  + "," + QString::number(vectorPosY[i] ,'f', 11) +
                  + "," + QString::number(vectorSpeed[i] ,'f', 11) + "\t";
  }
  file.close();
}

void MapCollectWidget::OnActionNodeManage()
{
  ui.nodeManageWidget->show();
}

void MapCollectWidget::OnActionNodeDataInsectionImport()
{
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString filePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  filePath = strListCurrentNodeExeDir.join("/");
  filePath = filePath + "/src/hmi/config";
  //QDateTime dateTime = QDateTime::currentDateTime();
  //QString strDataTime = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
  QString fileName = QFileDialog::getOpenFileName(this,tr("导入路口数据文件"),filePath,"*.txt");
  if(fileName.isEmpty())
  {
    return;
  }

  ui.nodeManageWidget->InsectionReadFromFile(fileName);
}

void MapCollectWidget::OnActionNodeDataSiteImport()
{
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString filePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  filePath = strListCurrentNodeExeDir.join("/");
  filePath = filePath + "/src/hmi/config";
  //QDateTime dateTime = QDateTime::currentDateTime();
  //QString strDataTime = dateTime.toString("yyyy-MM-dd-hh-mm-ss");
  QString fileName = QFileDialog::getOpenFileName(this,tr("导入站点数据文件"),filePath,"*.txt");
  if(fileName.isEmpty())
  {
    return;
  }

  ui.nodeManageWidget->SiteReadFromFile(fileName);
}

void MapCollectWidget::OnActionNodeDataDeliver()
{
  QMap<int, QPair<double,double>> mapInsection = ui.nodeManageWidget->GetInsectionData();
  QMap<int, QPair<double,double>> mapSite = ui.nodeManageWidget->GetSiteData();
  hmi::NodePointsInterface msg;
  QMap<int, QPair<double,double>>::Iterator iter = mapInsection.begin();
  while(iter != mapInsection.end())
  {
    msg.incppoint.push_back(iter.value().first);
    msg.incppoint.push_back(iter.value().second);
    ++iter;
  }

  iter = mapSite.begin();
  while(iter != mapSite.end())
  {
    msg.stationpoint.push_back(iter.value().first);
    msg.stationpoint.push_back(iter.value().second);
    ++iter;
  }


  emit SIGNodeDataDeliver(msg);
}

void MapCollectWidget::OnDoubleSpinBoxSpeedEditFinishing()
{
    startFlag = 0;
}
