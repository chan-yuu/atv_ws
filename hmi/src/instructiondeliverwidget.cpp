#include "instructiondeliverwidget.h"
#include "msgprocesstools.h"

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


InstructionDeliverWidget::InstructionDeliverWidget(QWidget* parent)
  :QWidget(parent)
{

}

InstructionDeliverWidget::~InstructionDeliverWidget()
{

}

void InstructionDeliverWidget::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  QPen pen;
  pen.setColor(Qt::gray);
  pen.setWidth(1);
  painter.setPen(pen);
  painter.drawRect(this->rect().x(), this->rect().y(), this->rect().width() - 1, this->rect().height() - 1);
  QWidget::paintEvent(event);
}


void InstructionDeliverWidget::InstructionDeliverWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

//void InstructionDeliverWidget::SetStatrLiadrFlag(const bool flag)
//{
//  startLidarFlag = flag;
//}

// bool InstructionDeliverWidget::GetStatrLiadrFlag()
//{
//  return startLidarFlag;
//}

void InstructionDeliverWidget::InstructionDeliverWidget::InitMember()
{
  //startLidarFlag = false;
  QFont font;
  font.setPointSize(font_size);

  ui.labelRoadId = new QLabel("RoadId:");
  ui.labelRoadId ->setFont(font);
  ui.labelRoadId->setAlignment(Qt::AlignCenter);
  ui.lineEditRoadId = new QLineEdit();
  ui.lineEditRoadId ->setFont(font);

  ui.labelLaneId = new QLabel("StationID:");
  ui.labelLaneId ->setFont(font);
  ui.labelLaneId ->setAlignment(Qt::AlignCenter);
  ui.lineEditLaneId = new QLineEdit();
  ui.lineEditLaneId ->setFont(font);

  ui.pushButtonRoadDeliver = new QPushButton("路段信息下发");
  ui.pushButtonRoadDeliver->setFont(font);

  ui.pushButtonOneStepStart = new QPushButton("一键启动");
  ui.pushButtonOneStepStart->setFont(font);
  ui.pushButtonOneStepStop = new QPushButton("一键关闭");
  ui.pushButtonOneStepStop->setFont(font);
  ui.pushButtonStartLidar = new QPushButton("启动传感器");
  ui.pushButtonStartLidar->setFont(font);
}

void InstructionDeliverWidget::InstructionDeliverWidget::InitView()
{
  QGridLayout* gridLayout = new QGridLayout(this);

  QHBoxLayout* hboxLayoutPushButton = new QHBoxLayout();
   QHBoxLayout* hboxLayoutPushButton1 = new QHBoxLayout();

  hboxLayoutPushButton->addWidget(ui.pushButtonRoadDeliver);
  hboxLayoutPushButton->addWidget(ui.pushButtonOneStepStart);

  hboxLayoutPushButton1->addWidget(ui.pushButtonStartLidar);
  hboxLayoutPushButton1->addWidget(ui.pushButtonOneStepStop);

  gridLayout->addWidget(ui.labelRoadId, 0, 0, 1, 1);
  gridLayout->addWidget(ui.lineEditRoadId, 0, 1, 1, 1);

  gridLayout->addWidget(ui.labelLaneId, 1, 0, 1, 1);
  gridLayout->addWidget(ui.lineEditLaneId, 1, 1, 1, 1);

  gridLayout->addLayout(hboxLayoutPushButton, 2, 0, 1, 2);
  gridLayout->addLayout(hboxLayoutPushButton1, 3, 0, 1, 2);

  QVBoxLayout* vboxLayout = new QVBoxLayout();
  vboxLayout->addStretch();
  gridLayout->addLayout(vboxLayout,4,0,1,2);
  gridLayout->setColumnStretch(0, 1);
  gridLayout->setColumnStretch(1, 2);
}

void InstructionDeliverWidget::InstructionDeliverWidget::InitSlots()
{
  connect(ui.pushButtonRoadDeliver,&QPushButton::clicked,this,&InstructionDeliverWidget::OnActionRoadDeliver);
  connect(ui.pushButtonOneStepStart,&QPushButton::clicked,this,&InstructionDeliverWidget::OnActionOneStepStart);
  connect(ui.pushButtonOneStepStop,&QPushButton::clicked,this,&InstructionDeliverWidget::OnActionOneStepStop);
  connect(ui.pushButtonStartLidar,&QPushButton::clicked,this,&InstructionDeliverWidget::OnActionStartLidar);
}

hmi::HmiStartEndPointInterface InstructionDeliverWidget::CreateHmiMsg()
{
  hmi::HmiStartEndPointInterface msg;
  msg.flag = 1;
  return msg;
}

void InstructionDeliverWidget::OnActionRoadDeliver()
{
  QString strRoadId = ui.lineEditRoadId->text();
  QString strStationId = ui.lineEditLaneId->text();
  bool ok1,ok2;
  int roadId = strRoadId.toInt(&ok1,10);
  int stationId = strStationId.toInt(&ok2,10);
  if(!ok1 && !ok2)
  {
    QMessageBox::warning(this, "指令下发", "输入的RoadId和StationId必须是整数!", QMessageBox::Yes, QMessageBox::Yes);
    return;
  }
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString mapFilePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  mapFilePath = strListCurrentNodeExeDir.join("/");
  mapFilePath = mapFilePath+"/src/hmi/config/" + strRoadId + ".map";
  hmi::HmiStartEndPointInterface msg = CreateHmiMsg();
  msg.roadid = roadId;
  msg.stationid = stationId;
  emit SIGInstructionDeliverMsg(msg);
  emit SIGMapFileToPathDrawWidget(mapFilePath);
}

void InstructionDeliverWidget::OnActionOneStepStart()
{
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString shellScriptionFilePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  shellScriptionFilePath = strListCurrentNodeExeDir.join("/");
  shellScriptionFilePath = shellScriptionFilePath+"/src/self_start/one_step_start.sh";
  qDebug() << "文件路径：" << shellScriptionFilePath;
  QProcess processShell;
  processShell.startDetached(shellScriptionFilePath);
}

void InstructionDeliverWidget::OnActionOneStepStop()
{
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString shellScriptionFilePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  shellScriptionFilePath = strListCurrentNodeExeDir.join("/");
  shellScriptionFilePath = shellScriptionFilePath+"/src/self_start/one_step_stop.sh";
  qDebug() << "文件路径：" << shellScriptionFilePath;
  QProcess processShell;
  processShell.startDetached(shellScriptionFilePath);
}

void InstructionDeliverWidget::OnActionStartLidar()
{
  //startLidarFlag = true;
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString shellScriptionFilePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  shellScriptionFilePath = strListCurrentNodeExeDir.join("/");
  shellScriptionFilePath = shellScriptionFilePath+"/src/self_start/startlidar.sh";
  qDebug() << "文件路径：" << shellScriptionFilePath;
  QProcess processShell;
  bool isStart = processShell.startDetached(shellScriptionFilePath);

//  processShell.start(shellScriptionFilePath);
//  processShell.waitForFinished();  // 等待进程完成

//  int exitCode = processShell.exitCode();

//  if (exitCode != 0) {
//    qDebug() << "启动进程时发生错误。退出码：" << exitCode;
//    qDebug() << "错误信息：" << processShell.errorString();
//                                     qDebug() << "进程输出：" << processShell.readAllStandardOutput();
//                                         qDebug() << "进程错误输出：" << processShell.readAllStandardError();
//  } else {
//    qDebug() << "进程成功启动";
//  }
}
