#include "carmoniterwidget.h"
#include "pathdrawwidget.h"
#include "matplotpathdrawwidget.h"
#include "msgprocesstools.h"

#include<QLayout>
#include <QPainter>
#include <QPaintEvent>
#include <QMainWindow>


CarMoniterWidget::CarMoniterWidget(QWidget* parent)
  :QWidget(parent)
{

}

CarMoniterWidget::~CarMoniterWidget()
{

}

void CarMoniterWidget::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  QPen pen;
  pen.setColor(Qt::gray);
  pen.setWidth(1);
  painter.setPen(pen);
  painter.drawRect(this->rect().x(), this->rect().y(), this->rect().width() - 1, this->rect().height() - 1);
  QWidget::paintEvent(event);
}


void CarMoniterWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

void CarMoniterWidget::InitMember()
{
  QFont font;
  font.setPointSize(font_size);
  //ui.pathDrawWidget = new PathDrawWidget(this);
  //ui.pathDrawWidget->Init();
  ui.matplotPathDrawWidget  = new MatplotPathDrawWidget();
  ui.matplotPathDrawWidget->Init();

  ui.labelLongitude = new QLabel("经度:");
  ui.labelLongitude->setFont(font);
  ui.labelLatitude = new QLabel("纬度:");
  ui.labelLatitude->setFont(font);
  ui.labelWorkSpeed = new QLabel("车辆速度:");
  ui.labelWorkSpeed->setFont(font);
  ui.labelWorkHeading = new QLabel("车辆航向:");
  ui.labelWorkHeading->setFont(font);
  ui.labelTrackingDeviation = new QLabel("横向偏差:");
  ui.labelTrackingDeviation->setFont(font);
  ui.labelHeadingDeviation = new QLabel("航向偏差:");
  ui.labelHeadingDeviation->setFont(font);

  ui.doubleSpinBoxLongitude = new QDoubleSpinBox();
  ui.doubleSpinBoxLongitude->setReadOnly(true);
  ui.doubleSpinBoxLongitude->setRange(0, 180);
  ui.doubleSpinBoxLongitude->setDecimals(4);
  ui.doubleSpinBoxLongitude->setSingleStep(0.0001);
  ui.doubleSpinBoxLongitude->setStyleSheet("QDoubleSpinBox::up-button,QDoubleSpinBox::down-button{width:0px;}");
  ui.doubleSpinBoxLatitude = new QDoubleSpinBox();
  ui.doubleSpinBoxLatitude->setReadOnly(true);
  ui.doubleSpinBoxLatitude->setRange(0, 180);
  ui.doubleSpinBoxLatitude->setDecimals(4);
  ui.doubleSpinBoxLatitude->setSingleStep(0.0001);
  ui.doubleSpinBoxLatitude->setStyleSheet("QDoubleSpinBox::up-button,QDoubleSpinBox::down-button{width:0px;}");


  ui.lineEditWorkSpeed = new QLineEdit();
  ui.lineEditWorkSpeed->setAlignment(Qt::AlignCenter);
  ui.lineEditWorkSpeed->setFont(font);
  ui.lineEditWorkHeading = new QLineEdit();
  ui.lineEditWorkHeading->setAlignment(Qt::AlignCenter);
  ui.lineEditWorkHeading->setFont(font);
  ui.lineEditTrackingDeviation = new QLineEdit();
  ui.lineEditTrackingDeviation->setAlignment(Qt::AlignCenter);
  ui.lineEditTrackingDeviation->setFont(font);
  ui.lineEditHeadingDeviation= new QLineEdit();
  ui.lineEditHeadingDeviation->setAlignment(Qt::AlignCenter);
  ui.lineEditHeadingDeviation->setFont(font);
  ui.lineEditWorkSpeed->setReadOnly(true);
  ui.lineEditWorkHeading->setReadOnly(true);
  ui.lineEditTrackingDeviation->setReadOnly(true);
  ui.lineEditHeadingDeviation->setReadOnly(true);
}

void CarMoniterWidget::InitView()
{
  QGridLayout* gridLayout = new QGridLayout(this);

  //gridLayout->addWidget(ui.pathDrawWidget, 0, 0, 1, 4);
  gridLayout->addWidget(ui.matplotPathDrawWidget, 0, 0, 1, 4);

  gridLayout->addWidget(ui.labelLongitude, 1, 0, 1, 1);
  gridLayout->addWidget(ui.doubleSpinBoxLongitude, 1, 1, 1, 1);
  gridLayout->addWidget(ui.labelLatitude, 1, 2, 1, 1);
  gridLayout->addWidget(ui.doubleSpinBoxLatitude, 1, 3, 1, 1);

  gridLayout->addWidget(ui.labelWorkSpeed, 2, 0, 1, 1);
  gridLayout->addWidget(ui.lineEditWorkSpeed, 2, 1, 1, 1);
  gridLayout->addWidget(ui.labelWorkHeading, 2, 2, 1, 1);
  gridLayout->addWidget(ui.lineEditWorkHeading, 2, 3, 1, 1);

  gridLayout->addWidget(ui.labelTrackingDeviation, 3, 0, 1, 1);
  gridLayout->addWidget(ui.lineEditTrackingDeviation, 3, 1, 1, 1);
  gridLayout->addWidget(ui.labelHeadingDeviation, 3, 2, 1, 1);
  gridLayout->addWidget(ui.lineEditHeadingDeviation, 3, 3, 1, 1);

  gridLayout->setRowStretch(0, 5);
  gridLayout->setRowStretch(1, 1);
  gridLayout->setRowStretch(2, 1);
  gridLayout->setRowStretch(3, 1);
}

void CarMoniterWidget::InitSlots()
{

}

void CarMoniterWidget::SlotFunctionReceiveGpsImuMsg(const hmi::GpsImuInterface msg)
{
  QString workHeadingQStr= QString::number(msg.yaw);
  ui.lineEditWorkHeading->setText(workHeadingQStr);

  QString workSpeedQStr=QString::number(msg.Vel, 'f', 3)+"km/h";
  ui.lineEditWorkSpeed->setText(workSpeedQStr);
  //ui.pathDrawWidget->SetCarGpsPos(msg);
  ui.matplotPathDrawWidget->SetCarGpsPos(msg);
  ui.doubleSpinBoxLongitude->setValue(msg.lon);
  ui.doubleSpinBoxLatitude->setValue(msg.lat);
}

void CarMoniterWidget::SlotFunctionReceiveGlobalPathPlanningMsg(const hmi::GlobalPathPlanningInterface msg)
{
  //ui.pathDrawWidget->SetGlobalPathPlanningPoints(msg);
  ui.matplotPathDrawWidget->SetGlobalPathPlanningPoints(msg);
}

void CarMoniterWidget::SlotFunctionReceivePathSpeedCtrlDeliverMsg(const hmi::PathSpeedCtrlInterface msg)
{
  QString trackingDeviationQStr=QString::number(msg.CTE);
  QString headingDeviationQStr = QString::number(msg.dHead);

  ui.lineEditTrackingDeviation->setText(trackingDeviationQStr);
  ui.lineEditHeadingDeviation->setText(headingDeviationQStr);
}

void CarMoniterWidget::SlotFunctionReceiveMapFilePath(const QString filePath)
{
  ui.matplotPathDrawWidget->SetMapFilPath(filePath);
}
