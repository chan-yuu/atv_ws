#include "matplotpathdrawwidget.h"
#include "msgprocesstools.h"
#include "qcustomplot.h"


#include <QLayout>
#include <QPainter>
#include <QDebug>
#include <sstream>
#include <string>
#include <QDir>
#include <QApplication>
#include <QFile>

MatplotPathDrawWidget::MatplotPathDrawWidget(QWidget* parent)
  :QWidget(parent)
{

}

MatplotPathDrawWidget::~MatplotPathDrawWidget()
{

}

void MatplotPathDrawWidget::paintEvent(QPaintEvent* event)
{
  QWidget::paintEvent(event);

}

void MatplotPathDrawWidget::resizeEvent(QResizeEvent* event)
{
  QWidget::resizeEvent(event);
}

void MatplotPathDrawWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

void MatplotPathDrawWidget::InitMember()
{
  imagePath = QImage(width(), height(), QImage::Format_RGB888);
  imagePath.fill(Qt::white);
  stopFlag = 1;

  ui.customPlot = new QCustomPlot();
  ui.customPlot->startTimer(1000);
  // add two new graphs and set their look:
  ui.customPlot->addGraph();
  // line color blue for first graph
  ui.customPlot->graph(0)->setPen(QPen(Qt::blue));
  // first graph will be filled with translucent blue
  //ui.customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20)));
  // line color blue for second graph
  ui.customPlot->addGraph();
  // line color red for second graph
  ui.customPlot->graph(1)->setPen(QPen(Qt::red));
  ui.customPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
  ui.customPlot->graph(1)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ScatterShape::ssDisc,5));
  // generate some points of data (y0 for first, y1 for second graph):

  // configure right and top axis to show ticks but no labels:
  // (see QCPAxisRect::setupFullAxesBox for a quicker method to do this)
  ui.customPlot->xAxis2->setVisible(true);
  ui.customPlot->xAxis2->setTickLabels(false);
  ui.customPlot->yAxis2->setVisible(true);
  ui.customPlot->yAxis2->setTickLabels(false);
  // make left and bottom axes always transfer their ranges to right and top axes:
  connect(ui.customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui.customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(ui.customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui.customPlot->yAxis2, SLOT(setRange(QCPRange)));
  // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
  ui.customPlot->graph(0)->rescaleAxes();
  // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
  ui.customPlot->graph(1)->rescaleAxes(true);
  // Note: we could have also just called customPlot->rescaleAxes(); instead
  // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
  ui.customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void MatplotPathDrawWidget::InitView()
{
  QPalette palette;
  palette.setColor(QPalette::Background, QColor(Qt::white));
  this->setPalette(palette);
  this->setAutoFillBackground(true);

  QGridLayout* gridLayout = new QGridLayout(this);
  gridLayout->addWidget(ui.customPlot,0,0,1,1);
}

void MatplotPathDrawWidget::InitSlots()
{

}

void MatplotPathDrawWidget::SetGlobalPathPlanningPoints(const hmi::GlobalPathPlanningInterface globalPathPlanningMsg)
{
  if(globalPathPlanningMsg.plan_over && stopFlag)
  {
    vectorPathPosX.clear();
    vectorPathPosY.clear();
    stopFlag = 0;
  }

  if(!globalPathPlanningMsg.plan_over)
  {
    stopFlag = 1;
  }

  for(int i=0;i<globalPathPlanningMsg.routedata.size();)
  {
    QPair<double,double> pointD;
    std::istringstream issX(globalPathPlanningMsg.routedata[i]);
    std::istringstream issY(globalPathPlanningMsg.routedata[i+1]);
    issX>>pointD.first;
    issY>>pointD.second;
    vectorPathPosX.push_back(pointD.first);
    vectorPathPosY.push_back(pointD.second);
    i+=5;
  }
  RenewPathPlot();
}

void MatplotPathDrawWidget::SetCarGpsPos(const hmi::GpsImuInterface gpsImuMsg)
{
  if(!vectorCarPosX.empty() && !vectorCarPosY.empty())
  {
    int oldPosX = vectorCarPosX[0];
    int oldPosY = vectorCarPosY[0];
    int newPosX = gpsImuMsg.posX;
    int newPosY = gpsImuMsg.posY;
    //    oldPosX = round(oldPosX * 10000) / 10000;
    //    oldPosY = round(oldPosY * 10000) / 10000;
    //    newPosX = round(newPosX * 10000) / 10000;
    //    newPosY = round(newPosY * 10000) / 10000;
    //if(oldPosX != newPosX && oldPosY != newPosY)
    if(abs(oldPosX - newPosX) >= 0.5 && abs(oldPosY - newPosY) >= 0.5)
    {
        RenewCarPlot();
    }
  }
  vectorCarPosX.clear();
  vectorCarPosY.clear();
  vectorCarPosX.push_back(gpsImuMsg.posX);
  vectorCarPosY.push_back(gpsImuMsg.posY);
}

void MatplotPathDrawWidget::SetMapFilPath(const QString filePath)
{
  // 打开文件
  QFile file(filePath);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    qDebug() << "Class MatplotPathDrawWidget；Function SetMapFilPath:无法打开文件："+ filePath<< file.errorString();
    return;
  }

  // 读取文件内容
  QTextStream in(&file);
  QString fileContent = in.readAll();
  QString fileDataContent = fileContent.split("\n")[1];
  QStringList listFileDataContent = fileDataContent.split("\t");
  // 关闭文件
  file.close();
  vectorPathPosX.clear();
  vectorPathPosY.clear();
  for(int i=5;i<listFileDataContent.size();)
  {
    QStringList listSingGroup = listFileDataContent[i].split(",");
    vectorPathPosX.push_back(listSingGroup[3].toDouble());
    vectorPathPosY.push_back(listSingGroup[4].toDouble());
    i+=6;
  }
  RenewPathPlot();
}

 void MatplotPathDrawWidget::RenewPathPlot()
{
  ui.customPlot->graph(0)->setData(vectorPathPosX, vectorPathPosY);
  // let the ranges scale themselves so graph 0 fits perfectly in the visible area:
  ui.customPlot->graph(0)->rescaleAxes();
  // same thing for graph 1, but only enlarge ranges (in case graph 1 is smaller than graph 0):
  ui.customPlot->graph(1)->rescaleAxes(true);
  // Note: we could have also just called customPlot->rescaleAxes(); instead
  ui.customPlot->replot();
}

void MatplotPathDrawWidget::RenewCarPlot()
{
  ui.customPlot->graph(1)->setData(vectorCarPosX, vectorCarPosY);
  ui.customPlot->graph(0)->rescaleAxes();
  ui.customPlot->graph(1)->rescaleAxes(true);
  ui.customPlot->replot();
}
