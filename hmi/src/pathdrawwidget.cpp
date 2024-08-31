#include "pathdrawwidget.h"
#include "msgprocesstools.h"


#include <QLayout>
#include <QPainter>
#include <QDebug>
#include <sstream>
#include <string>



PathDrawWidget::PathDrawWidget(QWidget* parent)
  :QWidget(parent)
{

}

PathDrawWidget::~PathDrawWidget()
{

}

void PathDrawWidget::paintEvent(QPaintEvent* event)
{
  QWidget::paintEvent(event);
  QPen pen = QPen(Qt::gray);
  pen.setWidth(1);
  QPainter painter(this);
  painter.setPen(pen);
  painter.setWindow(0, 0, this->width(), this->height());
  DrawRowLines(&painter);
  DrawColumnLines(&painter);
  DrawCoordinateAxis(&painter);
  DrawCoordinateValue(&painter);
  DrawGlobalPathPlanningPoint(&painter);
  DrawCarPos(&painter);
}

void PathDrawWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

void PathDrawWidget::InitMember()
{
  this->coordinateAccuracy = 10;
  this->coordinateIntervalX = 5;
  this->coordinateIntervalY = 10;
  this->drawLineGap = 30;
  this->startX = 20;
  this->startY = 20;
  this->pointSize=2980;
}

void PathDrawWidget::InitView()
{
  QPalette palette;
  palette.setColor(QPalette::Background, QColor(Qt::white));
  this->setPalette(palette);
  this->setAutoFillBackground(true);
}

void PathDrawWidget::InitSlots()
{

}

void PathDrawWidget::DrawRowLines(QPainter* painter)
{
  int lineCount = 0;
  int increaseY = 0;


  while (true)
  {
    painter->drawLine(QPoint(startX, increaseY), QPoint(this->width(), increaseY));

    increaseY += this->drawLineGap;

    if (increaseY >= this->height() - drawLineGap)
    {
      break;
    }
  }
}

void PathDrawWidget::DrawColumnLines(QPainter* painter)
{
  int lineCount = 0;
  int increaseX = startX;

  while (true)
  {
    painter->drawLine(QPoint(increaseX, 0), QPoint(increaseX, this->height() - startY));

    increaseX += this->drawLineGap;
    if (increaseX >= this->width())
    {
      break;
    }
  }
}


void PathDrawWidget::DrawCoordinateAxis(QPainter* painter)
{
  painter->setWindow(0, height(), width(), -height());

  QPoint axisStartPoint;
  QPoint axisXEndPoint; // x 轴终点
  QPoint axisYEndPoint; // y 轴终点

  axisStartPoint.setX(startX);
  axisStartPoint.setY(startY);

  axisXEndPoint.setX(startX + width());
  axisXEndPoint.setY(startY);

  axisYEndPoint.setX(startX);
  axisYEndPoint.setY(startY + height());

  painter->drawLine(axisStartPoint, axisXEndPoint);
  painter->drawLine(axisStartPoint, axisYEndPoint);
}

void PathDrawWidget::DrawCoordinateValue(QPainter* painter)
{
  QFont font;
  font.setPointSize(4);
  painter->setFont(font);
  int axisXValue = 25;
  int axisYValue = 390;

  // 必须恢复原来的坐标系，不然文字会镜像
  painter->setWindow(0, 0, this->width(), this->height());
  QPen penDrawLine = painter->pen();
  QPen penDrawXY;
  penDrawXY.setColor(Qt::black);
  penDrawXY.setWidth(1);
  painter->setPen(penDrawXY);
  painter->drawText(QPoint(width()/2,height()-5),QString("X coordinate"));
  painter->drawText(QPoint(width()-20,height()-5),QString("+3.886e5"));
  painter->drawText(QPoint(0,height()/2),QString("Y"));
  painter->drawText(QPoint(0,10),QString("+4.963e6"));
  painter->setPen(penDrawLine);

  //横坐标值
  painter->drawText(QPoint(startX,height()-10), QString("0"));
  for (int i = startX+drawLineGap; i <= width() + drawLineGap; i = i + drawLineGap)
  {
    QString strAxisXValue = QString::number(axisXValue);
    QPoint temp;
    temp.setX(i-3);
    temp.setY(height() - 10);
    painter->drawText(temp, strAxisXValue);
    axisXValue = axisXValue + coordinateIntervalX;
  }

//  painter->drawText(QPoint(startX,height()-10), QString("0"));
//  for (int i = startX+drawLineGap; i <= width() + drawLineGap; i = i + drawLineGap)
//  {
//    QString strAxisXValue = QString::number(axisXValue);
//    QPoint temp;
//    temp.setX(i);
//    temp.setY(height() - 10);
//    painter->drawText(temp, strAxisXValue);
//    axisXValue = axisXValue + coordinateIntervalX;
//  }

  //纵坐标值
  for (int i = 65 ; i <= height() + drawLineGap; i = i + drawLineGap)
  {
    QString strAxisYValue = QString::number(axisYValue);
    QPoint temp;
    temp.setX(10);
    temp.setY(height() - i + drawLineGap);

    painter->drawText(temp, strAxisYValue);
    axisYValue = axisYValue + coordinateIntervalY;
  }
//  for (int i = 38 ; i <= height() + drawLineGap; i = i + drawLineGap)
//  {
//    QString strAxisYValue = QString::number(axisYValue);
//    QPoint temp;
//    temp.setX(10);
//    temp.setY(height() - i + drawLineGap);

//    painter->drawText(temp, strAxisYValue);
//    axisYValue = axisYValue + coordinateIntervalY;
//  }
}

void PathDrawWidget::DrawGlobalPathPlanningPoint(QPainter* painter)
{
  painter->setWindow(0, this->height(), this->width(), -(this->height()));
  painter->translate(startX,startY);
  QPen pen;
  pen.setColor(Qt::blue);
  pen.setWidth(3);
  painter->setPen(pen);
  //坐标 x y 减去一定的数 然后x/(width()-startX) y/(height()-startY)
  //拿到转换后的xy 然后画图
  QList<QPointF> listConvertedPoints;
  for(int i=0;i<listGlobalPathPlanningPoint.size();++i)
  {
    QPair<double,double> pointD = listGlobalPathPlanningPoint[i];
    QPointF convertedPointF;
    convertedPointF.setX((pointD.first-388600-10)*drawLineGap/coordinateIntervalX-60);
    convertedPointF.setY((pointD.second-4963000-387.5)*drawLineGap/coordinateIntervalY+30);
    listConvertedPoints.append(convertedPointF);
  }
  for(int i=0;i<listConvertedPoints.size();++i)
  {
    painter->drawPoint(listConvertedPoints[i]);
  }
}

void PathDrawWidget::DrawCarPos(QPainter* painter)
{
  QPen pen;
  pen.setColor(Qt::red);
  pen.setWidth(5);
  painter->setPen(pen);

  QRectF carPosRectF(carPosPoint.x(),carPosPoint.y(),3,3);
  painter->drawRect(carPosPoint.x(),carPosPoint.y(),3,3);
  carOldPosPoint = carPosPoint;
  auto originMode =painter->compositionMode();
  painter->setCompositionMode(QPainter::CompositionMode_Clear);
  painter->eraseRect(carOldPosPoint.x(),carOldPosPoint.y(),3,3);
  painter->setCompositionMode(originMode);
}

void PathDrawWidget::SetGlobalPathPlanningPoints(const hmi::GlobalPathPlanningInterface globalPathPlanningMsg)
{
  std::vector<double> vectorPoints;

  for(int i=0;i<globalPathPlanningMsg.routedata.size();)
  {
    QPair<double,double> pointD;
    std::istringstream issX(globalPathPlanningMsg.routedata[i]);
    std::istringstream issY(globalPathPlanningMsg.routedata[i+1]);
    issX>>pointD.first;
    issY>>pointD.second;
    listGlobalPathPlanningPoint.append(pointD);
    i+=5;
  }
  QWidget::update();
}

void PathDrawWidget::SetCarGpsPos(const hmi::GpsImuInterface gpsImuMsg)
{
  carPosPoint.setX((gpsImuMsg.x-388600-10)*drawLineGap/coordinateIntervalX-60);
  carPosPoint.setY((gpsImuMsg.y-4963000-384)*drawLineGap/coordinateIntervalY+30);
  QWidget::update();
}
