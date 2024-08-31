#include "perceivedresultwidget.h"

#include <QLayout>
#include <QPainter>
#include <QDebug>
PerceivedResultWidget::PerceivedResultWidget(QWidget* parent)
  :QWidget(parent)
{

}

PerceivedResultWidget::~PerceivedResultWidget()
{

}

void PerceivedResultWidget::paintEvent(QPaintEvent* event)
{
  QWidget::paintEvent(event);
}

void PerceivedResultWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

void PerceivedResultWidget::InitMember()
{
  ui.labelCameraImage = new QLabel();
}

void PerceivedResultWidget::InitView()
{
  QGridLayout* gridLayout = new QGridLayout(this);
  gridLayout->addWidget(ui.labelCameraImage,0,0,1,1);
}

void PerceivedResultWidget::InitSlots()
{

}

void PerceivedResultWidget::SlotFunctionReceiveCameraImageMsg(const sensor_msgs::ImageConstPtr& msg)
{
  // 将ROS消息转换为OpenCV图像格式
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  // 创建一个OpenCV图像副本用于调整色彩空间
  cv::Mat src_image = cv_image->image.clone();

  // 调整色彩空间为RGB
  cv::cvtColor(src_image, src_image, cv::COLOR_BGR2RGB);

  // 创建一个QImage对象并从调整后的OpenCV图像数据中加载
  QImage q_image(src_image.data, src_image.cols, src_image.rows, QImage::Format_RGB888);

  // 将图像转换为更高质量的格式以提高还原效果
  QImage q_high_quality_image = q_image.convertToFormat(QImage::Format_ARGB32_Premultiplied);

  // 将QImage显示在QLabel上
  QPixmap pixmap = QPixmap::fromImage(q_high_quality_image);
  ui.labelCameraImage->setPixmap(pixmap);
  ui.labelCameraImage->setScaledContents(true);
  ui.labelCameraImage->update();
}
