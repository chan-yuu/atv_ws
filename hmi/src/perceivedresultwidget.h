#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <QWidget>
#include <QApplication>
#include <QLabel>
#include <QImage>
#include <QPixmap>
class PerceivedResultWidget :public QWidget
{
  Q_OBJECT
public:
  explicit PerceivedResultWidget(QWidget* parent = nullptr);
  ~PerceivedResultWidget();

public:
  void Init();

public slots:
  void SlotFunctionReceiveCameraImageMsg(const sensor_msgs::ImageConstPtr& msg);
signals:

protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();

private:
  struct PerceivedResultMembers
  {
      QLabel* labelCameraImage;
  };
  PerceivedResultMembers ui;
};
