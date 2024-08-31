#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPaintEvent>

#include <hmi/GlobalPathPlanningInterface.h>
#include <hmi/GpsImuInterface.h>

class PathDrawWidget :public QWidget
{
  Q_OBJECT
public:
  explicit PathDrawWidget(QWidget* parent = nullptr);
  ~PathDrawWidget();

public:
  void Init();
  void SetGlobalPathPlanningPoints(const hmi::GlobalPathPlanningInterface globalPathPlanningMsg);
  void SetCarGpsPos(const hmi::GpsImuInterface gpsImuMsg);

public slots:

signals:

protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();

  void DrawRowLines(QPainter* painter);
  void DrawColumnLines(QPainter* painter);

  void DrawCoordinateAxis(QPainter* painter);
  void DrawCoordinateValue(QPainter* painter);

  void DrawGlobalPathPlanningPoint(QPainter* painter);
  void DrawCarPos(QPainter* painter);
private:

  //参数从ros获取值
  int coordinateAccuracy;
  int coordinateIntervalX;
  int coordinateIntervalY;
  int drawLineGap;
  int startX;
  int startY;
  int pointSize;
  QList<QPair<double,double>> listGlobalPathPlanningPoint;
  QPointF carPosPoint;
  QPointF carOldPosPoint;
};
