#include <QWidget>
#include <QLabel>
#include <QPaintEvent>
#include <QResizeEvent>

#include <hmi/GlobalPathPlanningInterface.h>
#include <hmi/GpsImuInterface.h>

class QCustomPlot;
class MatplotPathDrawWidget :public QWidget
{
  Q_OBJECT
public:
  explicit MatplotPathDrawWidget(QWidget* parent = nullptr);
  ~MatplotPathDrawWidget();

public:
  void Init();
  void SetGlobalPathPlanningPoints(const hmi::GlobalPathPlanningInterface globalPathPlanningMsg);
  void SetCarGpsPos(const hmi::GpsImuInterface gpsImuMsg);
  void SetMapFilPath(const QString filePath);

public slots:

signals:

protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();

  void RenewPathPlot();
  void RenewCarPlot();


private:
  QVector<double> vectorPathPosX;
  QVector<double> vectorPathPosY;
  QVector<double> vectorCarPosX;
  QVector<double> vectorCarPosY;
  QImage imagePath;
  int stopFlag;


  struct MatplotPathDrawMembers
  {
    QCustomPlot* customPlot;
    //QLabel* labelImage;
  };
  MatplotPathDrawMembers ui;
};
