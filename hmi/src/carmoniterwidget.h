#include <QWidget>
#include <QLabel>
#include <QLineEdit>
#include <QDoubleSpinBox>


#include "hmi/GpsImuInterface.h"
#include "hmi/GlobalPathPlanningInterface.h"
#include "hmi/PathSpeedCtrlInterface.h"
class PathDrawWidget;
class MatplotPathDrawWidget;
class CarMoniterWidget :public QWidget
{
  Q_OBJECT
public:
  explicit CarMoniterWidget(QWidget* parent = nullptr);
  ~CarMoniterWidget();

  void Init();

public slots:
  void SlotFunctionReceiveGpsImuMsg(const hmi::GpsImuInterface msg);
  void SlotFunctionReceiveGlobalPathPlanningMsg(const hmi::GlobalPathPlanningInterface msg);
  void SlotFunctionReceivePathSpeedCtrlDeliverMsg(const hmi::PathSpeedCtrlInterface msg);
  void SlotFunctionReceiveMapFilePath(const QString filePath);

signals:

protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();
private:
  struct CarMoniterMembers
  {
    //PathDrawWidget* pathDrawWidget;
    MatplotPathDrawWidget* matplotPathDrawWidget;

    QLabel* labelLongitude;
    QLabel* labelLatitude;
    QLabel* labelWorkSpeed;
    QLabel* labelWorkHeading;
    QLabel* labelTrackingDeviation;
    QLabel* labelHeadingDeviation;

    QDoubleSpinBox* doubleSpinBoxLongitude;
    QDoubleSpinBox* doubleSpinBoxLatitude;
    QLineEdit* lineEditWorkSpeed;
    QLineEdit* lineEditWorkHeading;
    QLineEdit* lineEditTrackingDeviation;
    QLineEdit* lineEditHeadingDeviation;

  };

  CarMoniterMembers ui;
};
