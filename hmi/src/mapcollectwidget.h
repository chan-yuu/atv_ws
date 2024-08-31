#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QPaintEvent>
#include <QDateTime>
#include "hmi/NodePointsInterface.h"
#include "hmi/GpsImuInterface.h"
class NodeManageWidget;
class MapCollectWidget :public QWidget
{
  Q_OBJECT
public:
  explicit MapCollectWidget(QWidget* parent = nullptr);
  ~MapCollectWidget();

  void Init();
  NodeManageWidget* GetNodeManagetWidget();

public slots:
   void SlotFunctionReceiveGPSImuMsg(const hmi::GpsImuInterface gpsImuMsg);
private slots:
    void OnActionStartCollectPoints();
    void OnActionStopCollectPoints();
    void OnActionClearCollectInfo();
    void OnActionSaveCollectPoints();
    void OnActionNodeManage();
    void OnActionNodeDataInsectionImport();
    void OnActionNodeDataSiteImport();
    void OnActionNodeDataDeliver();

    void OnDoubleSpinBoxSpeedEditFinishing();

signals:
    void SIGNodeDataDeliver(const hmi::NodePointsInterface);
protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();
private:

  QVector<double> vectorPosLo;
  QVector<double> vectorPosLa;
  QVector<double> vectorPosHeading;
  QVector<double> vectorPosX;
  QVector<double> vectorPosY;
  QVector<double> vectorSpeed;
  int startFlag;
  int clearFlag;
  QPointF startPoint;

  struct MapCollectMembers
  {
    QLabel* labelRoadId;
    QLineEdit* lineEditRoadId;
    QLabel* labelLaneId;
    QLineEdit* lineEditLaneId;
    QLabel* labelLaneSelection;
    QComboBox* comboBoxLaneSelection;
    QLabel* labelWorkSpeed;
    QDoubleSpinBox* doubleSpinBoxWorkSpeed;
    QLabel* labelDistance;
    QLabel* labelDistanceValue;

    QPushButton* pushButtonStartCollectPoints;
    QPushButton* pushButtonStopCollectPoints;
    QPushButton* pushButtonClearCollectInfo;
    QPushButton* pushButtonSaveCollectPoints;
    QPushButton* pushButtonNodeManage;
    QPushButton* pushButtonNodeDataInsectionImport;
    QPushButton* pushButtonNodeDataSiteImport;
    QPushButton* pushButtonNodeDataDeliver;

    NodeManageWidget* nodeManageWidget;
  };

  MapCollectMembers ui;
};
