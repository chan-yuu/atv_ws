#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPaintEvent>

#include "hmi/FaultDiagnosisInterface.h"
#include "std_msgs/String.h"
enum StateColor :int
{
  GRAY = 0,
  RED = 1,
  GREEN = 2,
  YELLOW = 3,
};
class FaultDetectWidget :public QWidget
{
  Q_OBJECT
public:
  explicit FaultDetectWidget(QWidget* parent = nullptr);
  ~FaultDetectWidget();

  void Init();

public slots:
  void SlotFunctionReceiveFaultDetectMsg(const hmi::FaultDiagnosisInterface faultDetectMsg);
  void SlotFunctionReceiveSelfCheckMsg(const std_msgs::String selfCheckMsg);
signals:

protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();

  void SetLabelLEDState(QLabel* label, int color, int size=10);


private:
  hmi::FaultDiagnosisInterface faultDiagnosisMsg;
  struct FaultDetectMembers
  {
    QLabel* labelAutoDriveEnable;
    QLabel* labelSteering;
    QLabel* labelEbs;
    QLabel* labelBrake;
    QLabel* labelFrontDoor;
    QLabel* labelMiddleDoor;

    QLabel* labelAutoDriveEnableState;
    QLabel* labelSteeringState;
    QLabel* labelEbsState;
    QLabel* labelBrakeState;
    QLabel* labelFrontDoorState;
    QLabel* labelMiddleDoorState;

    QLabel* labelGps;
    QLabel* labelCan;

    QLabel* labelGpsState;
    QLabel* labelCanState;

    QLabel* labelInertialNavigation;
    QLabel* labelDriveByWire;
    QLabel* labelSearchStars;
    QLabel* labelBrakeByWire;
    QLabel* labelSteerByWire;
    QLabel* labelVisionInspection;
    QLabel* labelMillimeterWaveRadar;
    QLabel* labelHeadDrive;
    QLabel* labelVCUCommunication;
    QLabel* labelPublicNetworkCommunication;
    QLabel* labelInertialNavigationMode;
    QLabel* labelInertialNavigationCommunication;
    QLabel* labelMillimeterWaveRaderCommunication;
    QLabel* labelSatelliteStatus;

    QLabel* labelInertialNavigationState;
    QLabel* labelDriveByWireState;
    QLabel* labelSearchStarsState;
    QLabel* labelBrakeByWireState;
    QLabel* labelSteerByWireState;
    QLabel* labelVisionInspectionState;
    QLabel* labelMillimeterWaveRadarState;
    QLabel* labelHeadDriveState;
    QLabel* labelVCUCommunicationState;
    QLabel* labelPublicNetworkCommunicationState;
    QLabel* labelInertialNavigationModeState;
    QLabel* labelInertialNavigationCommunicationState;
    QLabel* labelMillimeterWaveRaderCommunicationState;
    QLabel* labelSatelliteStatusState;

  };

  FaultDetectMembers ui;
};
