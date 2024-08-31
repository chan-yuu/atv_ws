#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QPaintEvent>
#include <QDateTime>
#include "hmi/HmiStartEndPointInterface.h"
class InstructionDeliverWidget :public QWidget
{
  Q_OBJECT
public:
  explicit InstructionDeliverWidget(QWidget* parent = nullptr);
  ~InstructionDeliverWidget();

  void Init();
//  void SetStatrLiadrFlag(const bool flag);
//  bool GetStatrLiadrFlag();

public slots:

private slots:
    void OnActionRoadDeliver();
    void OnActionOneStepStart();
    void OnActionOneStepStop();
    void OnActionStartLidar();

signals:
    void SIGInstructionDeliverMsg(const hmi::HmiStartEndPointInterface);
    void SIGMapFileToPathDrawWidget(const QString filePath);
protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();

  hmi::HmiStartEndPointInterface CreateHmiMsg();
private:

  //bool startLidarFlag;
  struct InstructionDeliverMembers
  {
    QLabel* labelRoadId;
    QLineEdit* lineEditRoadId;
    QLabel* labelLaneId;
    QLineEdit* lineEditLaneId;

    QPushButton* pushButtonRoadDeliver;

    QPushButton* pushButtonOneStepStart;
    QPushButton* pushButtonOneStepStop;
    QPushButton* pushButtonStartLidar;

  };

  InstructionDeliverMembers ui;
};
