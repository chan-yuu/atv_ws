#include <QWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QPaintEvent>

#include "hmi/GpsImuInterface.h"
class NodeManageWidget :public QWidget
{
  Q_OBJECT
public:
    explicit NodeManageWidget(QWidget* parent = nullptr);
    ~NodeManageWidget();

    void Init();
    QMap<int, QPair<double,double>> GetInsectionData();
    QMap<int, QPair<double,double>> GetSiteData();
    void InsectionReadFromFile(const QString filePath);
    void SiteReadFromFile(const QString filePath);

public slots:
    void SlotFunctionReceiveGPSImuMsg(const hmi::GpsImuInterface gpsImuMsg);

private slots:
    void OnActionAdd();
    void OnActionEdit();
    void OnActionDelete();
    void OnActionUp();
    void OnActionDown();
    void OnActionSave();

signals:

protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();

  void SwapTableWidgetQueue(int currentrow, int tagrgetrow);

  void ReadFromFile();

private:

  int startFlag;
  double posX;
  double posY;

  QMap<int, QPair<double,double>> mapInsection;
  QMap<int, QPair<double,double>> mapSite;

  struct NodeManageMembers
  {
    QTabWidget* tabWidget;
    QTableWidget* tableWidgetIntersection;
    QTableWidget* tableWidgetSite;
    QPushButton* pushButtonAdd;
    QPushButton* pushButtonEdit;
    QPushButton* pushButtonDelete;
    QPushButton* pushButtonUp;
    QPushButton* pushButtonDown;
    QPushButton* pushButtonSave;
  };

  NodeManageMembers ui;
};
