#include <QWidget>
#include <QPaintEvent>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>


class RvizModuleWidget :public QWidget
{
  Q_OBJECT
public:
  explicit RvizModuleWidget(QWidget* parent = nullptr);
  ~RvizModuleWidget();

public:
  void Init();

public slots:

signals:

protected:
  void paintEvent(QPaintEvent* event) override;
private:
  void InitMember();
  void InitView();
  void InitSlots();
private:
  rviz::RenderPanel* render_panel_;
  rviz::VisualizationManager* visualization_manager_;
  //参数从ros获取值
};
