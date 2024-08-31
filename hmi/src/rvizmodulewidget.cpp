#include "rvizmodulewidget.h"
#include "msgprocesstools.h"


#include <QLayout>
#include <QPainter>
#include <QDebug>
#include <sstream>
#include <string>
RvizModuleWidget::RvizModuleWidget(QWidget* parent)
  :QWidget(parent)
{
    render_panel_= new rviz::RenderPanel;

    //设置布局
    QVBoxLayout *vboxLayout= new QVBoxLayout(this);
    vboxLayout->addWidget(render_panel_);
    //初始化rviz控制对象
    visualization_manager_= new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(visualization_manager_->getSceneManager(),visualization_manager_);
    //初始化camera 这行代码实现放大 缩小 平移等操作

    //显示
    visualization_manager_->initialize();
    visualization_manager_->removeAllDisplays();
    visualization_manager_->startUpdate();


    rviz::Display* grid_ = visualization_manager_->createDisplay("rviz/Grid","adjustable grid", true);
    grid_->subProp( "Line Style" )->subProp( "Line Width")->setValue("0.0299999993");
    grid_->subProp( "Line Style" )->subProp( "Value")->setValue("Lines");
    grid_->subProp( "Color" )->setValue(QColor(160,160,164));
    grid_->subProp( "Normal Cell Count" )->setValue("0");
    grid_->subProp( "Offset" )->subProp( "X")->setValue("100");
    grid_->subProp( "Offset" )->subProp( "Y")->setValue("0");
    grid_->subProp( "Offset" )->subProp( "Z")->setValue("0");
    grid_->subProp( "Plane" )->setValue("XY");
    grid_->subProp( "Plane Cell Count" )->setValue("20");
    grid_->subProp( "Alpha" )->setValue("0.5");
    grid_->subProp( "Cell Size" )->setValue("10");
    visualization_manager_->addDisplay(grid_,true);

    //设置rviz坐标系
    visualization_manager_->setFixedFrame("/base_link");

    //创建一个类型为rviz/PointCloud2的图层，用于接收topic为points_map的点云数据，就是我最终底图的图层
    rviz::Display* map_=visualization_manager_->createDisplay("rviz/PointCloud2","pointCloud2",true);

    map_->subProp("Topic")->setValue("/percept_origin_rviz");
    map_->subProp("Alpha")->setValue("1");
    map_->subProp("Autocompute Intensity Bounds")->setValue("false");
    map_->subProp("Autocompute Value Bounds")->subProp("Max Value")->setValue("10");
    map_->subProp("Autocompute Value Bounds")->subProp("Min Value")->setValue("-10");
    map_->subProp("Autocompute Value Bounds")->subProp("Value")->setValue("true");
    map_->subProp("Axis")->setValue("Z");
    map_->subProp("Channel Name")->setValue("intensity");
    map_->subProp("Color")->setValue(QColor(255,255,255));
    map_->subProp("Color Transformer")->setValue("Intensity");
    map_->subProp("Decay Time")->setValue(0);
    map_->subProp("Enabled")->setValue(true);
    map_->subProp("Invert Rainbow")->setValue("true");
    map_->subProp("Max Color")->setValue(QColor(255,255,255));
    map_->subProp("Max Intensity")->setValue(120);
    map_->subProp("Min Color")->setValue(QColor(0,0,0));
    map_->subProp("Min Intensity")->setValue(-30);
    map_->subProp("Position Transformer")->setValue("XYZ");
    map_->subProp("Queue Size")->setValue("10");
    map_->subProp("Selectable")->setValue("true");
    map_->subProp("Size (Pixels)")->setValue("2");
    map_->subProp("Size (m)")->setValue("0.00999999978");
    map_->subProp("Style")->setValue("Points");
    map_->subProp("Unreliable")->setValue("false");
    map_->subProp("Use Fixed Frame")->setValue("true");
    map_->subProp("Use rainbow")->setValue("true");
    map_->subProp("Value")->setValue("true");
    visualization_manager_->addDisplay(map_,true);

    rviz::Display* markerArray_=visualization_manager_->createDisplay("rviz/MarkerArray","MarkerArray",true);
    markerArray_->subProp("Marker Topic")->setValue("/perception_info_rviz");
    markerArray_->subProp("Queue Size")->setValue("100");
    markerArray_->subProp("Namespaces")->subProp("acc_dir")->setValue(false);
    markerArray_->subProp("Namespaces")->subProp("atten_label")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("atten_polygon")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("attention")->setValue(false);
    markerArray_->subProp("Namespaces")->subProp("barrier")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("box")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("box_info")->setValue(false);
    markerArray_->subProp("Namespaces")->subProp("cube")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("cylinder")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("cylinder_box")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("label_info")->setValue(true);
    markerArray_->subProp("Namespaces")->subProp("polygon")->setValue(false);
    markerArray_->subProp("Namespaces")->subProp("track_info")->setValue(false);

    visualization_manager_->addDisplay(markerArray_,true);
}

RvizModuleWidget::~RvizModuleWidget()
{

}

void RvizModuleWidget::paintEvent(QPaintEvent* event)
{
  QWidget::paintEvent(event);
}

void RvizModuleWidget::Init()
{
  InitMember();
  InitView();
  InitSlots();
}

void RvizModuleWidget::InitMember()
{

}

void RvizModuleWidget::InitView()
{

}

void RvizModuleWidget::InitSlots()
{

}

