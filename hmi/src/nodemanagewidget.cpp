#include "nodemanagewidget.h"
#include "msgprocesstools.h"

#include <QLayout>
#include <QPainter>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <QProcess>
#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QSettings>
#include <QInputDialog>
#include <QDateTime>
#include <QHeaderView>
#include <math.h>


NodeManageWidget::NodeManageWidget(QWidget* parent)
  :QWidget(parent)
{

}

NodeManageWidget::~NodeManageWidget()
{

}

void NodeManageWidget::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  QPen pen;
  pen.setColor(Qt::gray);
  pen.setWidth(1);
  painter.setPen(pen);
  painter.drawRect(this->rect().x(), this->rect().y(), this->rect().width() - 1, this->rect().height() - 1);
  QWidget::paintEvent(event);
}


void NodeManageWidget::NodeManageWidget::Init()
{
  InitMember();
  InitView();
  //ReadFromFile();
  InitSlots();
}

QMap<int, QPair<double,double>> NodeManageWidget::GetInsectionData()
{
  return mapInsection;
}

QMap<int, QPair<double,double>> NodeManageWidget::GetSiteData()
{
  return mapSite;
}

void NodeManageWidget::InsectionReadFromFile(const QString filePath)
{
  QFile fileInsection(filePath);
  bool okFileInsectionRead = fileInsection.open(QIODevice::ReadOnly);

  QStringList fileLineInfo;
  if(okFileInsectionRead)
  {
      while (!fileInsection.atEnd())
      {
          QByteArray line = fileInsection.readLine();
          QString strLine(line);
          if(!strLine.contains(";"))
          {
            QMessageBox::critical(this, tr("出现错误"), tr("导入的路口文件格式错误无法解析！"));
            return;
          }
          if(strLine.contains(";"))
          {
            if(strLine.split(";").size() != 3)
            {
                QMessageBox::critical(this, tr("出现错误"), tr("导入的路口文件格式错误无法解析！0"));
                return;
            }
            fileLineInfo.append(strLine);
        }
    }
    fileInsection.close();
  }

  if(!fileLineInfo.size())
  {
    return;
  }
  for(int i=0;i<ui.tableWidgetIntersection->rowCount();++i)
  {
    ui.tableWidgetIntersection->removeRow(i);
  }
  for(int i=0;i<fileLineInfo.size();++i)
  {
    QString strLine = fileLineInfo[i];
    QString strId = strLine.split(";")[0];
    QString strPosX = strLine.split(";")[1];
    QString strPosY = strLine.split(";")[2].remove("\n");
    bool ok1,ok2;
    double posXFile = strPosX.toDouble(&ok1);
    double posYFile = strPosY.toDouble(&ok2);
    if(ok1 && ok2)
    {
        QPair<double,double> nodePair;
        nodePair.first = posXFile;
        nodePair.second = posYFile;
        mapInsection.insert(strId.toInt(), nodePair);
    }

    ui.tableWidgetIntersection->setRowCount(ui.tableWidgetIntersection->rowCount() + 1);
    QTableWidgetItem* itemId = new QTableWidgetItem();
    itemId->setTextAlignment(Qt::AlignCenter);
    itemId->setText(strId);

    QTableWidgetItem* itemPosX = new QTableWidgetItem();
    itemPosX->setTextAlignment(Qt::AlignCenter);
    itemPosX->setText(strPosX);

    QTableWidgetItem* itemPosY = new QTableWidgetItem();
    itemPosY->setTextAlignment(Qt::AlignCenter);
    itemPosY->setText(strPosY);

    ui.tableWidgetIntersection->setItem(ui.tableWidgetIntersection->rowCount() - 1, 0, itemId);
    ui.tableWidgetIntersection->setItem(ui.tableWidgetIntersection->rowCount() - 1, 1, itemPosX);
    ui.tableWidgetIntersection->setItem(ui.tableWidgetIntersection->rowCount() - 1, 2, itemPosY);
  }
}

void NodeManageWidget::SiteReadFromFile(const QString filePath)
{
  QFile fileSite(filePath);
  QStringList fileLineInfo;
  bool okFileSiteRead = fileSite.open(QIODevice::ReadOnly);
  if(!okFileSiteRead)
  {
    return;
  }
  while (!fileSite.atEnd())
  {
    QByteArray line = fileSite.readLine();
    QString strLine(line);
    if(!strLine.contains(";"))
    {
        QMessageBox::critical(this, tr("出现错误"), tr("导入的站点文件格式错误无法解析！"));
        return;
    }
    if(strLine.split(";").size() != 3)
    {
        QMessageBox::critical(this, tr("出现错误"), tr("导入的站点文件格式错误无法解析！"));
        return;
    }
    fileLineInfo.append(strLine);
  }
  fileSite.close();

  if(!fileLineInfo.size())
  {
    return;
  }

  for(int i=0;i<ui.tableWidgetSite->rowCount();++i)
  {
    ui.tableWidgetSite->removeRow(i);
  }
  for(int i=0;i<fileLineInfo.size();++i)
  {
    QString strLine = fileLineInfo[i];
    QString strId = strLine.split(";")[0];
    QString strPosX = strLine.split(";")[1];
    QString strPosY = strLine.split(";")[2].remove("\n");
    bool ok1,ok2;
    double posXFile = strPosX.toDouble(&ok1);
    double posYFile = strPosY.toDouble(&ok2);
    if(ok1 && ok2)
    {
        QPair<double,double> nodePair;
        nodePair.first = posXFile;
        nodePair.second = posYFile;
        mapSite.insert(strId.toInt(), nodePair);
    }

    ui.tableWidgetSite->setRowCount(ui.tableWidgetSite->rowCount() + 1);
    QTableWidgetItem* itemId = new QTableWidgetItem();
    itemId->setTextAlignment(Qt::AlignCenter);
    itemId->setText(strId);

    QTableWidgetItem* itemPosX = new QTableWidgetItem();
    itemPosX->setTextAlignment(Qt::AlignCenter);
    itemPosX->setText(strPosX);

    QTableWidgetItem* itemPosY = new QTableWidgetItem();
    itemPosY->setTextAlignment(Qt::AlignCenter);
    itemPosY->setText(strPosY);

    ui.tableWidgetSite->setItem(ui.tableWidgetSite->rowCount() - 1, 0, itemId);
    ui.tableWidgetSite->setItem(ui.tableWidgetSite->rowCount() - 1, 1, itemPosX);
    ui.tableWidgetSite->setItem(ui.tableWidgetSite->rowCount() - 1, 2, itemPosY);
  }
}

void NodeManageWidget::NodeManageWidget::InitMember()
{
  QFont font;
  font.setPointSize(font_size);

  startFlag = 0;
  posX = 0;
  posY = 0;

  ui.tabWidget = new QTabWidget();
  ui.tabWidget->setFont(font);

  QStringList tableWidgetLabels;
  tableWidgetLabels <<"ID" << "UTMX" << "UTMY";

  ui.tableWidgetIntersection = new QTableWidget();
  ui.tableWidgetIntersection->setColumnCount(3);
  ui.tableWidgetIntersection->setFont(font);
  ui.tableWidgetIntersection->setHorizontalHeaderLabels(tableWidgetLabels);
  ui.tableWidgetIntersection->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  ui.tableWidgetIntersection->setEditTriggers(QAbstractItemView::NoEditTriggers);

  ui.tableWidgetSite = new QTableWidget();
  ui.tableWidgetSite->setColumnCount(3);
  ui.tableWidgetSite->setFont(font);
  ui.tableWidgetSite->setHorizontalHeaderLabels(tableWidgetLabels);
  ui.tableWidgetSite->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  ui.tableWidgetSite->setEditTriggers(QAbstractItemView::NoEditTriggers);

  ui.tabWidget->addTab(ui.tableWidgetIntersection, "路口管理");
  ui.tabWidget->addTab(ui.tableWidgetSite, "站点管理");

  ui.pushButtonAdd = new QPushButton("添加");
  ui.pushButtonAdd->setFont(font);
  ui.pushButtonEdit = new QPushButton("编辑");
  ui.pushButtonEdit->setFont(font);
  ui.pushButtonDelete = new QPushButton("删除");
  ui.pushButtonDelete->setFont(font);
  ui.pushButtonUp = new QPushButton("上移");
  ui.pushButtonUp->setFont(font);
  ui.pushButtonDown = new QPushButton("下移");
  ui.pushButtonDown->setFont(font);
  ui.pushButtonSave = new QPushButton("保存");
  ui.pushButtonSave->setFont(font);
}

void NodeManageWidget::NodeManageWidget::InitView()
{
  this->setMinimumSize(QSize(400, 300));
  this->setWindowTitle("路口和站点管理");
  this->setWindowIcon(QIcon("://source/1.png"));
  this->setWindowModality(Qt::WindowModal);

  QGridLayout* gridLayout = new QGridLayout(this);

  QVBoxLayout* vboxLayoutPushButton = new QVBoxLayout();

  vboxLayoutPushButton->addWidget(ui.pushButtonAdd);
  vboxLayoutPushButton->addWidget(ui.pushButtonEdit);
  vboxLayoutPushButton->addWidget(ui.pushButtonDelete);
  vboxLayoutPushButton->addWidget(ui.pushButtonUp);
  vboxLayoutPushButton->addWidget(ui.pushButtonDown);
  vboxLayoutPushButton->addWidget(ui.pushButtonSave);
  vboxLayoutPushButton->addStretch();

  gridLayout->addWidget(ui.tabWidget, 0, 0, 1, 1);
  gridLayout->addLayout(vboxLayoutPushButton, 0, 1, 1, 1);

  gridLayout->setColumnStretch(0, 4);
  gridLayout->setColumnStretch(1, 1);
}

void NodeManageWidget::NodeManageWidget::InitSlots()
{
  connect(ui.pushButtonAdd,&QPushButton::clicked,this,&NodeManageWidget::OnActionAdd);
  connect(ui.pushButtonEdit,&QPushButton::clicked,this,&NodeManageWidget::OnActionEdit);
  connect(ui.pushButtonDelete,&QPushButton::clicked,this,&NodeManageWidget::OnActionDelete);
  connect(ui.pushButtonUp,&QPushButton::clicked,this,&NodeManageWidget::OnActionUp);
  connect(ui.pushButtonDown,&QPushButton::clicked,this,&NodeManageWidget::OnActionDown);
  connect(ui.pushButtonSave,&QPushButton::clicked,this,&NodeManageWidget::OnActionSave);
}

void NodeManageWidget::SwapTableWidgetQueue(int currentrow, int tagrgetrow)
{
  QWidget* curWidget = ui.tabWidget->currentWidget();
  QTableWidget* curTableWidget = qobject_cast<QTableWidget*>(curWidget);
  int curId = curTableWidget->item(currentrow,0)->text().toInt();
  int targetId = curTableWidget->item(tagrgetrow,0)->text().toInt();
  if(curTableWidget == ui.tableWidgetIntersection)
  {
    auto temp = mapInsection[targetId];
    mapInsection[targetId] = mapInsection[curId];
    mapInsection[curId] = temp;
  }
  else
  {
    auto temp = mapSite[targetId];
    mapSite[targetId] = mapSite[curId];
    mapSite[curId] = temp;
  }

  QList<QString> listCurRow;
  QList<QString> listTargetRow;
  listCurRow<<curTableWidget->item(currentrow, 0)->text()
             <<curTableWidget->item(currentrow, 1)->text()
             <<curTableWidget->item(currentrow, 2)->text();
  listTargetRow<<curTableWidget->item(tagrgetrow, 0)->text()
             <<curTableWidget->item(tagrgetrow, 1)->text()
             <<curTableWidget->item(tagrgetrow, 2)->text();

  curTableWidget->item(currentrow, 0)->setText(listCurRow[0]);
  curTableWidget->item(currentrow, 1)->setText(listTargetRow[1]);
  curTableWidget->item(currentrow, 2)->setText(listTargetRow[2]);
  curTableWidget->item(tagrgetrow, 0)->setText(listTargetRow[0]);
  curTableWidget->item(tagrgetrow, 1)->setText(listCurRow[1]);
  curTableWidget->item(tagrgetrow, 2)->setText(listCurRow[2]);
}

void NodeManageWidget::ReadFromFile()
{
  mapInsection.clear();
  mapSite.clear();

  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString filePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  filePath = strListCurrentNodeExeDir.join("/");
  filePath = filePath + "/src/hmi/config";
  QString fileInsectionPath = filePath + "/insection.txt";
  QString fileSitePath = filePath + "/site.txt";

  QFile fileInsection(fileInsectionPath);
  bool okFileInsectionRead = fileInsection.open(QIODevice::ReadOnly);
  QFile fileSite(fileSitePath);
  bool okFileSiteRead = fileSite.open(QIODevice::ReadOnly);

  if(okFileInsectionRead)
  {
    while (!fileInsection.atEnd())
    {
        QByteArray line = fileInsection.readLine();
        QString strLine(line);
        if(strLine.contains(";"))
        {
            QString strId = strLine.split(";")[0];
            QString strPosX = strLine.split(";")[1];
            QString strPosY = strLine.split(";")[2].remove("\n");
            bool ok1,ok2;
            double posXFile = strPosX.toDouble(&ok1);
            double posYFile = strPosX.toDouble(&ok2);
            if(ok1 && ok2)
            {
                QPair<double,double> nodePair;
                nodePair.first = posXFile;
                nodePair.second = posYFile;
                mapInsection.insert(strId.toInt(), nodePair);
            }

            ui.tableWidgetIntersection->setRowCount(ui.tableWidgetIntersection->rowCount() + 1);
            QTableWidgetItem* itemId = new QTableWidgetItem();
            itemId->setTextAlignment(Qt::AlignCenter);
            itemId->setText(strId);

            QTableWidgetItem* itemPosX = new QTableWidgetItem();
            itemPosX->setTextAlignment(Qt::AlignCenter);
            itemPosX->setText(strPosX);

            QTableWidgetItem* itemPosY = new QTableWidgetItem();
            itemPosY->setTextAlignment(Qt::AlignCenter);
            itemPosY->setText(strPosY);

            ui.tableWidgetIntersection->setItem(ui.tableWidgetIntersection->rowCount() - 1, 0, itemId);
            ui.tableWidgetIntersection->setItem(ui.tableWidgetIntersection->rowCount() - 1, 1, itemPosX);
            ui.tableWidgetIntersection->setItem(ui.tableWidgetIntersection->rowCount() - 1, 2, itemPosY);
        }
    }
    fileInsection.close();
  }

  if(okFileSiteRead)
  {
    while (!fileSite.atEnd())
    {
        QByteArray line = fileSite.readLine();
        QString strLine(line);
        if(strLine.contains(";"))
        {
            QString strId = strLine.split(";")[0];
            QString strPosX = strLine.split(";")[1];
            QString strPosY = strLine.split(";")[2].remove("\n");
            bool ok1,ok2;
            double posXFile = strPosX.toDouble(&ok1);
            double posYFile = strPosX.toDouble(&ok2);
            if(ok1 && ok2)
            {
                QPair<double,double> nodePair;
                nodePair.first = posXFile;
                nodePair.second = posYFile;
                mapSite.insert(strId.toInt(), nodePair);
            }

            ui.tableWidgetSite->setRowCount(ui.tableWidgetSite->rowCount() + 1);
            QTableWidgetItem* itemId = new QTableWidgetItem();
            itemId->setTextAlignment(Qt::AlignCenter);
            itemId->setText(strId);

            QTableWidgetItem* itemPosX = new QTableWidgetItem();
            itemPosX->setTextAlignment(Qt::AlignCenter);
            itemPosX->setText(strPosX);

            QTableWidgetItem* itemPosY = new QTableWidgetItem();
            itemPosY->setTextAlignment(Qt::AlignCenter);
            itemPosY->setText(strPosY);

            ui.tableWidgetSite->setItem(ui.tableWidgetSite->rowCount() - 1, 0, itemId);
            ui.tableWidgetSite->setItem(ui.tableWidgetSite->rowCount() - 1, 1, itemPosX);
            ui.tableWidgetSite->setItem(ui.tableWidgetSite->rowCount() - 1, 2, itemPosY);
        }
    }

    fileSite.close();
  }
}

void NodeManageWidget::SlotFunctionReceiveGPSImuMsg(const hmi::GpsImuInterface gpsImuMsg)
{
  posX = gpsImuMsg.x;
  posY = gpsImuMsg.y;
}

void NodeManageWidget::OnActionAdd()
{
  QWidget* curWidget = ui.tabWidget->currentWidget();
  QTableWidget* curTableWidget = qobject_cast<QTableWidget*>(curWidget);

  curTableWidget->setRowCount(curTableWidget->rowCount() + 1);

  QTableWidgetItem* tableWidgetItemId = new QTableWidgetItem();
  tableWidgetItemId->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemId->setText(QString::number(curTableWidget->rowCount() + 1));

  QTableWidgetItem* tableWidgetItemX = new QTableWidgetItem();
  tableWidgetItemX->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemX->setText(QString::number(posX, 'f', 11));

  QTableWidgetItem* tableWidgetItemY = new QTableWidgetItem();
  tableWidgetItemY->setTextAlignment(Qt::AlignCenter);
  tableWidgetItemY->setText(QString::number(posY, 'f', 11));

  curTableWidget->setItem(curTableWidget->rowCount() - 1, 0, tableWidgetItemId);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 1, tableWidgetItemX);
  curTableWidget->setItem(curTableWidget->rowCount() - 1, 2, tableWidgetItemY);

  QPair<double,double> pairValue;
  pairValue.first = posX;
  pairValue.second = posY;
  if(curTableWidget == ui.tableWidgetIntersection)
  {
    mapInsection.insert(curTableWidget->rowCount() + 1, pairValue);
  }
  else
  {
    mapSite.insert(curTableWidget->rowCount() + 1, pairValue);
  }
}

void NodeManageWidget::OnActionEdit()
{
  QWidget* curWidget = ui.tabWidget->currentWidget();
  QTableWidget* curTableWidget = qobject_cast<QTableWidget*>(curWidget);
  int curRow = curTableWidget->currentRow();
  int curColumn = curTableWidget->currentColumn();
  bool ok;
  bool okToDouble;
  QString newValue;
  if(curRow == -1)
  {
    return;
  }
  if(curColumn == 0)
  {
    return;
  }

  if(curColumn == 1)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的UTMX:", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入新的UTMX类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
  }

  if(curColumn == 2)
  {
    newValue = QInputDialog::getText(this, "坐标输入", "请输入新的UMTY", QLineEdit::Normal, "", &ok);
    if(!ok)
    {
      return;
    }
    newValue.toDouble(&okToDouble);
    while(!okToDouble)
    {
      newValue = QInputDialog::getText(this, "坐标输入", "请重新输入新的UTMY类型为(double):", QLineEdit::Normal, "", &ok);
      newValue.toDouble(&okToDouble);
    }
  }

  if(newValue.isEmpty())
  {
    return;
  }

  QPair<double,double> pairOldValue;
  int id = curTableWidget->item(curRow, 0)->text().toInt();
  pairOldValue.first = curTableWidget->item(curRow, 1)->text().toDouble();
  pairOldValue.second = curTableWidget->item(curRow, 2)->text().toDouble();
  if(curTableWidget == ui.tableWidgetIntersection)
  {
    if(curColumn == 1)
    {
      mapInsection[id].first =  newValue.toDouble(&okToDouble);
    }
    if(curColumn == 2)
    {
      mapInsection[id].second =  newValue.toDouble(&okToDouble);
    }
  }
  else
  {
    if(curColumn == 1)
    {
      mapSite[id].first =  newValue.toDouble(&okToDouble);
    }
    if(curColumn == 2)
    {
      mapSite[id].second =  newValue.toDouble(&okToDouble);
    }
  }
  curTableWidget->currentItem()->setText(newValue);
}

void NodeManageWidget::OnActionDelete()
{
  QWidget* curWidget = ui.tabWidget->currentWidget();
  QTableWidget* curTableWidget = qobject_cast<QTableWidget*>(curWidget);
  int curRow = curTableWidget->currentRow();
  if(curRow == -1)
  {
    return;
  }
  int id = curTableWidget->item(curRow, 0)->text().toInt();
  if(curTableWidget == ui.tableWidgetIntersection)
  {
    mapInsection.remove(id);
  }
  else
  {
    mapSite.remove(id);
  }
  curTableWidget->removeRow(curRow);
}

void NodeManageWidget::OnActionUp()
{
  QWidget* curWidget = ui.tabWidget->currentWidget();
  QTableWidget* curTableWidget = qobject_cast<QTableWidget*>(curWidget);
  int curRow = curTableWidget->currentRow();


  //如果选中了数据，（未选择数据之前，返回值是-1）
  if (curRow != -1)
  {
    //如果选定行不在第一行
    if (curRow != 0)
    {
      SwapTableWidgetQueue(curRow, curRow - 1);
      //移动过后继续选定该行
      curTableWidget->setCurrentCell(curRow - 1, QItemSelectionModel::Select);
    }
  }
  else
  {
    //如果有数据，但是currentRow=--1 说明没有选择数据,把焦点定位到第一行
    if (curTableWidget->rowCount() != 0)
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：未选中数据"));
    }
    else
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：表格没有数据"));
    }
  }
}

void NodeManageWidget::OnActionDown()
{
  QWidget* curWidget = ui.tabWidget->currentWidget();
  QTableWidget* curTableWidget = qobject_cast<QTableWidget*>(curWidget);
  int curRow = curTableWidget->currentRow();

  if (curRow != -1)//如果选中了一行
  {
    if (curRow != (curTableWidget->rowCount() - 1))//如果不是最后一行
    {
      SwapTableWidgetQueue(curRow, curRow + 1);
      //移动过后继续选定该行
      curTableWidget->setCurrentCell(curRow + 1, QItemSelectionModel::Select);
    }
  }
  else
  {
    if (curTableWidget->rowCount() != 0)//如果有数据，但是currentRow=--1 说明没有选择数据,把焦点定位到第一行
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：未选中数据"));
    }
    else
    {
      QMessageBox::critical(this, tr("出现错误"), tr("无法移动：表格没有数据"));
    }
  }
}

void NodeManageWidget::OnActionSave()
{
  //"/home/jkroly/Jkroly/Code/AIConnect1024/devel/lib/hmi"
  QString currentNodeExeDirPath = QApplication::applicationDirPath();
  QString filePath;
  QStringList strListCurrentNodeExeDir = currentNodeExeDirPath.split("/");
  //node当前文件夹  lib  devel
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  strListCurrentNodeExeDir.removeLast();
  filePath = strListCurrentNodeExeDir.join("/");
  filePath = filePath + "/src/hmi/config";
  if(ui.tabWidget->currentWidget()==ui.tableWidgetIntersection)
  {
    QString fileName = QFileDialog::getSaveFileName(this,tr("保存路口是数据文件"),filePath,"*.txt");
    if(fileName.isEmpty())
    {
      return;
    }

    QFile fileInsection(fileName);
    bool okFileInsectionRead = fileInsection.open(QFile::WriteOnly|QFile::Truncate);
    if(!okFileInsectionRead)
    {
      QMessageBox::critical(this, tr("出现错误"), tr("保存文件创建失败！"));
      return;
    }

    QString strLine;

    QTextStream streamInsection(&fileInsection);
    QMap<int ,QPair<double,double>>::Iterator iter = mapInsection.begin();
    while(iter != mapInsection.end())
    {
      strLine = QString::number(iter.key()) + ";"
                + QString::number(iter.value().first, 'f', 11) + ";"
                + QString::number(iter.value().second, 'f', 11) + "\n";
      streamInsection<<strLine;
      ++iter;
    }
    fileInsection.close();
  }

  if(ui.tabWidget->currentWidget()==ui.tableWidgetSite)
  {
    QString fileName = QFileDialog::getSaveFileName(this,tr("保存路口是数据文件"),filePath,"*.txt");
    if(fileName.isEmpty())
    {
      return;
    }

    QFile fileSite(fileName);
    bool okFileSiteRead = fileSite.open(QFile::WriteOnly|QFile::Truncate);
    if(!okFileSiteRead)
    {
      QMessageBox::critical(this, tr("出现错误"), tr("保存文件创建失败！"));
      return;
    }

    QString strLine;

    QTextStream streamSite(&fileSite);
    QMap<int ,QPair<double,double>>::Iterator iter = mapSite.begin();
    while(iter != mapSite.end())
    {
      strLine = QString::number(iter.key()) + ";"
                + QString::number(iter.value().first, 'f', 11) + ";"
                + QString::number(iter.value().second, 'f', 11) + "\n";
      streamSite<<strLine;
      ++iter;
    }
    fileSite.close();
  }
  QMessageBox::information(this, tr("保存"), tr("保存文件写入成功！"));
}
