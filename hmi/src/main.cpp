#include <QApplication>
#include <ros/ros.h>
#include "mainwindow.h"

int main(int argc, char **argv)
{
    if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
        QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    ros::init(argc, argv, "AIConnect");
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    MainWindow mainWindow(argc,argv);
    mainWindow.show();
    return app.exec();
}
