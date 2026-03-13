#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    rclcpp::init(argc,argv);
    MainWindow w;
    w.show();
    return a.exec();
}
