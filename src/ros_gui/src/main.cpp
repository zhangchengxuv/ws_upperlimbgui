#include "mainwindow.h"
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <QFile>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);

    QFile file(":/resource/style.qss");
    if (file.open(QFile::ReadOnly))
    {
        a.setStyleSheet(file.readAll());
        file.close();
    }

    MainWindow w;
    w.show();

    int ret = a.exec();

    rclcpp::shutdown();
    return ret;
}
