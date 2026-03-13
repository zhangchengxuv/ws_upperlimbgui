#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include "ros_gui/msg/control_command.hpp"

namespace Ui {
class MainWindow;
}
// 控制模式枚举
enum ControlMode : int
{
    // 左右侧回归预设位置
    MODE_LEFT_PRESET_POSITION = 1,
    MODE_RIGHT_PRESET_POSITION = 2,

};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_LeftPresetPosition_clicked();
    void on_RightPresetPosition_clicked();

private:
    Ui::MainWindow *ui;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<ros_gui::msg::ControlCommand>::SharedPtr control_command_pub_;
};

#endif // MAINWINDOW_H
