#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    node_ = rclcpp::Node::make_shared("qt_gui_node");

    control_command_pub_ =
        node_->create_publisher<ros_gui::msg::ControlCommand>("control_command", 10);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_LeftPresetPosition_clicked()
{
    ros_gui::msg::ControlCommand msg;
    msg.command = MODE_LEFT_PRESET_POSITION;
    control_command_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published control_command = MODE_LEFT_PRESET_POSITION");
}

void MainWindow::on_RightPresetPosition_clicked()
{
    ros_gui::msg::ControlCommand msg;
    msg.command = MODE_RIGHT_PRESET_POSITION;
    control_command_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published control_command = MODE_RIGHT_PRESET_POSITION");
}

