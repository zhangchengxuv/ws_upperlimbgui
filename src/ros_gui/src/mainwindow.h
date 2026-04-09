#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTableWidget>
#include <QTimer>
#include <QMutex>

#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/msg/system_state.hpp"
#include "upperlimb_robot/msg/robot_state.hpp"

// ================================
// 控制模式枚举
// 数值必须与控制端一致
// ================================
enum ControlMode : int
{
    // 空闲模式
    // 默认不输出控制命令
    MODE_IDLE = 0,

    // 左臂预设位置模式
    MODE_LEFT_PRESET_POSITION = 1,

    // 右臂预设位置模式
    MODE_RIGHT_PRESET_POSITION = 2,

    // 左臂主动训练模式
    MODE_ACTIVE = 3,

    // 左臂零力模式
    MODE_ZERO_FORCE = 4,

    // 左臂正弦被动模式（PID跟踪）
    MODE_PASSIVE = 5,

    // 左臂正弦被动模式（PD+前馈跟踪）
    MODE_PASSIVE_SINE_PD_FF = 6,
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onIdleClicked();
    void onLeftPresetClicked();
    void onRightPresetClicked();
    void onActiveClicked();
    void onZeroForceClicked();
    void onLeftPassivePID();
    void onLeftPassivePD();

    void refreshUi();

private:
    // ================================
    // ROS 回调
    // ================================
    void systemStateCallback(const upperlimb_robot::msg::SystemState::SharedPtr msg);
    void robotStateCallback(const upperlimb_robot::msg::RobotState::SharedPtr msg);

    void publishCommand(int cmd);
    void appendLog(const QString &text);
    QString modeToString(int mode) const;
    QString boolToText(bool value) const;
    void updateStatusColor(QLabel *label, bool ok, const QString &prefix);
    void updateIndicator(QLabel *indicator, QLabel *textLabel, bool ok, const QString &text);

private:
    // ================================
    // ROS2
    // ================================
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<upperlimb_robot::msg::ControlCommand>::SharedPtr control_command_pub_;
    rclcpp::Subscription<upperlimb_robot::msg::SystemState>::SharedPtr system_state_sub_;
    rclcpp::Subscription<upperlimb_robot::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    std::thread ros_spin_thread_;

    // ================================
    // UI
    // ================================
    QLabel *controllerInitIndicator_ = nullptr;
    QLabel *controllerInitLabel_ = nullptr;

    QLabel *jointInitIndicator_ = nullptr;
    QLabel *jointInitLabel_ = nullptr;

    QLabel *systemReadyIndicator_ = nullptr;
    QLabel *systemReadyLabel_ = nullptr;

    QLabel *currentModeIndicator_ = nullptr;
    QLabel *currentModeLabel_ = nullptr;

    QLabel *timeLabel_ = nullptr;

    QPushButton *idleButton_ = nullptr;
    QPushButton *leftPresetButton_ = nullptr;
    QPushButton *rightPresetButton_ = nullptr;
    QPushButton *activeButton_ = nullptr;
    QPushButton *zeroForceButton_ = nullptr;
    QPushButton *leftPassivePIDButton_ = nullptr;
    QPushButton *leftPassivePDButton_ = nullptr;

    QTableWidget *jointTable_ = nullptr;

    QLabel *leftHandFxLabel_ = nullptr;
    QLabel *leftHandFyLabel_ = nullptr;
    QLabel *leftHandFzLabel_ = nullptr;
    QLabel *leftHandMxLabel_ = nullptr;
    QLabel *leftHandMyLabel_ = nullptr;
    QLabel *leftHandMzLabel_ = nullptr;

    QLabel *leftArmFxLabel_ = nullptr;
    QLabel *leftArmFyLabel_ = nullptr;
    QLabel *leftArmFzLabel_ = nullptr;
    QLabel *leftArmMxLabel_ = nullptr;
    QLabel *leftArmMyLabel_ = nullptr;
    QLabel *leftArmMzLabel_ = nullptr;

    QTextEdit *logTextEdit_ = nullptr;

    QTimer *uiTimer_ = nullptr;

    // ================================
    // 共享缓存（ROS线程写，Qt线程读）
    // ================================
    QMutex dataMutex_;

    bool controller_init_ = false;
    bool joint_init_ = false;
    bool system_ready_ = false;

    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> torque_;
    std::vector<bool> enabled_;
    std::vector<double> force_sensor_;

    int requested_mode_ = MODE_IDLE;
};

#endif // MAINWINDOW_H
