#include "mainwindow.h"

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSplitter>
#include <QGroupBox>
#include <QHeaderView>
#include <QDateTime>
#include <QButtonGroup>
#include <QMutexLocker>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // ================================
    // ROS2 初始化
    // ================================

    node_ = rclcpp::Node::make_shared("qt_gui_node");

    control_command_pub_ =
        node_->create_publisher<ros_gui::msg::ControlCommand>("control_command", 10);

    system_state_sub_ =
        node_->create_subscription<ros_gui::msg::SystemState>(
            "system_state",
            10,
            std::bind(&MainWindow::systemStateCallback, this, std::placeholders::_1));

    robot_state_sub_ =
        node_->create_subscription<ros_gui::msg::RobotState>(
            "robot_state",
            10,
            std::bind(&MainWindow::robotStateCallback, this, std::placeholders::_1));

    // ================================
    // 主窗口
    // ================================
    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(8);

    // ================================
    // 顶部状态栏
    // ================================
    QGroupBox *statusGroup = new QGroupBox("系统状态", central);
    QHBoxLayout *statusLayout = new QHBoxLayout(statusGroup);
    statusLayout->setContentsMargins(12, 8, 12, 8);
    statusLayout->setSpacing(12);

    // 控制器初始化
    controllerInitIndicator_ = new QLabel(statusGroup);
    controllerInitIndicator_->setFixedSize(14, 14);
    controllerInitLabel_ = new QLabel("控制器初始化", statusGroup);

    // 电机初始化
    jointInitIndicator_ = new QLabel(statusGroup);
    jointInitIndicator_->setFixedSize(14, 14);
    jointInitLabel_ = new QLabel("电机初始化", statusGroup);

    // 系统就绪
    systemReadyIndicator_ = new QLabel(statusGroup);
    systemReadyIndicator_->setFixedSize(14, 14);
    systemReadyLabel_ = new QLabel("系统就绪", statusGroup);

    // 当前模式
    currentModeIndicator_ = new QLabel(statusGroup);
    currentModeIndicator_->setFixedSize(14, 14);
    currentModeLabel_ = new QLabel("当前模式: Idle", statusGroup);

    // 时间
    timeLabel_ = new QLabel("--", statusGroup);
    timeLabel_->setStyleSheet("QLabel { color: gray; }");

    // 初始色块样式
    auto initIndicatorStyle = [](QLabel *label)
    {
        label->setStyleSheet(
            "QLabel {"
            "background-color: rgb(140, 140, 140);"
            "border: 1px solid rgb(90, 90, 90);"
            "border-radius: 3px;"
            "}");
    };

    initIndicatorStyle(controllerInitIndicator_);
    initIndicatorStyle(jointInitIndicator_);
    initIndicatorStyle(systemReadyIndicator_);
    initIndicatorStyle(currentModeIndicator_);

    // 控制器初始化
    statusLayout->addWidget(controllerInitIndicator_);
    statusLayout->addWidget(controllerInitLabel_);
    statusLayout->addSpacing(24);

    // 电机初始化
    statusLayout->addWidget(jointInitIndicator_);
    statusLayout->addWidget(jointInitLabel_);
    statusLayout->addSpacing(24);

    // 系统就绪
    statusLayout->addWidget(systemReadyIndicator_);
    statusLayout->addWidget(systemReadyLabel_);
    statusLayout->addSpacing(24);

    // 当前模式
    statusLayout->addWidget(currentModeIndicator_);
    statusLayout->addWidget(currentModeLabel_);

    // 时间推到最右
    statusLayout->addStretch();
    statusLayout->addWidget(timeLabel_);

    // ================================
    // 左右分区
    // ================================
    QSplitter *mainSplitter = new QSplitter(Qt::Horizontal, central);
    mainSplitter->setChildrenCollapsible(false);

    // ================================
    // 左侧控制区
    // ================================
    QGroupBox *controlGroup = new QGroupBox("控制区", mainSplitter);
    QVBoxLayout *controlLayout = new QVBoxLayout(controlGroup);
    controlLayout->setContentsMargins(12, 12, 12, 12);
    controlLayout->setSpacing(10);

    idleButton_ = new QPushButton("未选状态", controlGroup);
    leftPresetButton_ = new QPushButton("左预设位", controlGroup);
    rightPresetButton_ = new QPushButton("右预设位", controlGroup);
    activeButton_ = new QPushButton("左主动模式", controlGroup);
    zeroForceButton_ = new QPushButton("左零力模式", controlGroup);

    idleButton_->setCheckable(true);
    leftPresetButton_->setCheckable(true);
    rightPresetButton_->setCheckable(true);
    activeButton_->setCheckable(true);
    zeroForceButton_->setCheckable(true);
    idleButton_->setChecked(true);

    QButtonGroup *modeGroup = new QButtonGroup(this);
    modeGroup->setExclusive(true);
    modeGroup->addButton(idleButton_);
    modeGroup->addButton(leftPresetButton_);
    modeGroup->addButton(rightPresetButton_);
    modeGroup->addButton(activeButton_);
    modeGroup->addButton(zeroForceButton_);

    connect(idleButton_, &QPushButton::clicked, this, &MainWindow::onIdleClicked);
    connect(leftPresetButton_, &QPushButton::clicked, this, &MainWindow::onLeftPresetClicked);
    connect(rightPresetButton_, &QPushButton::clicked, this, &MainWindow::onRightPresetClicked);
    connect(activeButton_, &QPushButton::clicked, this, &MainWindow::onActiveClicked);
    connect(zeroForceButton_, &QPushButton::clicked, this, &MainWindow::onZeroForceClicked);

    controlLayout->addWidget(idleButton_);
    controlLayout->addWidget(leftPresetButton_);
    controlLayout->addWidget(rightPresetButton_);
    controlLayout->addWidget(activeButton_);
    controlLayout->addWidget(zeroForceButton_);
    controlLayout->addStretch();

    // ================================
    // 右侧上下分区
    // ================================
    QSplitter *rightSplitter = new QSplitter(Qt::Vertical, mainSplitter);
    rightSplitter->setChildrenCollapsible(false);



    // ================================
    // 右上：数据区
    // ================================
    QWidget *dataWidget = new QWidget(rightSplitter);
    QVBoxLayout *dataMainLayout = new QVBoxLayout(dataWidget);
    dataMainLayout->setContentsMargins(0, 0, 0, 0);
    dataMainLayout->setSpacing(8);

    // 关节状态表
    QGroupBox *jointGroup = new QGroupBox("关节状态", dataWidget);
    QVBoxLayout *jointLayout = new QVBoxLayout(jointGroup);

    jointTable_ = new QTableWidget(jointGroup);
    jointTable_->setColumnCount(5);
    jointTable_->setHorizontalHeaderLabels(
        QStringList() << "轴号" << "位置(deg)" << "速度(deg/s)" << "力矩(Nm)" << "使能");
    jointTable_->setRowCount(20);

    // 拉伸，分别设置
    jointTable_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    jointTable_->setColumnWidth(0, 40);

    jointTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    jointTable_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    jointTable_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

    jointTable_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
    jointTable_->setColumnWidth(4, 60);

    // 表头文字居中
    jointTable_->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);

    jointTable_->verticalHeader()->setVisible(false);
    jointTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    jointTable_->setSelectionMode(QAbstractItemView::NoSelection);

    for (int i = 0; i < 20; ++i)
    {
        QTableWidgetItem *axisItem = new QTableWidgetItem(QString::number(i + 1));
        axisItem->setTextAlignment(Qt::AlignCenter);
        jointTable_->setItem(i, 0, axisItem);

        QTableWidgetItem *posItem = new QTableWidgetItem("--");
        posItem->setTextAlignment(Qt::AlignCenter);
        jointTable_->setItem(i, 1, posItem);

        QTableWidgetItem *velItem = new QTableWidgetItem("--");
        velItem->setTextAlignment(Qt::AlignCenter);
        jointTable_->setItem(i, 2, velItem);

        QTableWidgetItem *torqueItem = new QTableWidgetItem("--");
        torqueItem->setTextAlignment(Qt::AlignCenter);
        jointTable_->setItem(i, 3, torqueItem);

        QTableWidgetItem *enableItem = new QTableWidgetItem("--");
        enableItem->setTextAlignment(Qt::AlignCenter);
        jointTable_->setItem(i, 4, enableItem);
    }

    jointLayout->addWidget(jointTable_);

    // 六维力区
    QGroupBox *forceGroup = new QGroupBox("左侧六维力", dataWidget);
    QGridLayout *forceLayout = new QGridLayout(forceGroup);

    QLabel *leftHandTitle = new QLabel("左手传感器", forceGroup);
    QLabel *leftArmTitle = new QLabel("左臂传感器", forceGroup);

    leftHandFxLabel_ = new QLabel("Fx: --", forceGroup);
    leftHandFyLabel_ = new QLabel("Fy: --", forceGroup);
    leftHandFzLabel_ = new QLabel("Fz: --", forceGroup);
    leftHandMxLabel_ = new QLabel("Mx: --", forceGroup);
    leftHandMyLabel_ = new QLabel("My: --", forceGroup);
    leftHandMzLabel_ = new QLabel("Mz: --", forceGroup);

    leftArmFxLabel_ = new QLabel("Fx: --", forceGroup);
    leftArmFyLabel_ = new QLabel("Fy: --", forceGroup);
    leftArmFzLabel_ = new QLabel("Fz: --", forceGroup);
    leftArmMxLabel_ = new QLabel("Mx: --", forceGroup);
    leftArmMyLabel_ = new QLabel("My: --", forceGroup);
    leftArmMzLabel_ = new QLabel("Mz: --", forceGroup);

    forceLayout->addWidget(leftHandTitle, 0, 0, 1, 2);
    forceLayout->addWidget(leftHandFxLabel_, 1, 0);
    forceLayout->addWidget(leftHandFyLabel_, 1, 1);
    forceLayout->addWidget(leftHandFzLabel_, 2, 0);
    forceLayout->addWidget(leftHandMxLabel_, 2, 1);
    forceLayout->addWidget(leftHandMyLabel_, 3, 0);
    forceLayout->addWidget(leftHandMzLabel_, 3, 1);

    forceLayout->addWidget(leftArmTitle, 0, 2, 1, 2);
    forceLayout->addWidget(leftArmFxLabel_, 1, 2);
    forceLayout->addWidget(leftArmFyLabel_, 1, 3);
    forceLayout->addWidget(leftArmFzLabel_, 2, 2);
    forceLayout->addWidget(leftArmMxLabel_, 2, 3);
    forceLayout->addWidget(leftArmMyLabel_, 3, 2);
    forceLayout->addWidget(leftArmMzLabel_, 3, 3);

    dataMainLayout->addWidget(jointGroup, 3);
    dataMainLayout->addWidget(forceGroup, 1);

    // ================================
    // 右下：日志区
    // ================================
    QGroupBox *logGroup = new QGroupBox("日志", rightSplitter);
    QVBoxLayout *logLayout = new QVBoxLayout(logGroup);

    logTextEdit_ = new QTextEdit(logGroup);
    logTextEdit_->setReadOnly(true);
    logLayout->addWidget(logTextEdit_);

    // splitter 比例
    mainSplitter->addWidget(controlGroup);
    mainSplitter->addWidget(rightSplitter);
    mainSplitter->setStretchFactor(0, 3);
    mainSplitter->setStretchFactor(1, 7);

    rightSplitter->addWidget(dataWidget);
    rightSplitter->addWidget(logGroup);
    rightSplitter->setStretchFactor(0, 7);
    rightSplitter->setStretchFactor(1, 3);

    mainLayout->addWidget(statusGroup);
    mainLayout->addWidget(mainSplitter, 1);

    setWindowTitle("UpperLimb GUI");
    resize(1400, 900);

    // ================================
    // UI 定时刷新
    // ================================
    uiTimer_ = new QTimer(this);
    connect(uiTimer_, &QTimer::timeout, this, &MainWindow::refreshUi);
    uiTimer_->start(100); // 10 Hz 刷新界面，先稳一些

    // ================================
    // 启动 ROS spin 线程
    // ================================
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    ros_spin_thread_ = std::thread([this]() {
        executor_->spin();
    });

    appendLog("GUI 启动完成");
}

MainWindow::~MainWindow()
{
    if (executor_)
    {
        executor_->cancel();
        executor_->remove_node(node_);
    }

    if (ros_spin_thread_.joinable())
    {
        ros_spin_thread_.join();
    }
}

void MainWindow::publishCommand(int cmd)
{
    ros_gui::msg::ControlCommand msg;
    msg.command = cmd;
    control_command_pub_->publish(msg);

    {
        QMutexLocker locker(&dataMutex_);
        requested_mode_ = cmd;
    }

    appendLog(QString("发送控制命令: %1").arg(modeToString(cmd)));
}

void MainWindow::onIdleClicked()
{
    publishCommand(MODE_IDLE);
}

void MainWindow::onLeftPresetClicked()
{
    publishCommand(MODE_LEFT_PRESET_POSITION);
}

void MainWindow::onRightPresetClicked()
{
    publishCommand(MODE_RIGHT_PRESET_POSITION);
}

void MainWindow::onActiveClicked()
{
    publishCommand(MODE_ACTIVE);
}

void MainWindow::onZeroForceClicked()
{
    publishCommand(MODE_ZERO_FORCE);
}

void MainWindow::systemStateCallback(const ros_gui::msg::SystemState::SharedPtr msg)
{
    QMutexLocker locker(&dataMutex_);
    controller_init_ = msg->controller_init;
    joint_init_ = msg->joint_init;
    system_ready_ = msg->system_ready;
}

void MainWindow::robotStateCallback(const ros_gui::msg::RobotState::SharedPtr msg)
{
    QMutexLocker locker(&dataMutex_);
    position_ = msg->position;
    velocity_ = msg->velocity;
    torque_ = msg->torque;
    enabled_ = msg->enabled;
    force_sensor_ = msg->force_sensor;
}

void MainWindow::refreshUi()
{
    QMutexLocker locker(&dataMutex_);

    updateIndicator(controllerInitIndicator_, controllerInitLabel_, controller_init_, "控制器初始化");
    updateIndicator(jointInitIndicator_, jointInitLabel_, joint_init_, "电机初始化");
    updateIndicator(systemReadyIndicator_, systemReadyLabel_, system_ready_, "系统就绪");

    // 当前模式文字
    currentModeLabel_->setText(QString("当前模式: %1").arg(modeToString(requested_mode_)));

    // 当前模式色块：这里只做简单示意
    currentModeIndicator_->setStyleSheet(
        "QLabel {"
        "background-color: rgb(0, 120, 215);"
        "border: 1px solid rgb(0, 70, 150);"
        "border-radius: 3px;"
        "}");

    // 右侧时间
    timeLabel_->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));

    QString now = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    timeLabel_->setText(now);

    // 更新关节表
    const int jointCount = std::min<int>(20, position_.size());

    for (int i = 0; i < 20; ++i)
    {
        QString posText = "--";
        QString velText = "--";
        QString torText = "--";
        QString enText = "--";

        if (i < static_cast<int>(position_.size()))
        {
            posText = QString::number(position_[i], 'f', 2);
        }

        if (i < static_cast<int>(velocity_.size()))
        {
            velText = QString::number(velocity_[i], 'f', 2);
        }

        if (i < static_cast<int>(torque_.size()))
        {
            torText = QString::number(torque_[i], 'f', 3);
        }

        if (i < static_cast<int>(enabled_.size()))
        {
            enText = enabled_[i] ? "ON" : "OFF";
        }

        jointTable_->item(i, 1)->setText(posText);
        jointTable_->item(i, 2)->setText(velText);
        jointTable_->item(i, 3)->setText(torText);
        jointTable_->item(i, 4)->setText(enText);
    }

    // force_sensor 顺序：
    // 0~5 右手
    // 6~11 右臂
    // 12~17 左手
    // 18~23 左臂
    auto getForce = [this](int idx) -> QString {
        if (idx >= 0 && idx < static_cast<int>(force_sensor_.size()))
        {
            return QString::number(force_sensor_[idx], 'f', 3);
        }
        return "--";
    };

    leftHandFxLabel_->setText("Fx: " + getForce(12));
    leftHandFyLabel_->setText("Fy: " + getForce(13));
    leftHandFzLabel_->setText("Fz: " + getForce(14));
    leftHandMxLabel_->setText("Mx: " + getForce(15));
    leftHandMyLabel_->setText("My: " + getForce(16));
    leftHandMzLabel_->setText("Mz: " + getForce(17));

    leftArmFxLabel_->setText("Fx: " + getForce(18));
    leftArmFyLabel_->setText("Fy: " + getForce(19));
    leftArmFzLabel_->setText("Fz: " + getForce(20));
    leftArmMxLabel_->setText("Mx: " + getForce(21));
    leftArmMyLabel_->setText("My: " + getForce(22));
    leftArmMzLabel_->setText("Mz: " + getForce(23));

    Q_UNUSED(jointCount);
}

QString MainWindow::modeToString(int mode) const
{
    switch (mode)
    {
    case MODE_IDLE:
        return "未选";
    case MODE_LEFT_PRESET_POSITION:
        return "左预设位";
    case MODE_RIGHT_PRESET_POSITION:
        return "右预设位";
    case MODE_ACTIVE:
        return "左主动模式";
    case MODE_ZERO_FORCE:
        return "左零力模式";
    default:
        return QString("未知(%1)").arg(mode);
    }
}

QString MainWindow::boolToText(bool value) const
{
    return value ? "是" : "否";
}

void MainWindow::updateStatusColor(QLabel *label, bool ok, const QString &prefix)
{
    label->setText(QString("%1: %2").arg(prefix, ok ? "正常" : "未就绪"));
    if (ok)
    {
        label->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    }
    else
    {
        label->setStyleSheet("QLabel { color: red; font-weight: bold; }");
    }
}

void MainWindow::appendLog(const QString &text)
{
    const QString now = QDateTime::currentDateTime().toString("HH:mm:ss");
    logTextEdit_->append(QString("[%1] %2").arg(now, text));
}

void MainWindow::updateIndicator(QLabel *indicator, QLabel *textLabel, bool ok, const QString &text)
{
    textLabel->setText(text);

    if (ok)
    {
        indicator->setStyleSheet(
            "QLabel {"
            "background-color: rgb(0, 190, 0);"
            "border: 1px solid rgb(0, 120, 0);"
            "border-radius: 3px;"
            "}");
    }
    else
    {
        indicator->setStyleSheet(
            "QLabel {"
            "background-color: rgb(220, 0, 0);"
            "border: 1px solid rgb(140, 0, 0);"
            "border-radius: 3px;"
            "}");
    }
}
