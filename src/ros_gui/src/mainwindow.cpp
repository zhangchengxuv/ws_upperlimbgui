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

#include <algorithm>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // ================================
    // ROS2 初始化
    // ================================

    node_ = rclcpp::Node::make_shared("qt_gui_node");

    control_command_pub_ =
        node_->create_publisher<upperlimb_robot::msg::ControlCommand>("control_command", 10);

    system_state_sub_ =
        node_->create_subscription<upperlimb_robot::msg::SystemState>(
            "system_state",
            10,
            std::bind(&MainWindow::systemStateCallback, this, std::placeholders::_1));

    robot_state_sub_ =
        node_->create_subscription<upperlimb_robot::msg::RobotState>(
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
    rightActiveButton_ = new QPushButton("右主动模式", controlGroup);
    zeroForceButton_ = new QPushButton("左零力模式", controlGroup);
    rightZeroForceButton_ = new QPushButton("右零力模式", controlGroup);
    bilateralZeroForceButton_ = new QPushButton("双臂零力模式", controlGroup);
    leftPassivePIDButton_ = new QPushButton("左正弦被动（PID）", controlGroup);
    leftPassivePDButton_ = new QPushButton("左正弦被动（PD）", controlGroup);
    leftPassiveTPDButton_ = new QPushButton("左正弦被动（扭矩模式PD）", controlGroup);
    mirrorButton_ = new QPushButton("双臂镜像模式（零力）", controlGroup);
    mirrorActiveButton_ = new QPushButton("双臂镜像模式（主动）", controlGroup);
    activeSpringButton_ = new QPushButton("双手 X 向虚拟弹簧主动模式", controlGroup);
    activeSpring3DButton_ = new QPushButton("双手空间虚拟弹簧主动模式", controlGroup);
    bilateralActiveButton_ = new QPushButton("双侧主动模式", controlGroup);
    
    idleButton_->setCheckable(true);
    leftPresetButton_->setCheckable(true);
    rightPresetButton_->setCheckable(true);
    activeButton_->setCheckable(true);
    rightActiveButton_->setCheckable(true);
    zeroForceButton_->setCheckable(true);
    rightZeroForceButton_->setCheckable(true);
    bilateralZeroForceButton_->setCheckable(true);
    leftPassivePIDButton_->setCheckable(true);
    leftPassivePDButton_->setCheckable(true);
    leftPassiveTPDButton_->setCheckable(true);
    mirrorButton_->setCheckable(true);
    mirrorActiveButton_->setCheckable(true);
    activeSpringButton_->setCheckable(true);
    activeSpring3DButton_->setCheckable(true);
    bilateralActiveButton_->setCheckable(true);

    idleButton_->setChecked(true);

    QButtonGroup *modeGroup = new QButtonGroup(this);
    modeGroup->setExclusive(true);
    modeGroup->addButton(idleButton_);
    modeGroup->addButton(leftPresetButton_);
    modeGroup->addButton(rightPresetButton_);
    modeGroup->addButton(activeButton_);
    modeGroup->addButton(rightActiveButton_);
    modeGroup->addButton(zeroForceButton_);
    modeGroup->addButton(rightZeroForceButton_);
    modeGroup->addButton(bilateralZeroForceButton_);
    modeGroup->addButton(leftPassivePIDButton_);
    modeGroup->addButton(leftPassivePDButton_);
    modeGroup->addButton(leftPassiveTPDButton_);
    modeGroup->addButton(mirrorButton_);
    modeGroup->addButton(mirrorActiveButton_);
    modeGroup->addButton(activeSpringButton_);
    modeGroup->addButton(activeSpring3DButton_);
    modeGroup->addButton(bilateralActiveButton_);

    connect(idleButton_, &QPushButton::clicked, this, &MainWindow::onIdleClicked);
    connect(leftPresetButton_, &QPushButton::clicked, this, &MainWindow::onLeftPresetClicked);
    connect(rightPresetButton_, &QPushButton::clicked, this, &MainWindow::onRightPresetClicked);
    connect(activeButton_, &QPushButton::clicked, this, &MainWindow::onActiveClicked);
    connect(rightActiveButton_, &QPushButton::clicked, this, &MainWindow::onRightActiveClicked);
    connect(zeroForceButton_, &QPushButton::clicked, this, &MainWindow::onZeroForceClicked);
    connect(leftPassivePIDButton_, &QPushButton::clicked, this, &MainWindow::onLeftPassivePID);
    connect(leftPassivePDButton_, &QPushButton::clicked, this, &MainWindow::onLeftPassivePD);
    connect(leftPassiveTPDButton_, &QPushButton::clicked, this, &MainWindow::onLeftPassiveTPD);
    connect(rightZeroForceButton_, &QPushButton::clicked, this, &MainWindow::onRightZeroForceClicked);
    connect(bilateralZeroForceButton_, &QPushButton::clicked, this, &MainWindow::onBilateralZeroForceClicked);
    connect(mirrorButton_, &QPushButton::clicked, this, &MainWindow::onMirrorClicked);
    connect(mirrorActiveButton_, &QPushButton::clicked, this, &MainWindow::onMirrorActiveClicked);
    connect(activeSpringButton_, &QPushButton::clicked, this, &MainWindow::onActiveSpringClicked);
    connect(activeSpring3DButton_, &QPushButton::clicked, this, &MainWindow::onActiveSpring3DClicked);
    connect(bilateralActiveButton_, &QPushButton::clicked, this, &MainWindow::onBilateralActiveClicked);
    // --- IGNORE ---
    controlLayout->addWidget(idleButton_);

    controlLayout->addWidget(leftPresetButton_);
    controlLayout->addWidget(rightPresetButton_);

    controlLayout->addWidget(activeButton_);
    controlLayout->addWidget(rightActiveButton_);

    controlLayout->addWidget(zeroForceButton_);
    controlLayout->addWidget(rightZeroForceButton_);
    controlLayout->addWidget(bilateralZeroForceButton_);

    controlLayout->addWidget(bilateralActiveButton_);

    controlLayout->addWidget(activeSpringButton_);
    controlLayout->addWidget(activeSpring3DButton_);

    controlLayout->addWidget(mirrorButton_);
    controlLayout->addWidget(mirrorActiveButton_);

    controlLayout->addWidget(leftPassivePIDButton_);
    controlLayout->addWidget(leftPassivePDButton_);
    controlLayout->addWidget(leftPassiveTPDButton_);

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

    // 关节状态表：左右臂分开显示，避免 20 行单表需要下翻
    QGroupBox *jointGroup = new QGroupBox("关节状态", dataWidget);
    QHBoxLayout *jointLayout = new QHBoxLayout(jointGroup);
    jointLayout->setContentsMargins(8, 8, 8, 8);
    jointLayout->setSpacing(8);

    auto createJointTable = [](QWidget *parent) -> QTableWidget *
    {
        QTableWidget *table = new QTableWidget(parent);
        table->setColumnCount(5);
        table->setHorizontalHeaderLabels(
            QStringList() << "轴号" << "位置(deg)" << "速度(deg/s)" << "力矩(Nm)" << "使能");
        table->setRowCount(10);

        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
        table->setColumnWidth(0, 45);

        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);

        table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Fixed);
        table->setColumnWidth(4, 60);

        table->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
        table->verticalHeader()->setVisible(false);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setSelectionMode(QAbstractItemView::NoSelection);

        for (int i = 0; i < 10; ++i)
        {
            QTableWidgetItem *axisItem = new QTableWidgetItem(QString::number(i + 1));
            axisItem->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, 0, axisItem);

            QTableWidgetItem *posItem = new QTableWidgetItem("--");
            posItem->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, 1, posItem);

            QTableWidgetItem *velItem = new QTableWidgetItem("--");
            velItem->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, 2, velItem);

            QTableWidgetItem *torqueItem = new QTableWidgetItem("--");
            torqueItem->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, 3, torqueItem);

            QTableWidgetItem *enableItem = new QTableWidgetItem("--");
            enableItem->setTextAlignment(Qt::AlignCenter);
            table->setItem(i, 4, enableItem);
        }

        return table;
    };

    QGroupBox *leftJointGroup = new QGroupBox("左臂关节", jointGroup);
    QVBoxLayout *leftJointLayout = new QVBoxLayout(leftJointGroup);
    leftJointLayout->setContentsMargins(6, 6, 6, 6);

    QGroupBox *rightJointGroup = new QGroupBox("右臂关节", jointGroup);
    QVBoxLayout *rightJointLayout = new QVBoxLayout(rightJointGroup);
    rightJointLayout->setContentsMargins(6, 6, 6, 6);

    leftJointTable_ = createJointTable(leftJointGroup);
    rightJointTable_ = createJointTable(rightJointGroup);

    for (int i = 0; i < 10; ++i)
    {
        leftJointTable_->item(i, 0)->setText(QString("L%1").arg(i + 1));
        rightJointTable_->item(i, 0)->setText(QString("R%1").arg(i + 1));
    }

    leftJointLayout->addWidget(leftJointTable_);
    rightJointLayout->addWidget(rightJointTable_);

    jointLayout->addWidget(leftJointGroup);
    jointLayout->addWidget(rightJointGroup);

    // 六维力区
    QGroupBox *forceGroup = new QGroupBox("六维力信息", dataWidget);
    QGridLayout *forceLayout = new QGridLayout(forceGroup);
    forceLayout->setContentsMargins(12, 8, 12, 8);
    forceLayout->setHorizontalSpacing(20);
    forceLayout->setVerticalSpacing(6);

    QLabel *rightHandTitle = new QLabel("右手传感器", forceGroup);
    QLabel *rightArmTitle = new QLabel("右臂传感器", forceGroup);
    QLabel *leftHandTitle = new QLabel("左手传感器", forceGroup);
    QLabel *leftArmTitle = new QLabel("左臂传感器", forceGroup);

    rightHandFxLabel_ = new QLabel("Fx: --", forceGroup);
    rightHandFyLabel_ = new QLabel("Fy: --", forceGroup);
    rightHandFzLabel_ = new QLabel("Fz: --", forceGroup);
    rightHandMxLabel_ = new QLabel("Mx: --", forceGroup);
    rightHandMyLabel_ = new QLabel("My: --", forceGroup);
    rightHandMzLabel_ = new QLabel("Mz: --", forceGroup);

    rightArmFxLabel_ = new QLabel("Fx: --", forceGroup);
    rightArmFyLabel_ = new QLabel("Fy: --", forceGroup);
    rightArmFzLabel_ = new QLabel("Fz: --", forceGroup);
    rightArmMxLabel_ = new QLabel("Mx: --", forceGroup);
    rightArmMyLabel_ = new QLabel("My: --", forceGroup);
    rightArmMzLabel_ = new QLabel("Mz: --", forceGroup);

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

    // 布局：上排右手/右臂，下排左手/左臂
    forceLayout->addWidget(rightHandTitle, 0, 0, 1, 2);
    forceLayout->addWidget(rightHandFxLabel_, 1, 0);
    forceLayout->addWidget(rightHandFyLabel_, 1, 1);
    forceLayout->addWidget(rightHandFzLabel_, 2, 0);
    forceLayout->addWidget(rightHandMxLabel_, 2, 1);
    forceLayout->addWidget(rightHandMyLabel_, 3, 0);
    forceLayout->addWidget(rightHandMzLabel_, 3, 1);

    forceLayout->addWidget(rightArmTitle, 0, 2, 1, 2);
    forceLayout->addWidget(rightArmFxLabel_, 1, 2);
    forceLayout->addWidget(rightArmFyLabel_, 1, 3);
    forceLayout->addWidget(rightArmFzLabel_, 2, 2);
    forceLayout->addWidget(rightArmMxLabel_, 2, 3);
    forceLayout->addWidget(rightArmMyLabel_, 3, 2);
    forceLayout->addWidget(rightArmMzLabel_, 3, 3);

    forceLayout->addWidget(leftHandTitle, 4, 0, 1, 2);
    forceLayout->addWidget(leftHandFxLabel_, 5, 0);
    forceLayout->addWidget(leftHandFyLabel_, 5, 1);
    forceLayout->addWidget(leftHandFzLabel_, 6, 0);
    forceLayout->addWidget(leftHandMxLabel_, 6, 1);
    forceLayout->addWidget(leftHandMyLabel_, 7, 0);
    forceLayout->addWidget(leftHandMzLabel_, 7, 1);

    forceLayout->addWidget(leftArmTitle, 4, 2, 1, 2);
    forceLayout->addWidget(leftArmFxLabel_, 5, 2);
    forceLayout->addWidget(leftArmFyLabel_, 5, 3);
    forceLayout->addWidget(leftArmFzLabel_, 6, 2);
    forceLayout->addWidget(leftArmMxLabel_, 6, 3);
    forceLayout->addWidget(leftArmMyLabel_, 7, 2);
    forceLayout->addWidget(leftArmMzLabel_, 7, 3);

    // 虚拟弹簧状态区
    QGroupBox *springGroup = new QGroupBox("虚拟弹簧状态", dataWidget);
    QGridLayout *springLayout = new QGridLayout(springGroup);
    springLayout->setContentsMargins(12, 8, 12, 8);
    springLayout->setHorizontalSpacing(20);
    springLayout->setVerticalSpacing(6);

    springModeLabel_ = new QLabel("模式: --", springGroup);
    springValidLabel_ = new QLabel("状态: --", springGroup);

    springExLabel_ = new QLabel("ex: -- m", springGroup);
    springEyLabel_ = new QLabel("ey: -- m", springGroup);
    springEzLabel_ = new QLabel("ez: -- m", springGroup);
    springENormLabel_ = new QLabel("|e|: -- m", springGroup);

    springFxLabel_ = new QLabel("Fx: -- N", springGroup);
    springFyLabel_ = new QLabel("Fy: -- N", springGroup);
    springFzLabel_ = new QLabel("Fz: -- N", springGroup);
    springFNormLabel_ = new QLabel("|F|: -- N", springGroup);

    springLayout->addWidget(springModeLabel_, 0, 0);
    springLayout->addWidget(springValidLabel_, 0, 1);

    springLayout->addWidget(springExLabel_, 1, 0);
    springLayout->addWidget(springEyLabel_, 1, 1);
    springLayout->addWidget(springEzLabel_, 1, 2);
    springLayout->addWidget(springENormLabel_, 1, 3);

    springLayout->addWidget(springFxLabel_, 2, 0);
    springLayout->addWidget(springFyLabel_, 2, 1);
    springLayout->addWidget(springFzLabel_, 2, 2);
    springLayout->addWidget(springFNormLabel_, 2, 3);

    dataMainLayout->addWidget(jointGroup, 3);
    dataMainLayout->addWidget(forceGroup, 1);
    dataMainLayout->addWidget(springGroup, 1);

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

    ros_spin_thread_ = std::thread([this]()
                                   { executor_->spin(); });

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
    upperlimb_robot::msg::ControlCommand msg;
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

void MainWindow::onRightActiveClicked()
{
    publishCommand(MODE_RIGHT_ACTIVE);
}

void MainWindow::onZeroForceClicked()
{
    publishCommand(MODE_ZERO_FORCE);
}

void MainWindow::onLeftPassivePID()
{
    publishCommand(MODE_PASSIVE);
}

void MainWindow::onLeftPassivePD()
{
    publishCommand(MODE_PASSIVE_SINE_PD_FF);
}

void MainWindow::onLeftPassiveTPD()
{
    publishCommand(MODE_PASSIVE_TORQUE);
}

void MainWindow::onRightZeroForceClicked()
{
    publishCommand(MODE_RIGHT_ZERO_FORCE);
}

void MainWindow::onBilateralZeroForceClicked()
{
    publishCommand(MODE_BILATERAL_ZERO_FORCE);
}

void MainWindow::onMirrorClicked()
{
    publishCommand(MODE_MIRROR);
}

void MainWindow::onMirrorActiveClicked()
{
    publishCommand(MODE_MIRROR_ACTIVE);
}

void MainWindow::onActiveSpringClicked()
{
    publishCommand(MODE_ACTIVE_SPRING);
}

void MainWindow::onActiveSpring3DClicked()
{
    publishCommand(MODE_ACTIVE_SPRING_3D);
}

void MainWindow::onBilateralActiveClicked()
{
    publishCommand(MODE_BILATERAL_ACTIVE);
}

void MainWindow::systemStateCallback(const upperlimb_robot::msg::SystemState::SharedPtr msg)
{
    QMutexLocker locker(&dataMutex_);
    controller_init_ = msg->controller_init;
    joint_init_ = msg->joint_init;
    system_ready_ = msg->system_ready;
}

void MainWindow::robotStateCallback(const upperlimb_robot::msg::RobotState::SharedPtr msg)
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
    // 默认映射：
    // 0~9   左臂
    // 10~19 右臂
    // 如果你的 RobotState 顺序相反，只需要交换下面两个 offset。
    auto updateJointTable = [this](QTableWidget *table, int offset)
    {
        for (int row = 0; row < 10; ++row)
        {
            const int idx = offset + row;

            QString posText = "--";
            QString velText = "--";
            QString torText = "--";
            QString enText = "--";

            if (idx >= 0 && idx < static_cast<int>(position_.size()))
            {
                posText = QString::number(position_[idx], 'f', 2);
            }

            if (idx >= 0 && idx < static_cast<int>(velocity_.size()))
            {
                velText = QString::number(velocity_[idx], 'f', 2);
            }

            if (idx >= 0 && idx < static_cast<int>(torque_.size()))
            {
                torText = QString::number(torque_[idx], 'f', 3);
            }

            if (idx >= 0 && idx < static_cast<int>(enabled_.size()))
            {
                enText = enabled_[idx] ? "ON" : "OFF";
            }

            table->item(row, 1)->setText(posText);
            table->item(row, 2)->setText(velText);
            table->item(row, 3)->setText(torText);
            table->item(row, 4)->setText(enText);
        }
    };

    updateJointTable(leftJointTable_, 10);
    updateJointTable(rightJointTable_, 0);

    // force_sensor 顺序：
    // 0~5   右手
    // 6~11  右臂
    // 12~17 左手
    // 18~23 左臂
    auto getForce = [this](int idx) -> QString
    {
        if (idx >= 0 && idx < static_cast<int>(force_sensor_.size()))
        {
            return QString::number(force_sensor_[idx], 'f', 3);
        }
        return "--";
    };

    // 右手：0~5
    rightHandFxLabel_->setText("Fx: " + getForce(0));
    rightHandFyLabel_->setText("Fy: " + getForce(1));
    rightHandFzLabel_->setText("Fz: " + getForce(2));
    rightHandMxLabel_->setText("Mx: " + getForce(3));
    rightHandMyLabel_->setText("My: " + getForce(4));
    rightHandMzLabel_->setText("Mz: " + getForce(5));

    // 右臂：6~11
    rightArmFxLabel_->setText("Fx: " + getForce(6));
    rightArmFyLabel_->setText("Fy: " + getForce(7));
    rightArmFzLabel_->setText("Fz: " + getForce(8));
    rightArmMxLabel_->setText("Mx: " + getForce(9));
    rightArmMyLabel_->setText("My: " + getForce(10));
    rightArmMzLabel_->setText("Mz: " + getForce(11));

    // 左手：12~17
    leftHandFxLabel_->setText("Fx: " + getForce(12));
    leftHandFyLabel_->setText("Fy: " + getForce(13));
    leftHandFzLabel_->setText("Fz: " + getForce(14));
    leftHandMxLabel_->setText("Mx: " + getForce(15));
    leftHandMyLabel_->setText("My: " + getForce(16));
    leftHandMzLabel_->setText("Mz: " + getForce(17));

    // 左臂：18~23
    leftArmFxLabel_->setText("Fx: " + getForce(18));
    leftArmFyLabel_->setText("Fy: " + getForce(19));
    leftArmFzLabel_->setText("Fz: " + getForce(20));
    leftArmMxLabel_->setText("Mx: " + getForce(21));
    leftArmMyLabel_->setText("My: " + getForce(22));
    leftArmMzLabel_->setText("Mz: " + getForce(23));

    auto getExtra = [this](int idx) -> double
    {
        if (idx >= 0 && idx < static_cast<int>(force_sensor_.size()))
        {
            return force_sensor_[idx];
        }
        return 0.0;
    };

    // force_sensor[0~23] 仍然是四个六维力传感器。
    // force_sensor[24~33] 是控制节点追加的弹簧调试量：
    // [mode, valid, ex, ey, ez, |e|, Fx, Fy, Fz, |F|]
    const int springMode = static_cast<int>(std::lround(getExtra(24)));
    const bool springValid = (getExtra(25) > 0.5);

    const double ex = getExtra(26);
    const double ey = getExtra(27);
    const double ez = getExtra(28);
    const double enorm = getExtra(29);

    const double fx = getExtra(30);
    const double fy = getExtra(31);
    const double fz = getExtra(32);
    const double fnorm = getExtra(33);

    QString springModeText = "--";
    if (springMode == 1)
    {
        springModeText = "X 向一维虚拟弹簧";
    }
    else if (springMode == 2)
    {
        springModeText = "空间三维虚拟弹簧";
    }

    springModeLabel_->setText("模式: " + springModeText);
    springValidLabel_->setText(QString("状态: %1").arg(springValid ? "有效" : "无效/未启用"));

    springExLabel_->setText(QString("ex: %1 m").arg(ex, 0, 'f', 4));
    springEyLabel_->setText(QString("ey: %1 m").arg(ey, 0, 'f', 4));
    springEzLabel_->setText(QString("ez: %1 m").arg(ez, 0, 'f', 4));
    springENormLabel_->setText(QString("|e|: %1 m").arg(enorm, 0, 'f', 4));

    springFxLabel_->setText(QString("Fx: %1 N").arg(fx, 0, 'f', 3));
    springFyLabel_->setText(QString("Fy: %1 N").arg(fy, 0, 'f', 3));
    springFzLabel_->setText(QString("Fz: %1 N").arg(fz, 0, 'f', 3));
    springFNormLabel_->setText(QString("|F|: %1 N").arg(fnorm, 0, 'f', 3));

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
    case MODE_RIGHT_ACTIVE:
        return "右主动模式";
    case MODE_ZERO_FORCE:
        return "左零力模式";
    case MODE_PASSIVE:
        return "左臂正弦被动模式（PID跟踪）";
    case MODE_PASSIVE_SINE_PD_FF:
        return "左臂正弦被动模式（PD+前馈跟踪）";
    case MODE_PASSIVE_TORQUE:
        return "左臂正弦被动模式（扭矩模式）";
    case MODE_RIGHT_ZERO_FORCE:
        return "右零力模式";
    case MODE_BILATERAL_ZERO_FORCE:
        return "双臂零力模式";
    case MODE_MIRROR:
        return "双臂镜像模式(零力)";
    case MODE_MIRROR_ACTIVE:
        return "双臂镜像模式(主动)";
    case MODE_ACTIVE_SPRING:
        return "双手 X 向虚拟弹簧主动模式";
    case MODE_ACTIVE_SPRING_3D:
        return "双手空间虚拟弹簧主动模式";
    case MODE_BILATERAL_ACTIVE:
        return "双侧主动模式";

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