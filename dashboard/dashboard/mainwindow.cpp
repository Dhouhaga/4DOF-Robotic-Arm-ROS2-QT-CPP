#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QRandomGenerator>
#include <QTimer>
#include <QProcess>
#include <QProcessEnvironment>
#include <QDebug>

static const QString ROS_SETUP = "source /opt/ros/jazzy/setup.bash";
static const QString TOPIC     = "/forward_position_controller/commands";

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , connectionState(false)
    , gripperGrabbed(false)
    , simulatedObjectX(0), simulatedObjectY(0), simulatedObjectZ(0)
    , baseSlider(nullptr), shoulderSlider(nullptr)
    , wristSlider(nullptr), gripperSlider(nullptr)
    , baseSpinBox(nullptr), shoulderSpinBox(nullptr)
    , wristSpinBox(nullptr), gripperSpinBox(nullptr)
    , baseAngleValue(nullptr), shoulderAngleValue(nullptr)
    , wristAngleValue(nullptr), gripperAngleValue(nullptr)
    , controllerStatusLabel(nullptr), statusDotLabel(nullptr)
    , gripperStatusLabel(nullptr), objectStatusLabel(nullptr)
    , coordinatesLabel(nullptr), angleVisualization(nullptr)
    , publishStatusLabel(nullptr)
    , sendButton(nullptr), resetButton(nullptr), emergencyButton(nullptr)
    , rosProcess(nullptr), connectionCheckProcess(nullptr)
{
    ui->setupUi(this);
    setupUI();
    setupConnections();

    // Simulation timer (status panel updates)
    simulationTimer = new QTimer(this);
    connect(simulationTimer, &QTimer::timeout, this, &MainWindow::updateSimulatedData);
    simulationTimer->start(2000);

    // Connection polling — check every 3 seconds until controller is found
    connectionCheckTimer = new QTimer(this);
    connect(connectionCheckTimer, &QTimer::timeout, this, &MainWindow::checkControllerConnection);
    connectionCheckTimer->start(3000);

    // First check immediately
    QTimer::singleShot(500, this, &MainWindow::checkControllerConnection);
}

MainWindow::~MainWindow()
{
    if (rosProcess) {
        rosProcess->kill();
        rosProcess->waitForFinished(1000);
    }
    if (connectionCheckProcess) {
        connectionCheckProcess->kill();
        connectionCheckProcess->waitForFinished(1000);
    }
    delete ui;
}

// ─── Connection Check ────────────────────────────────────────────────────────

void MainWindow::checkControllerConnection()
{
    // Don't pile up processes
    if (connectionCheckProcess && connectionCheckProcess->state() != QProcess::NotRunning)
        return;

    if (connectionCheckProcess) {
        delete connectionCheckProcess;
        connectionCheckProcess = nullptr;
    }

    connectionCheckProcess = new QProcess(this);
    connect(connectionCheckProcess,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onConnectionCheckFinished);

    // ros2 topic info returns subscriber count; we grep for "Subscription count: 0"
    // If count > 0 → controller is live
    QString cmd = QString(
        "%1 && ros2 topic info %2 2>/dev/null"
    ).arg(ROS_SETUP, TOPIC);

    connectionCheckProcess->start("bash", QStringList() << "-c" << cmd);
}

void MainWindow::onConnectionCheckFinished(int exitCode, QProcess::ExitStatus)
{
    QString output = connectionCheckProcess->readAllStandardOutput();

    // "Subscription count: N" — we want N > 0
    bool hasSubscriber = false;
    if (exitCode == 0 && output.contains("Subscription count:")) {
        QRegularExpression re("Subscription count: (\\d+)");
        // Use plain string search for simplicity (no regex header needed)
        int idx = output.indexOf("Subscription count: ");
        if (idx != -1) {
            QString rest = output.mid(idx + 20).trimmed();
            int count = rest.split('\n').first().trimmed().toInt();
            hasSubscriber = (count > 0);
        }
    }

    if (hasSubscriber && !connectionState) {
        // Controller just came online
        connectionState = true;
        connectionCheckTimer->stop();

        statusDotLabel->setText("●");
        statusDotLabel->setStyleSheet("font-size: 14px; color: #27ae60;");
        controllerStatusLabel->setText("CONNECTED");
        controllerStatusLabel->setStyleSheet("font-weight: 600; color: #27ae60; font-size: 12px;");

        sendButton->setEnabled(true);
        emergencyButton->setEnabled(true);

    } else if (!hasSubscriber && connectionState) {
        // Controller went offline
        connectionState = false;
        connectionCheckTimer->start(3000); // resume polling

        statusDotLabel->setText("●");
        statusDotLabel->setStyleSheet("font-size: 14px; color: #e74c3c;");
        controllerStatusLabel->setText("DISCONNECTED — WAITING...");
        controllerStatusLabel->setStyleSheet("font-weight: 600; color: #e74c3c; font-size: 12px;");

        sendButton->setEnabled(false);
        emergencyButton->setEnabled(false);

    } else if (!hasSubscriber && !connectionState) {
        // Still waiting
        statusDotLabel->setText("●");
        statusDotLabel->setStyleSheet("font-size: 14px; color: #f39c12;");
        controllerStatusLabel->setText("WAITING FOR CONTROLLER...");
        controllerStatusLabel->setStyleSheet("font-weight: 600; color: #f39c12; font-size: 12px;");

        sendButton->setEnabled(false);
        emergencyButton->setEnabled(false);
    }
}

// ─── ROS2 Publish ────────────────────────────────────────────────────────────

void MainWindow::publishToROS2(double base, double shoulder, double wrist, double gripper)
{
    if (rosProcess) {
        rosProcess->kill();
        rosProcess->waitForFinished(500);
        delete rosProcess;
    }

    rosProcess = new QProcess(this);
    connect(rosProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::onPublishFinished);

    // Inherit the current process environment (carries ROS_DOMAIN_ID,
    // RMW_IMPLEMENTATION, etc. that were set before launching the dashboard)
    // then layer the ROS2 setup.bash on top inside the bash -c call.
    rosProcess->setProcessEnvironment(QProcessEnvironment::systemEnvironment());

    QString cmd = QString(
        "%1 && ros2 topic pub -1 %2 "
        "std_msgs/msg/Float64MultiArray \"{data: [%3, %4, %5, %6]}\""
    ).arg(ROS_SETUP, TOPIC)
     .arg(base,     0, 'f', 1)
     .arg(shoulder, 0, 'f', 1)
     .arg(wrist,    0, 'f', 1)
     .arg(gripper,  0, 'f', 1);

    rosProcess->start("bash", QStringList() << "-c" << cmd);

    publishStatusLabel->setText("⟳ PUBLISHING...");
    publishStatusLabel->setStyleSheet("color: #f39c12; font-size: 11px; font-weight: 600;");
}

void MainWindow::onPublishFinished(int exitCode, QProcess::ExitStatus)
{
    if (exitCode == 0) {
        publishStatusLabel->setText("✓ PUBLISHED");
        publishStatusLabel->setStyleSheet("color: #27ae60; font-size: 11px; font-weight: 600;");
    } else {
        publishStatusLabel->setText("✗ FAILED");
        publishStatusLabel->setStyleSheet("color: #e74c3c; font-size: 11px; font-weight: 600;");
        QString err = rosProcess->readAllStandardError();
        QString out = rosProcess->readAllStandardOutput();
        qWarning() << "ROS2 publish failed (exit" << exitCode << ")";
        if (!err.isEmpty()) qWarning() << "STDERR:" << err;
        if (!out.isEmpty()) qWarning() << "STDOUT:" << out;
    }
    QTimer::singleShot(3000, [this]() {
        publishStatusLabel->setText("IDLE");
        publishStatusLabel->setStyleSheet("color: #95a5a6; font-size: 11px; font-weight: 600;");
    });
}

// ─── UI Setup ────────────────────────────────────────────────────────────────

void MainWindow::setupUI()
{
    this->setWindowTitle("4DOF Robotic Arm Dashboard");
    this->setMinimumSize(800, 600);
    this->showMaximized();
    this->setStyleSheet("QMainWindow { background-color: #f5f5f5; }");

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setStyleSheet("background-color: #f5f5f5;");
    this->setCentralWidget(centralWidget);

    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->setSpacing(20);
    mainLayout->setContentsMargins(30, 30, 30, 30);

    // ── Header ──
    QWidget *headerWidget = new QWidget();
    headerWidget->setStyleSheet("background-color: #ffffff; border-radius: 8px;");
    QHBoxLayout *headerLayout = new QHBoxLayout(headerWidget);

    QLabel *titleLabel = new QLabel("4DOF ROBOTIC ARM CONTROL SYSTEM");
    titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #2c3e50; letter-spacing: 1px;");

    QWidget *statusWidget = new QWidget();
    QHBoxLayout *statusLayout = new QHBoxLayout(statusWidget);
    statusLayout->setContentsMargins(0, 0, 0, 0);

    statusDotLabel = new QLabel("●");
    statusDotLabel->setStyleSheet("font-size: 14px; color: #f39c12;");

    controllerStatusLabel = new QLabel("WAITING FOR CONTROLLER...");
    controllerStatusLabel->setStyleSheet("font-weight: 600; color: #f39c12; font-size: 12px;");

    statusLayout->addWidget(statusDotLabel);
    statusLayout->addWidget(controllerStatusLabel);

    headerLayout->addWidget(titleLabel);
    headerLayout->addStretch();
    headerLayout->addWidget(statusWidget);
    mainLayout->addWidget(headerWidget);

    // ── Content ──
    QHBoxLayout *contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(20);

    // Left Panel
    QWidget *leftPanel = new QWidget();
    leftPanel->setStyleSheet("background-color: #ffffff; border-radius: 8px;");
    QVBoxLayout *leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setSpacing(20);
    leftLayout->setContentsMargins(25, 25, 25, 25);

    QLabel *controlsTitle = new QLabel("JOINT CONTROLS");
    controlsTitle->setStyleSheet("font-weight: bold; font-size: 14px; color: #2c3e50; padding-bottom: 5px; border-bottom: 2px solid #3498db;");
    leftLayout->addWidget(controlsTitle);

    auto addJointControl = [&](const QString& name, QSlider*& slider, QSpinBox*& spinBox, QLabel*& angleLabel) {
        QVBoxLayout *jointLayout = new QVBoxLayout();
        jointLayout->setSpacing(8);
        QLabel *nameLabel = new QLabel(name);
        nameLabel->setStyleSheet("font-weight: 600; color: #34495e; font-size: 12px;");
        jointLayout->addWidget(nameLabel);

        QHBoxLayout *controlLayout = new QHBoxLayout();
        controlLayout->setSpacing(10);

        slider = new QSlider(Qt::Horizontal);
        slider->setRange(0, 180);
        slider->setValue(90);
        slider->setStyleSheet(
            "QSlider::groove:horizontal { height: 4px; background: #ecf0f1; border-radius: 2px; }"
            "QSlider::handle:horizontal { background: #3498db; width: 14px; height: 14px; border-radius: 7px; margin: -5px 0; }"
            "QSlider::sub-page:horizontal { background: #3498db; border-radius: 2px; }"
        );

        spinBox = new QSpinBox();
        spinBox->setRange(0, 180);
        spinBox->setValue(90);
        spinBox->setStyleSheet(
            "QSpinBox { padding: 4px 8px; border: 1px solid #bdc3c7; border-radius: 4px; background: white; font-size: 12px; }"
            "QSpinBox:focus { border-color: #3498db; }"
        );
        spinBox->setFixedWidth(70);

        angleLabel = new QLabel("90°");
        angleLabel->setStyleSheet("font-weight: 600; color: #3498db; font-size: 13px; min-width: 45px;");
        angleLabel->setAlignment(Qt::AlignRight);

        controlLayout->addWidget(slider);
        controlLayout->addWidget(spinBox);
        controlLayout->addWidget(angleLabel);
        jointLayout->addLayout(controlLayout);
        leftLayout->addLayout(jointLayout);

        connect(slider,  &QSlider::valueChanged, spinBox, &QSpinBox::setValue);
        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);
    };

    addJointControl("WRIST JOINT",     baseSlider,    baseSpinBox,    baseAngleValue);
    addJointControl("SHOULDER JOINT", shoulderSlider, shoulderSpinBox, shoulderAngleValue);
    addJointControl("GRIPPER JOINT",    wristSlider,   wristSpinBox,   wristAngleValue);
    addJointControl("BASE JOINT",  gripperSlider, gripperSpinBox, gripperAngleValue);

    leftLayout->addStretch();

    // Buttons
    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(15);

    sendButton = new QPushButton("SEND COMMAND");
    sendButton->setEnabled(false);   // disabled until controller found
    sendButton->setStyleSheet(
        "QPushButton { background-color: #3498db; color: white; padding: 10px 20px; border-radius: 6px; font-weight: 600; font-size: 12px; border: none; }"
        "QPushButton:hover { background-color: #2980b9; }"
        "QPushButton:pressed { background-color: #21618c; }"
        "QPushButton:disabled { background-color: #bdc3c7; color: #ecf0f1; }"
    );

    resetButton = new QPushButton("RESET DEFAULTS");
    resetButton->setStyleSheet(
        "QPushButton { background-color: #95a5a6; color: white; padding: 10px 20px; border-radius: 6px; font-weight: 600; font-size: 12px; border: none; }"
        "QPushButton:hover { background-color: #7f8c8d; }"
    );

    emergencyButton = new QPushButton("EMERGENCY STOP");
    emergencyButton->setEnabled(false);  // disabled until controller found
    emergencyButton->setStyleSheet(
        "QPushButton { background-color: #e74c3c; color: white; padding: 10px 20px; border-radius: 6px; font-weight: 600; font-size: 12px; border: none; }"
        "QPushButton:hover { background-color: #c0392b; }"
        "QPushButton:disabled { background-color: #f1948a; color: #fadbd8; }"
    );

    buttonLayout->addWidget(sendButton);
    buttonLayout->addWidget(resetButton);
    buttonLayout->addWidget(emergencyButton);
    leftLayout->addLayout(buttonLayout);

    // Publish status
    QHBoxLayout *pubStatusLayout = new QHBoxLayout();
    QLabel *pubStatusTitle = new QLabel("ROS2 PUBLISH:");
    pubStatusTitle->setStyleSheet("color: #95a5a6; font-size: 11px;");
    publishStatusLabel = new QLabel("IDLE");
    publishStatusLabel->setStyleSheet("color: #95a5a6; font-size: 11px; font-weight: 600;");
    pubStatusLayout->addStretch();
    pubStatusLayout->addWidget(pubStatusTitle);
    pubStatusLayout->addWidget(publishStatusLabel);
    leftLayout->addLayout(pubStatusLayout);

    // Right Panel
    QWidget *rightPanel = new QWidget();
    rightPanel->setStyleSheet("background-color: #ffffff; border-radius: 8px;");
    QVBoxLayout *rightLayout = new QVBoxLayout(rightPanel);
    rightLayout->setSpacing(20);
    rightLayout->setContentsMargins(25, 25, 25, 25);

    QLabel *statusTitle = new QLabel("SYSTEM STATUS");
    statusTitle->setStyleSheet("font-weight: bold; font-size: 14px; color: #2c3e50; padding-bottom: 5px; border-bottom: 2px solid #3498db;");
    rightLayout->addWidget(statusTitle);

    // Gripper
    QVBoxLayout *gripperLayout = new QVBoxLayout();
    gripperLayout->setSpacing(8);
    QLabel *gripperLabel = new QLabel("GRIPPER STATE");
    gripperLabel->setStyleSheet("font-weight: 600; color: #34495e; font-size: 12px;");
    gripperLayout->addWidget(gripperLabel);
    gripperStatusLabel = new QLabel("IDLE");
    gripperStatusLabel->setStyleSheet("background-color: #ecf0f1; color: #7f8c8d; padding: 10px; border-radius: 6px; font-weight: 500; font-size: 12px;");
    gripperLayout->addWidget(gripperStatusLabel);
    rightLayout->addLayout(gripperLayout);

    // Object Detection
    QVBoxLayout *objectLayout = new QVBoxLayout();
    objectLayout->setSpacing(8);
    QLabel *objectLabel = new QLabel("OBJECT DETECTION");
    objectLabel->setStyleSheet("font-weight: 600; color: #34495e; font-size: 12px;");
    objectLayout->addWidget(objectLabel);
    objectStatusLabel = new QLabel("SCANNING...");
    objectStatusLabel->setStyleSheet("background-color: #ecf0f1; color: #7f8c8d; padding: 10px; border-radius: 6px; font-size: 11px;");
    objectLayout->addWidget(objectStatusLabel);
    coordinatesLabel = new QLabel("X: -- | Y: -- | Z: --");
    coordinatesLabel->setStyleSheet("background-color: #f8f9fa; color: #34495e; padding: 8px; border-radius: 4px; font-family: monospace; font-size: 11px;");
    objectLayout->addWidget(coordinatesLabel);
    rightLayout->addLayout(objectLayout);

    // Angle Visualization
    QVBoxLayout *vizLayout = new QVBoxLayout();
    vizLayout->setSpacing(8);
    QLabel *vizLabel = new QLabel("ANGLE VISUALIZATION");
    vizLabel->setStyleSheet("font-weight: 600; color: #34495e; font-size: 12px;");
    vizLayout->addWidget(vizLabel);
    angleVisualization = new QLabel();
    angleVisualization->setStyleSheet("background-color: #f8f9fa; border-radius: 6px; padding: 15px; font-family: monospace; font-size: 11px; color: #2c3e50;");
    angleVisualization->setAlignment(Qt::AlignCenter);
    vizLayout->addWidget(angleVisualization);
    rightLayout->addLayout(vizLayout);

    rightLayout->addStretch();

    contentLayout->addWidget(leftPanel, 2);
    contentLayout->addWidget(rightPanel, 1);
    mainLayout->addLayout(contentLayout);

    updateAngleDisplay();
}

void MainWindow::setupConnections()
{
    connect(baseSlider,     &QSlider::valueChanged, this, &MainWindow::onBaseAngleChanged);
    connect(shoulderSlider, &QSlider::valueChanged, this, &MainWindow::onShoulderAngleChanged);
    connect(wristSlider,    &QSlider::valueChanged, this, &MainWindow::onWristAngleChanged);
    connect(gripperSlider,  &QSlider::valueChanged, this, &MainWindow::onGripperAngleChanged);
    connect(sendButton,      &QPushButton::clicked, this, &MainWindow::onSendAnglesClicked);
    connect(resetButton,     &QPushButton::clicked, this, &MainWindow::onResetAnglesClicked);
    connect(emergencyButton, &QPushButton::clicked, this, &MainWindow::onEmergencyStopClicked);
}

// ─── Slots ───────────────────────────────────────────────────────────────────

void MainWindow::onBaseAngleChanged(int value)     { baseAngleValue->setText(QString::number(value) + "°");     updateAngleDisplay(); }
void MainWindow::onShoulderAngleChanged(int value) { shoulderAngleValue->setText(QString::number(value) + "°"); updateAngleDisplay(); }
void MainWindow::onWristAngleChanged(int value)    { wristAngleValue->setText(QString::number(value) + "°");    updateAngleDisplay(); }
void MainWindow::onGripperAngleChanged(int value)  { gripperAngleValue->setText(QString::number(value) + "°");  updateAngleDisplay(); }

void MainWindow::onSendAnglesClicked()
{
    double base = baseSlider->value(), shoulder = shoulderSlider->value(),
           wrist = wristSlider->value(), gripper = gripperSlider->value();

    publishToROS2(base, shoulder, wrist, gripper);

    sendButton->setStyleSheet(
        "QPushButton { background-color: #27ae60; color: white; padding: 10px 20px; border-radius: 6px; font-weight: 600; font-size: 12px; border: none; }"
    );
    QTimer::singleShot(300, [this]() {
        sendButton->setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; padding: 10px 20px; border-radius: 6px; font-weight: 600; font-size: 12px; border: none; }"
            "QPushButton:hover { background-color: #2980b9; }"
            "QPushButton:disabled { background-color: #bdc3c7; color: #ecf0f1; }"
        );
    });

    QMessageBox::information(this, "Command Sent",
        QString("Sent to %1:\n[%2, %3, %4, %5]")
            .arg(TOPIC).arg(base).arg(shoulder).arg(wrist).arg(gripper));
}

void MainWindow::onEmergencyStopClicked()
{
    auto reply = QMessageBox::warning(this, "EMERGENCY STOP",
        "WARNING: This will immediately stop all robot movement!\n\nAre you sure?",
        QMessageBox::Yes | QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        publishToROS2(0.0, 0.0, 0.0, 0.0);
        QMessageBox::critical(this, "Emergency Stop",
            "Emergency stop activated!\nZero command sent to all joints.");
    }
}

void MainWindow::onResetAnglesClicked()
{
    baseSlider->setValue(90);
    shoulderSlider->setValue(90);
    wristSlider->setValue(90);
    gripperSlider->setValue(90);
    QMessageBox::information(this, "Reset", "All joints reset to 90 degrees.");
}

void MainWindow::updateAngleDisplay()
{
    angleVisualization->setText(
        QString("BASE:      %1°\nSHOULDER:  %2°\nWRIST:     %3°\nGRIPPER:   %4°")
            .arg(baseSlider->value()).arg(shoulderSlider->value())
            .arg(wristSlider->value()).arg(gripperSlider->value())
    );
}

void MainWindow::updateSimulatedData()
{
    static int counter = 0;
    counter++;

    if (!connectionState) return;

    if (counter % 3 == 0) {
        gripperGrabbed = !gripperGrabbed;
        if (gripperGrabbed) {
            QStringList objects = {"CUBE", "SPHERE", "CYLINDER", "BOX"};
            gripperStatusLabel->setText(QString("GRABBED: %1").arg(objects[counter % objects.size()]));
            gripperStatusLabel->setStyleSheet("background-color: #d5f4e6; color: #27ae60; padding: 10px; border-radius: 6px; font-weight: 500; font-size: 12px;");
        } else {
            gripperStatusLabel->setText("OPEN");
            gripperStatusLabel->setStyleSheet("background-color: #fadbd8; color: #e74c3c; padding: 10px; border-radius: 6px; font-weight: 500; font-size: 12px;");
        }
    }

    if (counter % 2 == 0) {
        simulatedObjectX = QRandomGenerator::global()->bounded(-100, 100);
        simulatedObjectY = QRandomGenerator::global()->bounded(0, 200);
        simulatedObjectZ = QRandomGenerator::global()->bounded(-50, 150);
        QStringList detectedObjects = {"METAL BLOCK", "PLASTIC CONTAINER", "WOODEN PIECE", "GLASS BOTTLE"};
        objectStatusLabel->setText(detectedObjects[counter % detectedObjects.size()]);
        objectStatusLabel->setStyleSheet("background-color: #e8daef; color: #8e44ad; padding: 10px; border-radius: 6px; font-size: 11px; font-weight: 500;");
        coordinatesLabel->setText(QString("X: %1 cm | Y: %2 cm | Z: %3 cm")
            .arg(simulatedObjectX).arg(simulatedObjectY).arg(simulatedObjectZ));
    } else if (counter % 4 == 1) {
        objectStatusLabel->setText("NONE");
        objectStatusLabel->setStyleSheet("background-color: #ecf0f1; color: #7f8c8d; padding: 10px; border-radius: 6px; font-size: 11px;");
        coordinatesLabel->setText("X: -- | Y: -- | Z: --");
    }
}
