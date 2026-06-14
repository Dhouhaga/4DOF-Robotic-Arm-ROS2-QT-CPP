#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QPushButton>
#include <QProcess>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onBaseAngleChanged(int value);
    void onShoulderAngleChanged(int value);
    void onWristAngleChanged(int value);
    void onGripperAngleChanged(int value);
    void onSendAnglesClicked();
    void onEmergencyStopClicked();
    void onResetAnglesClicked();
    void updateSimulatedData();
    void onPublishFinished(int exitCode, QProcess::ExitStatus exitStatus);
    void checkControllerConnection();
    void onConnectionCheckFinished(int exitCode, QProcess::ExitStatus exitStatus);

private:
    Ui::MainWindow *ui;
    QTimer *simulationTimer;
    QTimer *connectionCheckTimer;   // polls for controller
    QProcess *rosProcess;
    QProcess *connectionCheckProcess;
    bool connectionState;
    bool gripperGrabbed;
    int simulatedObjectX, simulatedObjectY, simulatedObjectZ;

    QSlider *baseSlider;
    QSlider *shoulderSlider;
    QSlider *wristSlider;
    QSlider *gripperSlider;
    QSpinBox *baseSpinBox;
    QSpinBox *shoulderSpinBox;
    QSpinBox *wristSpinBox;
    QSpinBox *gripperSpinBox;
    QLabel *baseAngleValue;
    QLabel *shoulderAngleValue;
    QLabel *wristAngleValue;
    QLabel *gripperAngleValue;
    QLabel *controllerStatusLabel;
    QLabel *statusDotLabel;
    QLabel *gripperStatusLabel;
    QLabel *objectStatusLabel;
    QLabel *coordinatesLabel;
    QLabel *angleVisualization;
    QLabel *publishStatusLabel;
    QPushButton *sendButton;
    QPushButton *resetButton;
    QPushButton *emergencyButton;

    void setupUI();
    void updateAngleDisplay();
    void setupConnections();
    void publishToROS2(double base, double shoulder, double wrist, double gripper);
};

#endif
