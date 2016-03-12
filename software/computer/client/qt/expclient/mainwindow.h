#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QSlider>
#include <QMessageBox>

#include <valter.h>
#include <ivaltermodule.h>

namespace Ui {
class MainWindow;
}
using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    static MainWindow* getInstance();
    void refreshControlDeviceTableWidget();
    void addMsgToLog(string msg);
    void delayGUIAction(IValterModule *valterModule);

protected:
    bool sliderWheelEventFilter(QObject *obj, QEvent *event)
    {
        (void) obj;
        (void) event;
        return false;
    }

private slots:
    void on_scanControlDevicesBtn_clicked();

    void on_connectDisconnectControlDeviceButton_clicked();

    void on_controlDeviceTableWidget_clicked(const QModelIndex &index);

    void on_selectedControlDeviceListWidget_doubleClicked(const QModelIndex &index);

    void on_clearCommandButton_clicked();

    void on_clearLogButton_clicked();

    void on_pauseLoggingButton_clicked();

    void on_sendCommandButton_clicked();

    void controlDevicesDataExchangeLogTimerUpdate();

    void controlDevicesTableRefreshTimerUpdate();

    void on_connectAllPushButton_clicked();

    void on_wdResetStopButton_clicked();

    void on_reScanControlDevicesButton_clicked();

    void on_wdResetOnButton_clicked();

    void on_stopAllPlatformControlP1Button_clicked();

    void platformControlP1TabRefreshTimerUpdate();

    void delayedGUIActionsProcessingTimerUpdate();

    void on_platformControlP1WheelMotorsDutySlider_sliderMoved(int dutyValue);

    void on_leftMotorPlatformControlP1DutySlider_sliderMoved(int value);

    void on_rightMotorPlatformControlP1DutySlider_sliderMoved(int value);

    void on_on5VPlatformControlP1pushButton_clicked();

    void on_off5VPlatformControlP1pushButton_clicked();

    void on_onLeftAccumulatorPlatformControlP1Button_clicked();

    void on_offLeftAccumulatorPlatformControlP1Button_clicked();

    void on_onRightAccumulatorPlatformControlP1Button_clicked();

    void on_offRightAccumulatorPlatformControlP1Button_clicked();

    void on_scan220VAOnCButton_clicked();

    void on_scan220VAOffCButton_clicked();

    void on_onMainAccumulatorRelayPlatformControlP1Button_clicked();

    void on_offMainAccumulatorRelayPlatformControlP1Button_clicked();

    void on_onLeftAccumulatorRelayPlatformControlP1Button_clicked();

    void on_offLeftAccumulatorRelayPlatformControlP1Button_clicked();

    void on_onRightAccumulatorRelayPlatformControlP1Button_clicked();

    void on_offRightAccumulatorRelayPlatformControlP1Button_clicked();

    void on_chargerButton_clicked();

    void on_setChargeOnButton_clicked();

    void on_setChargeOffButton_clicked();

    void on_platformMoveStopButton_clicked();

    void on_platformMovementAccelerationSlider_sliderMoved(int value);

    void on_platformMovementDecelerationSlider_sliderMoved(int value);

    void on_platformMoveForwardButton_pressed();

    void on_platformMoveForwardButton_released();

    void on_platformBackwardForwardButton_pressed();

    void on_platformBackwardForwardButton_released();

    void on_platformMoveForwardLeftButton_pressed();

    void on_platformMoveForwardLeftButton_released();

    void on_platformMoveForwardRightButton_pressed();

    void on_platformMoveForwardRightButton_released();

    void on_platformMoveBackwardLeftButton_pressed();

    void on_platformMoveBackwardRightButton_pressed();

    void on_platformMoveBackwardLeftButton_released();

    void on_platformMoveBackwardRightButton_released();

    void on_platformRotateLeftButton_pressed();

    void on_platformRotateRightButton_pressed();

    void on_platformRotateLeftButton_released();

    void on_platformRotateRightButton_released();

    void on_turretRotationDutySlider_sliderMoved(int value);

    void on_decelerationTurretRotationSlider_sliderMoved(int value);

    void on_accelerationTurretRotationSlider_sliderMoved(int value);

    void on_turretRotateLeftButton_pressed();

    void on_turretRotateLeftButton_released();

    void on_turretRotateRightButton_pressed();

    void on_bodyRotationStopButton_clicked();

    void on_turretRotateRightButton_released();

    void on_platformControlP1LoadDefaultsButton_clicked();


    void on_platformControlP1MotorsPWMFrequencySetButton_clicked();

    void on_leftMotorCurrentCheckBox_clicked();

    void on_rightMotorCurrentCheckBox_clicked();

    void on_turretMotorCurrentCheckBox_clicked();

    void on_platformControlP1LeftWheelEncoderResetButton_clicked();

    void on_platformControlP1RightWheelEncoderResetButton_clicked();

    void on_platformControlP1LeftWheelEncoderGetButton_clicked();

    void on_platformControlP1RightWheelEncoderGetButton_clicked();

    void on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked(bool checked);

    void on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked(bool checked);

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    static MainWindow* pMainWindow;
    QLabel *statusBarText;
    static bool instanceFlag;
    static const int logMaxLength = 5000;
    int logLength;
    QTimer *controlDevicesDataExchangeLogTimer;
    QTimer *controlDevicesTableRefreshTimer;
    QTimer *platformControlP1TabRefreshTimer;
    QTimer *delayedGUIActionsProcessingTimer;
    string selectedControlDeviceId;
    bool allConnect;

};

#endif // MAINWINDOW_H
