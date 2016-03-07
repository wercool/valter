#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include <valter.h>

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
    string selectedControlDeviceId;
    bool allConnect;
};

#endif // MAINWINDOW_H
