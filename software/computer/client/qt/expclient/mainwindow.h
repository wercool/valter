#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QSlider>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>

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

    //helpers
    QPixmap redLedOffPix;
    QIcon redLedOffIcon;
    QPixmap redLedOnPix;
    QIcon redLedOnIcon;
    QPixmap greenLedOffPix;
    QIcon greenLedOffIcon;
    QPixmap greenLedOnPix;
    QIcon greenLedOnIcon;

    //platform locaiton p1
    QGraphicsLineItem *leftUSSonarVector;
    QGraphicsLineItem *rightUSSonarVector;
    QGraphicsScene* platformLocationP1SonarsGraphicsViewScene;
    map<int, QGraphicsEllipseItem*> leftSonarDots;
    map<int, QGraphicsEllipseItem*> rightSonarDots;

    QGraphicsScene* platformLocationP1AccelerometerGraphicsViewScene;
    QGraphicsScene* platformLocationP1MagnetometerGraphicsViewScene;

    QGraphicsScene* platformLocationP1CompassGraphicsViewScene;
    QGraphicsLineItem *northDirection;

    QGraphicsScene* platformLocationP1InclinometerGraphicsViewScene;
    QGraphicsLineItem *xInclination;
    QGraphicsLineItem *yInclination;
    QGraphicsLineItem *zInclination;

    Ui::MainWindow *getUi() const;
    void setUi(Ui::MainWindow *value);

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

    void on_platformControlP1ReadingsTable_itemClicked(QTableWidgetItem *item);

    void on_leftMotorPlatformControlP1DutySlider_valueChanged(int value);

    void on_rightMotorPlatformControlP1DutySlider_valueChanged(int value);

    void on_platformMovementAccelerationSlider_valueChanged(int value);

    void on_platformMovementDecelerationSlider_valueChanged(int value);

    void on_platformControlP1WheelMotorsDutySlider_valueChanged(int value);

    void on_turretRotationDutySlider_valueChanged(int value);

    void on_decelerationTurretRotationSlider_valueChanged(int value);

    void on_accelerationTurretRotationSlider_valueChanged(int value);

    void on_platformControlP1additionalReadingsTrackingDelay_valueChanged(int value);

    void on_mainTabWidget_tabBarDoubleClicked(int index);

    //PLATFORM-LOCATION-P1
    void platformLocationP1TabRefreshTimerUpdate();

    void platfromLocationP1LEDHandler();

    void on_loadDefaultsPlatformLocationP1Button_clicked();

    void on_irSensorsPresetsTable_itemClicked(QTableWidgetItem *item);

    void on_usSensorsPresetsTable_itemClicked(QTableWidgetItem *item);

    void on_LEDStatesButton_toggled(bool checked);

    void on_USSignalDutyScroller_valueChanged(int value);

    void on_USSignalBurstScroller_valueChanged(int value);

    void on_USSignalDelayScroller_valueChanged(int value);

    void on_USVoltageUpButton_clicked();

    void on_USVoltageDownButton_clicked();

    void on_leftSonarScanButton_toggled(bool checked);

    void on_rightSonarScanButton_toggled(bool checked);

    void on_leftSonarAngleScroller_valueChanged(int value);

    void on_rightSonarAngleScroller_valueChanged(int value);

    void on_leftSonarAngleScroller_sliderPressed();

    void on_rightSonarAngleScroller_sliderPressed();

    void on_leftSonarAngleScroller_sliderReleased();

    void on_rightSonarAngleScroller_sliderReleased();

    void on_detatchSonarsFrameButton_clicked();

    void on_accelerometerTrackCheckBox_toggled(bool checked);

    void on_magnetometerTrackCheckBox_toggled(bool checked);

    void on_accelerometerGraphicsViewRedrawCheckbox_toggled(bool checked);

    void platformLocationP1AccelerometerRefreshTimerTimerUpdate();
    void platformLocationP1MagnetometerRefreshTimerTimerUpdate();

    void on_detatchAccAndMagFrameButton_clicked();

    void on_magnetometerGraphicsViewRedrawCheckbox_toggled(bool checked);

    void on_compassHeadingTrackCheckBox_toggled(bool checked);

    void platformLocationP1CompassHeadingRefreshTimerUpdate();

    void on_compassGraphicsViewRedrawCheckbox_toggled(bool checked);

    void on_detatchCompassFrameButton_clicked();

    void on_inclinometerXCheckbox_toggled(bool checked);

    void on_inclinometerYCheckbox_toggled(bool checked);

    void on_inclinometerZCheckbox_toggled(bool checked);

    void on_llLedToggleButton_toggled(bool checked);

    void on_lrLedToggleButton_toggled(bool checked);

    void on_rlLedToggleButton_toggled(bool checked);

    void on_rrLedToggleButton_toggled(bool checked);

    void on_allSonarsLedsToggleButton_toggled(bool checked);

    void on_manLedToggleButton_toggled(bool checked);

    void on_inclinometerFrameDetatchButton_clicked();

    //platfrom-control-p2
    void on_chargerMotorDutyScrollBar_valueChanged(int value);

    void on_chargerMotorPushDurationScrollBar_valueChanged(int value);

    void on_horizontalScrollBar_4_valueChanged(int value);

    void on_detachIRScanningFrameButton_clicked();

    void on_chargerLedsButton_toggled(bool checked);

    void on_loadDefaultsPlatfromControlP2Button_clicked();

private:
    Ui::MainWindow *ui;
    static MainWindow* pMainWindow;
    QLabel *statusBarText;
    static bool instanceFlag;
    static const int logMaxLength = 5000;
    int logLength;
    QTimer *controlDevicesDataExchangeLogTimer;
    QTimer *delayedGUIActionsProcessingTimer;
    string selectedControlDeviceId;
    bool allConnect;

    QTimer *controlDevicesTableRefreshTimer;

    //platform-control-p1
    QTimer *platformControlP1TabRefreshTimer;

    //platform-location-p1
    QTimer *platformLocationP1TabRefreshTimer;
    QTimer *platformLocationP1AccelerometerRefreshTimer;
    QTimer *platformLocationP1MagnetometerRefreshTimer;
    QTimer *platformLocationP1CompassHeadingRefreshTimer;

    //platfrom-control-p2
};

#endif // MAINWINDOW_H
