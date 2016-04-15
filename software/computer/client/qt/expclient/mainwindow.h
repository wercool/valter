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
#include <QGraphicsTextItem>

#include <QtQuick/QQuickView>

#include <valter.h>
#include <ivaltermodule.h>

#include <valter3d.h>

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

    Valter3d* valter3d;

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

    //platfrom-control-p2
    QTimer *platformControlP2EncodersRefreshTimer;
    QGraphicsScene* irScanningGraphicsViewScene;
    QTimer *platformControlP2IRScannerRefreshTimer;
    QGraphicsLineItem *IRScannerVector;
    map<int, QGraphicsEllipseItem*> IRScannerDots;

    //platformmanipulatorandirbumper
    QGraphicsScene* platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene;
    QGraphicsLineItem *platfromManipulatorLink1;
    QGraphicsLineItem *platfromManipulatorLink1Initial;
    QGraphicsLineItem *platfromManipulatorLink1Helper;
    QGraphicsLineItem *platfromManipulatorLink2;
    QGraphicsLineItem *platfromManipulatorLink2Initial;
    QGraphicsLineItem *platfromManipulatorLink2Helper;
    QGraphicsLineItem *platfromManipulatorLink3;
    QGraphicsLineItem *platfromManipulatorLink3Initial;
    QGraphicsLineItem *platfromManipulatorLink3Helper;
    QGraphicsLineItem *platfromManipulatorLink1Link2Console;
    QGraphicsLineItem *platfromManipulatorLink2Link3Console;
    QGraphicsEllipseItem *link1RootPoint;
    QGraphicsEllipseItem *link1link2Point;
    QGraphicsEllipseItem *link2link3Point;
    QGraphicsEllipseItem *link1EndPoint;
    QGraphicsEllipseItem *link2EndPoint;
    QGraphicsEllipseItem *link3EndPoint;
    QGraphicsTextItem * alpha;
    QGraphicsTextItem * beta;
    QGraphicsTextItem * gamma;

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
    //Control Devices
    void initControlDevices(Ui::MainWindow *ui);

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

    void on_resetControlDeviceButton_clicked();

    //platfrom control p1
    void initPlatfromControlP1(Ui::MainWindow *ui);

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

    void on_valterPlatformMovementControlsDetachButton_clicked();

    //PLATFORM-LOCATION-P1
    void initPlatfromLocationP1(Ui::MainWindow *ui);

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

    void platformLocationP1AccelerometerRefreshTimerUpdate();
    void platformLocationP1MagnetometerRefreshTimerUpdate();

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

    void on_platformLocationP1EnableSensorsButton_clicked();

    void on_platformLocationP1DisableSensorsButton_clicked();

    void on_platformLocationP1AllLEDsONButton_clicked();

    void on_platformLocationP1AllLEDsOFFButton_clicked();

    void on_leftSonarReleaseButton_clicked();

    void on_rightSonarReleaseButton_clicked();

    void on_updateCompassHeadingButton_clicked();

    void on_updateAccelerometerButton_clicked();

    void on_updateMagnetometerButton_clicked();

    //platfrom-control-p2
    void initPlatfromControlP2(Ui::MainWindow *ui);

    void on_chargerMotorDutyScrollBar_valueChanged(int value);

    void on_chargerMotorPushDurationScrollBar_valueChanged(int value);

    void on_detachIRScanningFrameButton_clicked();

    void on_chargerLedsButton_toggled(bool checked);

    void on_loadDefaultsPlatfromControlP2Button_clicked();

    void on_platformControlP2LeftWheelEncoderCheckBox_toggled(bool checked);

    void on_platformControlP2RightWheelEncoderCheckBox_toggled(bool checked);

    void on_platformControlP2LeftWheelEncoderResetButton_clicked();

    void on_platformControlP2RightWheelEncoderResetButton_clicked();

    void on_beepDurationScrollBar_valueChanged(int value);

    void on_platformControlP2ResetIRScannerServoButton_clicked();

    //platfrom control p2
    void platformControlP2EncodersRefreshTimerUpdate();

    void on_platformControlP2LeftWheelEncoderGetButton_clicked();

    void on_platformControlP2RightWheelEncoderGetButton_clicked();

    void on_irScanningButton_toggled(bool checked);

    void platformControlP2IRScannerRefreshTimerUpdate();

    void on_irScannerAngleScrollBar_valueChanged(int value);

    void on_irScannerAngleScrollBar_sliderPressed();

    void on_irScannerAngleScrollBar_sliderReleased();

    void on_chargerMotorPushCCWButton_clicked();

    void on_chargerMotorPushCWButton_clicked();

    void on_chargerMotorRotateCCWButton_pressed();

    void on_chargerMotorRotateCWButton_pressed();

    void on_chargerMotorRotateCCWButton_released();

    void on_chargerMotorRotateCWButton_released();

    void on_beepButton_clicked();

    void on_alarmButton_toggled(bool checked);

    void on_bottomFronLedsButton_toggled(bool checked);

    void on_bottomRearLedsButton_toggled(bool checked);

    void on_platformManipulatorAndIRBumperLoadDefaultsButton_clicked();

    void on_manipulatorLiknk1MotorDutyScroller_valueChanged(int value);

    void on_manipulatorLiknk1DecelerationScroller_valueChanged(int value);

    void on_manipulatorLiknk1AccelerationScroller_valueChanged(int value);

    void on_manipulatorLiknk2MotorDutyScroller_valueChanged(int value);

    void on_manipulatorLiknk2DecelerationScroller_valueChanged(int value);

    void on_manipulatorLiknk2AccelerationScroller_valueChanged(int value);

    void on_powerOnOff24VPlatfromManipulatorAndIRBumperButton_clicked();

    void on_manipulatorLiknk1AscentButton_released();

    void on_manipulatorLiknk1DescentButton_released();

    void on_manipulatorLiknk2AscentButton_pressed();

    void on_manipulatorLiknk2AscentButton_released();

    void on_manipulatorLiknk2DescentButton_pressed();

    void on_manipulatorLiknk2DescentButton_released();

    void on_gripperTiltAscentButton_pressed();

    void on_gripperTiltDescentButton_released();

    void on_gripperTiltDescentButton_pressed();

    void on_gripperTiltAscentButton_released();

    void on_manGripperOpenButton_pressed();

    void on_manGripperOpenButton_released();

    void on_manGripperCloseButton_pressed();

    void on_manGripperCloseButton_released();

    void on_manGripperRotationMotorDutyScroller_valueChanged(int value);

    void on_manGripperRotateCCW_pressed();

    void on_manGripperRotateCCW_released();

    void on_manGripperRotateCW_pressed();

    void on_manGripperRotateCW_released();

    //platform-manipulator-and-ir-bumper
    void initPlatformManipulatorAndIRBumper(Ui::MainWindow *ui);

    void platformManipulatorAndIRBumperRefreshTimerUpdate();

    void on_platformManipulatorAndIRBumperRedrawGUICheckBox_toggled(bool checked);

    void on_platformManipulatorExecuteAngleSettingsButton_clicked();

    void on_detachManipulatorFrameButton_clicked();

    void on_manipulatorLiknk1AscentButton_pressed();

    void on_manipulatorLiknk1DescentButton_pressed();

    void on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked(QTableWidgetItem *item);

    //Utilities

    //Valter 3D
    void on_valter3dOpenButton_clicked();

    void on_horizontalScrollBar_9_valueChanged(int value);

    void on_horizontalScrollBar_10_valueChanged(int value);









private:
    Ui::MainWindow *ui;
    static MainWindow* pMainWindow;
    QLabel *statusBarText;
    static bool instanceFlag;

    //control devices
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

    //platform-manipulator-and-ir-bumper
    QTimer *platformManipulatorAndIRBumperRefreshTimer;

    // QWidget interface
protected:
};

#endif // MAINWINDOW_H
