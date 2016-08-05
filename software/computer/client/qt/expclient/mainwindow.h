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

    void on_stopAllButton_clicked();

protected:
    bool sliderWheelEventFilter(QObject *obj, QEvent *event)
    {
        (void) obj;
        (void) event;
        return false;
    }

public slots:
    void on_stopAllButton1_clicked();
    void on_stopAllButton2_clicked();
    void on_stopAllButton3_clicked();
    void on_stopAllButton4_clicked();
    void on_stopAllButton5_clicked();
    void on_stopAllButton6_clicked();
    void on_stopAllButton7_clicked();

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
    void on_stopSelectedControlDeviceLogButton_clicked();
    void on_startSelectedControlDeviceLogButton_clicked();
    void refreshControlDeviceTableWidgetWorker();

    //platfrom-control-p1
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
    void on_platformControlP1LeftWheelEncoderCheckBox_clicked(bool checked);
    void on_platformControlP1RightWheelEncoderCheckBox_clicked(bool checked);
    void on_turretMotorCurrentCheckBox_clicked(bool checked);
    void on_platformControlP1LeftWheelEncoderResetButton_clicked();
    void on_platformControlP1RightWheelEncoderResetButton_clicked();
    void on_platformControlP1LeftWheelEncoderGetButton_clicked();
    void on_platformControlP1RightWheelEncoderGetButton_clicked();
    void on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked(bool checked);
    void on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked(bool checked);
    void on_getBodyRotationPositionButton_clicked();
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
    void on_leftMotorCurrentCheckBox_clicked(bool checked);
    void on_rightMotorCurrentCheckBox_clicked(bool checked);
    void on_platformControlP1MotorsPWMFrequencySpinBox_valueChanged(int value);
    void on_wheelMotorsCurrentADCCheckBox_clicked(bool checked);
    void on_turretMotorCurrentADCCheckBox_clicked(bool checked);
    void on_turretPositionReadCheckBox_clicked(bool checked);
    void on_chargerVoltageADCCheckBox_clicked(bool checked);
    void on_platformControlP1RedrawGUICheckBox_clicked(bool checked);
    void on_platformControlP1UntrackAllAdditionalReadingsButton_clicked();

    //platform-location-p1
    void initPlatfromLocationP1(Ui::MainWindow *ui);
    void on_platformLocationP1RedrawGUICheckBox_clicked(bool checked);
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
    void on_enableAllIRSensorsButton_clicked();
    void on_disableAllIRSensorsButton_clicked();
    void on_enableAllUSSensorsButton_clicked();
    void on_disableAllUSSensorsButton_clicked();
    void platfromLocationP1LEDOnOff(string ledButtonName_stdString);
    void on_accelerometerRawCheckBox_clicked(bool checked);
    void on_magnetometerRawCheckBox_clicked(bool checked);

    //platfrom-control-p2
    void initPlatfromControlP2(Ui::MainWindow *ui);
    void on_platfromControlP2RedrawGUICheckBox_clicked(bool checked);
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
    void on_platformControlP2GetBottomIRReadingButton_clicked();
    void on_platformControlP2LeftWheelEncoderCheckBox_clicked(bool checked);
    void on_platformControlP2RightWheelEncoderCheckBox_clicked(bool checked);
    void on_platformControlP2LeftWheelEncoderAutoresetCheckBox_clicked(bool checked);
    void on_platformControlP2RightWheelEncoderAutoresetCheckBox_clicked(bool checked);
    void on_platfromControlP2IRScannerADCCheckBox_clicked(bool checked);

    //platform-manipulator-and-ir-bumper
    void initPlatformManipulatorAndIRBumper(Ui::MainWindow *ui);
    void platformManipulatorAndIRBumperRefreshTimerUpdate();
    void on_platformManipulatorAndIRBumperRedrawGUICheckBox_toggled(bool checked);
    void on_platformManipulatorExecuteAngleSettingsButton_clicked();
    void on_detachManipulatorFrameButton_clicked();
    void on_manipulatorLiknk1AscentButton_pressed();
    void on_manipulatorLiknk1DescentButton_pressed();
    void on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked(QTableWidgetItem *item);
    void on_irBumperEnableButton_clicked();
    void on_irBumperDisableButton_clicked();
    void on_irBumperInitButton_clicked();
    void on_irBumperFrequencySpin_valueChanged(int value);
    void on_irBumperDutySpin_valueChanged(int value);
    void on_irBumperTrackAllButton_clicked();
    void on_irBumperTrackNoneButton_clicked();
    void on_platformManipulatorAndIRBumperRedrawGUICheckBox_clicked(bool checked);
    void on_platformManipulatorAngleASpinBox_valueChanged(double value);
    void on_platformManipulatorAngleBSpinBox_valueChanged(double value);
    void on_platformManipulatorAngleGSpinBox_valueChanged(double value);
    void on_irBumperReadingsTable_itemClicked(QTableWidgetItem *item);
    void on_platformManipulatorAndIRBumperTrackAllReadingsButton_clicked();
    void on_platformManipulatorAndIRBumperTrackNoneReadingsButton_clicked();

    //body-control-p1
    void initBodyControlP1(Ui::MainWindow *ui);
    void bodyControlP1TabRefreshTimerUpdate();
    void on_bodyControlP1CheckBox_clicked(bool checked);
    void on_headYawLeftDirectonCheckButton_toggled(bool checked);
    void on_headYawRightDirectonCheckButton_toggled(bool checked);
    void on_headYawLeftRotateButton_pressed();
    void on_headYawLeftRotateButton_released();
    void on_headYawStepDelaySpinBox_valueChanged(int value);
    void on_headYawStepSwitchDelaySpinBox_valueChanged(int value);
    void on_headYawRightRotateButton_pressed();
    void on_headYawRightRotateButton_released();
    void on_pitchHeadDownDirectionCheckButton_toggled(bool checked);
    void on_pitchHeadUpDirectionCheckButton_toggled(bool checked);
    void on_headPitchStepDelaySpinBox_valueChanged(int value);
    void on_headPitchStepSwitchDelaySpinBox_valueChanged(int value);
    void on_headPitchDownButton_pressed();
    void on_headPitchDownButton_released();
    void on_headPitchUpButton_pressed();
    void on_headPitchUpButton_released();
    void on_head24VOnOffButton_clicked();
    void on_headYawMotorEnableDisableButton_clicked();
    void on_headPitchMotorEnableDisableButton_clicked();
    void on_bodyPitchMotorDutyScroller_valueChanged(int value);
    void on_bodyPitchMotorDecelerationScroller_valueChanged(int value);
    void on_bodyPitchMotorAccelerationScroller_valueChanged(int value);
    void on_bodyControlP1LoadDefaultsButton_clicked();
    void on_rightArmMotorDutyScroller_valueChanged(int value);
    void on_rightArmMotorDecelerationScroller_valueChanged(int value);
    void on_rightArmMotorAccelerationScroller_valueChanged(int value);
    void on_leftArmMotorDutyScroller_valueChanged(int value);
    void on_leftArmMotorDecelerationScroller_valueChanged(int value);
    void on_leftArmMotorAccelerationScroller_valueChanged(int value);
    void on_bodyPitchUpButton_pressed();
    void on_bodyPitchUpButton_released();
    void on_bodyPitchDownButton_pressed();
    void on_bodyPitchDownButton_released();
    void on_rightArmYawOpenButton_pressed();
    void on_rightArmYawOpenButton_released();
    void on_rightArmYawCloseButton_pressed();
    void on_rightArmYawCloseButton_released();
    void on_leftArmYawOpenButton_pressed();
    void on_leftArmYawOpenButton_released();
    void on_leftArmYawCloseButton_pressed();
    void on_leftArmYawCloseButton_released();
    void on_bodyControlP1ShiftRegEnableButton_clicked();
    void on_bodyControlP1ShiftRegDisableButton_clicked();
    void on_bodyControlP1ShiftRegResetButton_clicked();
    void on_bodyControlP1StopShiftRegResetButton_clicked();
    void on_powerSource5V5OnOffButton_clicked();
    void on_wifiOnOffButton_clicked();
    void on_leftArm24VOnOffButton_clicked();
    void on_rightArm24VButton_clicked();
    void on_headLedOnOffButton_clicked();
    void on_leftAccumulatorOnOffButton_clicked();
    void on_rightAccumulatorOnOffButton_clicked();
    void on_leftArm12VOnOffButton_clicked();
    void on_rightArm12VButton_clicked();
    void on_kinect1OnOffButton_clicked();
    void on_kinect2OnOffButton_clicked();
    void on_headYawTestModeExecuteButton_clicked();
    void on_headPitchTestModeExecuteButton_clicked();
    void on_getHeadYawPositionButton_clicked();
    void on_getHeadPitchPositionButton_clicked();
    void on_headYawStepDelayTestModeSpinBox_valueChanged(int value);
    void on_headYawMoveStepsSpinBox_valueChanged(int value);
    void on_headPitchStepDelayTestModeSpinBox_valueChanged(int value);
    void on_headPitchMoveStepsSpinBox_valueChanged(int value);
    void on_bodyControlP1ReadingsTable_itemClicked(QTableWidgetItem *item);
    void on_bodyCameraReleaseButton_clicked();
    void on_bodyCameraPositionScroller_valueChanged(int value);

    //arm-control-left
    void initArmControlLeft(Ui::MainWindow *ui);
    void armControlLeftTabRefreshTimerUpdate();
    void on_leftForearmMotorDutyScroller_valueChanged(int value);
    void on_leftForearmMotorDecelerationScroller_valueChanged(int value);
    void on_leftForearmAccelerationScroller_valueChanged(int value);
    void on_armControlLeftArmMotorDutyScroller_valueChanged(int value);
    void on_armControlLeftArmMotorDecelerationScroller_valueChanged(int value);
    void on_armControlLeftArmMotorAccelerationScroller_valueChanged(int value);
    void on_leftLimbMotorDutyScroller_valueChanged(int value);
    void on_leftLimbMotorDecelerationScroller_valueChanged(int value);
    void on_leftLimbMotorAccelerationScroller_valueChanged(int value);
    void on_leftForearmMoveUpButton_pressed();
    void on_leftForearmMoveUpButton_released();
    void on_leftForearmMoveDownButton_pressed();
    void on_leftForearmMoveDownButton_released();
    void on_leftArmMoveUpButton_pressed();
    void on_leftArmMoveUpButton_released();
    void on_leftArmMoveDownButton_pressed();
    void on_leftArmMoveDownButton_released();
    void on_leftLimbMoveUpButton_pressed();
    void on_leftLimbMoveUpButton_released();
    void on_leftLimbMoveDownButton_pressed();
    void on_leftLimbMoveDownButton_released();
    void on_leftHandYawCCWButton_pressed();
    void on_leftHandYawCCWButton_released();
    void on_leftHandYawCWButton_pressed();
    void on_leftHandYawCWButton_released();
    void on_leftHandPitchUpButton_pressed();
    void on_leftHandPitchUpButton_released();
    void on_leftHandPitchDownButton_pressed();
    void on_leftHandPitchDownButton_released();
    void on_leftForearmRollCCWButton_pressed();
    void on_leftForearmRollCCWButton_released();
    void on_leftForearmRollCWButton_pressed();
    void on_leftForearmRollCWButton_released();
    void on_leftForearmRollMotorOnOffButton_clicked();
    void on_leftForearmRollStepDelaySpinBox_valueChanged(int value);
    void on_leftForearmRollStepSwitchDelaySpinBox_valueChanged(int value);
    void on_leftForearmResetPositionButton_clicked();
    void on_leftArmLedsOnOffButton_clicked();
    void on_armControlLeftLoadDefaultsButton_clicked();
    void on_leftArmReadingsTable_itemClicked(QTableWidgetItem *item);
    void on_leftHandSensorsTable_itemClicked(QTableWidgetItem *item);
    void on_leftForearmYawCCWButton_pressed();
    void on_leftForearmYawCCWButton_released();
    void on_leftForearmYawCWButton_pressed();
    void on_leftForearmYawCWButton_released();
    void on_leftHandSensorsTrackAllButton_clicked();
    void on_leftHandSensorsTrackNoneButton_clicked();
    void on_leftArmReadingsTrackAllButton_clicked();
    void on_leftArmReadingsTrackNoneButton_clicked();
    void on_armControlRightStopAllWatchersButton_clicked();
    void on_armControlRightStartAllWatchersButton_clicked();
    void on_armControlLeftPalmReleaseButton_clicked();
    void on_armControlLeftPalmActivatedButton_clicked();
    void on_armControlLeftPalmGraspButton_clicked();
    void on_armControlLeftPalmSqueezeButton_clicked();
    void on_armControlLeftFinger6PositionScoller_valueChanged(int value);
    void on_armControlLeftFinger7PositionScoller_valueChanged(int value);
    void on_armControlLeftFinger8PositionScoller_valueChanged(int value);
    void on_armControlLeftFinger9PositionScoller_valueChanged(int value);
    void on_armControlLeftFinger10PositionScoller_valueChanged(int value);
    void on_armControlLeftFinger11PositionScoller_valueChanged(int value);
    void on_armControlLeftFinger6ReleaseButton_clicked();
    void on_armControlLeftFinger7ReleaseButton_clicked();
    void on_armControlLeftFinger8ReleaseButton_clicked();
    void on_armControlLeftFinger9ReleaseButton_clicked();
    void on_armControlLeftFinger10ReleaseButton_clicked();
    void on_armControlLeftFinger11ReleaseButton_clicked();

    //arm-control-right
    void initArmControlRight(Ui::MainWindow *ui);
    void armControlRightTabRefreshTimerUpdate();
    void on_rightForearmMoveUpButton_pressed();
    void on_rightForearmMoveUpButton_released();
    void on_rightForearmMoveDownButton_pressed();
    void on_rightForearmMoveDownButton_released();
    void on_rightForearmMotorDutyScroller_valueChanged(int value);
    void on_rightForearmMotorDecelerationScroller_valueChanged(int value);
    void on_rightForearmAccelerationScroller_valueChanged(int value);
    void on_armControlRightArmMotorDutyScroller_valueChanged(int value);
    void on_armControlRightArmMotorDecelerationScroller_valueChanged(int value);
    void on_armControlRightMotorAccelerationScroller_valueChanged(int value);
    void on_rightLimbMotorDutyScroller_valueChanged(int value);
    void on_rightLimbMotorDecelerationScroller_valueChanged(int value);
    void on_rightLimbMotorAccelerationScroller_valueChanged(int value);
    void on_rightArmMoveUpButton_pressed();
    void on_rightArmMoveUpButton_released();
    void on_rightArmMoveDownButton_pressed();
    void on_rightArmMoveDownButton_released();
    void on_rightLimbMoveUpButton_pressed();
    void on_rightLimbMoveUpButton_released();
    void on_rightLimbMoveDownButton_pressed();
    void on_rightLimbMoveDownButton_released();
    void on_rightHandYawCCWButton_pressed();
    void on_rightHandYawCCWButton_released();
    void on_rightHandYawCWButton_pressed();
    void on_rightHandYawCWButton_released();
    void on_rightHandPitchUpButton_pressed();
    void on_rightHandPitchUpButton_released();
    void on_rightHandPitchDownButton_pressed();
    void on_rightHandPitchDownButton_released();
    void on_rightForearmRollMotorOnOffButton_clicked();
    void on_rightForearmRollCCWButton_pressed();
    void on_rightForearmRollCCWButton_released();
    void on_rightForearmRollCWButton_pressed();
    void on_rightForearmRollCWButton_released();
    void on_rightForearmRollStepDelaySpinBox_valueChanged(int value);
    void on_rightForearmRollStepSwitchDelaySpinBox_valueChanged(int value);
    void on_rightForearmResetPositionButton_clicked();
    void on_rightArmLedsOnOffButton_clicked();
    void on_armControlRightLoadDefaultsButton_clicked();
    void on_rightArmReadingsTable_itemClicked(QTableWidgetItem *item);
    void on_rightHandSensorsTable_itemClicked(QTableWidgetItem *item);
    void on_rightForearmYawCCWButton_pressed();
    void on_rightForearmYawCCWButton_released();
    void on_rightForearmYawCWButton_pressed();
    void on_rightForearmYawCWButton_released();
    void on_rightHandSensorsTrackAllButton_clicked();
    void on_rightHandSensorsTrackNoneButton_clicked();
    void on_rightArmReadingsTrackAllButton_clicked();
    void on_rightArmReadingsTrackNoneButton_clicked();
    void on_armControlLeftStopAllWatchersButton_clicked();
    void on_armControlLeftStartAllWatchersButton_clicked();
    void on_armControlRightRedrawGUICheckBox_clicked(bool checked);
    void on_armControlRightFinger0ReleaseButton_clicked();
    void on_armControlRightFinger1ReleaseButton_clicked();
    void on_armControlRightFinger2ReleaseButton_clicked();
    void on_armControlRightFinger3ReleaseButton_clicked();
    void on_armControlRightFinger4ReleaseButton_clicked();
    void on_armControlRightFinger5ReleaseButton_clicked();
    void on_armControlRightPalmReleaseButton_clicked();
    void on_armControlRightPalmActivatedButton_clicked();
    void on_armControlRightPalmGraspButton_clicked();
    void on_armControlRightPalmSqueezeButton_clicked();
    void on_armControlRightFinger0PositionScoller_valueChanged(int value);
    void on_armControlRightFinger1PositionScoller_valueChanged(int value);
    void on_armControlRightFinger2PositionScoller_valueChanged(int value);
    void on_armControlRightFinger3PositionScoller_valueChanged(int value);
    void on_armControlRightFinger4PositionScoller_valueChanged(int value);
    void on_armControlRightFinger5PositionScoller_valueChanged(int value);

    //Tasks Tab
    void tasksTabRefreshTimerUpdate();

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
    QTimer *refreshControlDeviceTableTimer;
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

    //body-control-p1
    QTimer *bodyControlP1TabRefreshTimer;

    //arm-control-left
    QTimer *armControlLeftTabRefreshTimer;

    //arm-control-right
    QTimer *armControlRightTabRefreshTimer;

    //tasks tab
    QTimer *tasksTabRefreshTimer;

    // QWidget interface
protected:

private slots:

    //TCP Interface
    void on_updateCentralCommandHostConnectionInfoOnAllSlavesButton_clicked();
    void on_tcpInterfaceRemoteControlDevicesHostsUpdateSettingsButton_clicked();
    void on_armControlLeftRedrawGUICheckBox_clicked(bool checked);
    void on_tcpInterfaceRemoteControlDevicesHostsDisconnectAllButton_clicked();
    void on_tcpInterfaceRemoteControlCheckAllAsRemoteControlledButton_clicked();

    //Tasks
    void on_executeScriptButton_clicked();
    void on_clearTasksQueueButton_clicked();
    void on_forcefullyCompleteTaskButton_clicked();
    void on_clearRemoteTaskManagerTasksMapButton_clicked();
};

#endif // MAINWINDOW_H
