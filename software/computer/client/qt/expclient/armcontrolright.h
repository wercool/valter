#ifndef ARMCONTROLRIGHT_H
#define ARMCONTROLRIGHT_H

#include "ivaltermodule.h"

using namespace std;

class ArmControlRight: public IValterModule
{
public:
    static ArmControlRight *getInstance();

    static string getControlDeviceId();

    // IValterModule interface
public:
    void stopAll();
    void resetToDefault();
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();
    void initTcpInterface();
    void initTcpCommandAcceptorInterface();

    int getRightForearmMotorDuty() const;
    void setRightForearmMotorDuty(int value);

    int getRightForearmMotorDutyMax() const;
    void setRightForearmMotorDutyMax(int value);

    int getRightForearmMotorAcceleration() const;
    void setRightForearmMotorAcceleration(int value);

    int getRightForearmMotorDeceleration() const;
    void setRightForearmMotorDeceleration(int value);

    bool getRightForearmMotorAccelerating() const;
    void setRightForearmMotorAccelerating(bool value);

    bool getRightForearmMotorDecelerating() const;
    void setRightForearmMotorDecelerating(bool value);

    bool getRightForearmMotorMovementDirection() const;
    bool setRightForearmMotorMovementDirection(bool value);

    bool getRightForearmMotorActivated() const;
    void setRightForearmMotorActivated(bool value);

    bool getRightForearmMotorStop() const;
    void setRightForearmMotorStop(bool value);

    int getRightForearmADCPosition() const;
    void setRightForearmADCPosition(int value);

    int getRightForearmADCCurrent() const;
    void setRightForearmADCCurrent(int value);

    int getRightForearmMotorDutyPresetCur() const;
    void setRightForearmMotorDutyPresetCur(int value);

    int getRightForearmMotorDutyPresetMin() const;
    void setRightForearmMotorDutyPresetMin(int value);

    int getRightForearmMotorDutyPresetMax() const;
    void setRightForearmMotorDutyPresetMax(int value);

    int getRightArmMotorDuty() const;
    void setRightArmMotorDuty(int value);

    int getRightArmMotorDutyMax() const;
    void setRightArmMotorDutyMax(int value);

    int getRightArmMotorAcceleration() const;
    void setRightArmMotorAcceleration(int value);

    int getRightArmMotorDeceleration() const;
    void setRightArmMotorDeceleration(int value);

    bool getRightArmMotorAccelerating() const;
    void setRightArmMotorAccelerating(bool value);

    bool getRightArmMotorDecelerating() const;
    void setRightArmMotorDecelerating(bool value);

    bool getRightArmMotorMovementDirection() const;
    bool setRightArmMotorMovementDirection(bool value);

    bool getRightArmMotorActivated() const;
    void setRightArmMotorActivated(bool value);

    bool getRightArmMotorStop() const;
    void setRightArmMotorStop(bool value);

    int getRightArmADCPosition() const;
    void setRightArmADCPosition(int value);

    int getRightArmADCCurrent() const;
    void setRightArmADCCurrent(int value);

    int getRightArmMotorDutyPresetCur() const;
    void setRightArmMotorDutyPresetCur(int value);

    int getRightArmMotorDutyPresetMin() const;
    void setRightArmMotorDutyPresetMin(int value);

    int getRightArmMotorDutyPresetMax() const;
    void setRightArmMotorDutyPresetMax(int value);

    bool getHandYawDirection() const;
    void setHandYawDirection(bool value);

    bool getHandPitchDirection() const;
    void setHandPitchDirection(bool value);

    bool getForearmRollDirection() const;
    void setForearmRollDirection(bool value);

    bool getForearmRollMotorState() const;
    void setForearmRollMotorState(bool value);

    bool getForearmRollMotorActivated() const;
    void setForearmRollMotorActivated(bool value);

    int getForearmRollStepDelay() const;
    void setForearmRollStepDelay(int value);

    int getForearmRollStepSwitchDelay() const;
    void setForearmRollStepSwitchDelay(int value);

    int getForearmRollStepPosition() const;
    void setForearmRollStepPosition(int value);

    bool getForearmRollCWLimit() const;
    void setForearmRollCWLimit(bool value);

    bool getForearmRollCCWLimit() const;
    void setForearmRollCCWLimit(bool value);

    bool getForearmRollResettingStepPosition() const;
    void setForearmRollResettingStepPosition(bool value);

    bool getForearmYawDirection() const;
    void setForearmYawDirection(bool value);

    bool getForearmPositionTrack() const;
    void setForearmPositionTrack(bool value);

    bool getForearmPositionADC() const;
    void setForearmPositionADC(bool value);

    bool getArmPositionTrack() const;
    void setArmPositionTrack(bool value);

    bool getArmPositionADC() const;
    void setArmPositionADC(bool value);

    bool getLimbPositionTrack() const;
    void setLimbPositionTrack(bool value);

    bool getLimbPositionADC() const;
    void setLimbPositionADC(bool value);

    bool getForearmMotorCurrentTrack() const;
    void setForearmMotorCurrentTrack(bool value);

    bool getForearmMotorCurrentADC() const;
    void setForearmMotorCurrentADC(bool value);

    bool getArmMotorCurrentTrack() const;
    void setArmMotorCurrentTrack(bool value);

    bool getArmMotorCurrentADC() const;
    void setArmMotorCurrentADC(bool value);

    bool getLimbMotorCurrentTrack() const;
    void setLimbMotorCurrentTrack(bool value);

    bool getLimbMotorCurrentADC() const;
    void setLimbMotorCurrentADC(bool value);

    bool getHandYawPositionTrack() const;
    void setHandYawPositionTrack(bool value);

    bool getHandYawPositionADC() const;
    void setHandYawPositionADC(bool value);

    bool getHandPitchPositionTrack() const;
    void setHandPitchPositionTrack(bool value);

    bool getHandPitchPositionADC() const;
    void setHandPitchPositionADC(bool value);

    bool getForearmYawPositionTrack() const;
    void setForearmYawPositionTrack(bool value);

    bool getForearmYawPositionADC() const;
    void setForearmYawPositionADC(bool value);

    bool getForearmYawMotorCurrentTrack() const;
    void setForearmYawMotorCurrentTrack(bool value);

    bool getForearmYawMotorCurrentADC() const;
    void setForearmYawMotorCurrentADC(bool value);

    bool getHandYawMotorCurrentTrack() const;
    void setHandYawMotorCurrentTrack(bool value);

    bool getHandYawMotorCurrentADC() const;
    void setHandYawMotorCurrentADC(bool value);

    bool getHandPitchMotorCurrentTrack() const;
    void setHandPitchMotorCurrentTrack(bool value);

    bool getHandPitchMotorCurrentADC() const;
    void setHandPitchMotorCurrentADC(bool value);

    int getForearmADCPosition() const;
    void setForearmADCPosition(int value);

    int getArmADCPosition() const;
    void setArmADCPosition(int value);

    int getLimbADCPosition() const;
    void setLimbADCPosition(int value);

    int getForearmMotorADCCurrent() const;
    void setForearmMotorADCCurrent(int value);

    int getArmMotorADCCurrent() const;
    void setArmMotorADCCurrent(int value);

    int getLimbMotorADCCurrent() const;
    void setLimbMotorADCCurrent(int value);

    int getHandYawADCPosition() const;
    void setHandYawADCPosition(int value);

    int getHandPitchADCPosition() const;
    void setHandPitchADCPosition(int value);

    int getForearmYawADCPosition() const;
    void setForearmYawADCPosition(int value);

    int getForearmYawMotorADCCurrent() const;
    void setForearmYawMotorADCCurrent(int value);

    int getHandYawMotorADCCurrent() const;
    void setHandYawMotorADCCurrent(int value);

    int getHandPitchMotorADCCurrent() const;
    void setHandPitchMotorADCCurrent(int value);

    //hand finger sesnors preset
    void setHandSensorsTrack(int idx, bool state);
    bool getHandSensorsTrack(int idx);
    void setHandSensorsADCForce(int idx, int value);
    int getHandSensorsADCForce(int idx);

    bool getArmLedsState() const;
    void setArmLedsState(bool value);

    int getRightLimbMotorDuty() const;
    void setRightLimbMotorDuty(int value);

    int getRightLimbMotorDutyMax() const;
    void setRightLimbMotorDutyMax(int value);

    int getRightLimbMotorAcceleration() const;
    void setRightLimbMotorAcceleration(int value);

    int getRightLimbMotorDeceleration() const;
    void setRightLimbMotorDeceleration(int value);

    bool getRightLimbMotorAccelerating() const;
    void setRightLimbMotorAccelerating(bool value);

    bool getRightLimbMotorDecelerating() const;
    void setRightLimbMotorDecelerating(bool value);

    bool getRightLimbMotorMovementDirection() const;
    bool setRightLimbMotorMovementDirection(bool value);

    bool getRightLimbMotorActivated() const;
    void setRightLimbMotorActivated(bool value);

    bool getRightLimbMotorStop() const;
    void setRightLimbMotorStop(bool value);

    int getRightLimbADCPosition() const;
    void setRightLimbADCPosition(int value);

    int getRightLimbADCCurrent() const;
    void setRightLimbADCCurrent(int value);

    int getRightLimbMotorDutyPresetCur() const;
    void setRightLimbMotorDutyPresetCur(int value);

    int getRightLimbMotorDutyPresetMin() const;
    void setRightLimbMotorDutyPresetMin(int value);

    int getRightLimbMotorDutyPresetMax() const;
    void setRightLimbMotorDutyPresetMax(int value);

    bool prepareRightForearmMovement();
    bool prepareRightArmMovement();
    bool prepareRightLimbMovement();

    void handYaw(bool state);
    void handPitch(bool state);

    void setForearmRollMotorOnOff(bool state);

    void forearmYaw(bool state);

    void turnArmLedsOnOff();

private:
    ArmControlRight();
    static ArmControlRight* pArmControlRight;       // ARM-CONTROL-RIGHT's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

    void rightForearmWorker();
    void rightArmWorker();
    void rightLimbWorker();
    void rightForearmRollWorker();
    void rightArmSensorReadingsWorker();

    //---------------right forearm
    int rightForearmMotorDuty;
    int rightForearmMotorDutyMax;
    int rightForearmMotorAcceleration;
    int rightForearmMotorDeceleration;

    bool rightForearmMotorAccelerating;
    bool rightForearmMotorDecelerating;

    bool rightForearmMotorMovementDirection;        //true - down, false - up
    bool rightForearmMotorActivated;
    bool rightForearmMotorStop;

    int rightForearmADCPosition;
    int rightForearmADCCurrent;

    //presets
    int rightForearmMotorDutyPresetCur;
    int rightForearmMotorDutyPresetMin;
    int rightForearmMotorDutyPresetMax;

    //---------------right arm
    int rightArmMotorDuty;
    int rightArmMotorDutyMax;
    int rightArmMotorAcceleration;
    int rightArmMotorDeceleration;

    bool rightArmMotorAccelerating;
    bool rightArmMotorDecelerating;

    bool rightArmMotorMovementDirection;        //true - down, false - up
    bool rightArmMotorActivated;
    bool rightArmMotorStop;

    int rightArmADCPosition;
    int rightArmADCCurrent;

    //presets
    int rightArmMotorDutyPresetCur;
    int rightArmMotorDutyPresetMin;
    int rightArmMotorDutyPresetMax;

    //---------------right limb
    int rightLimbMotorDuty;
    int rightLimbMotorDutyMax;
    int rightLimbMotorAcceleration;
    int rightLimbMotorDeceleration;

    bool rightLimbMotorAccelerating;
    bool rightLimbMotorDecelerating;

    bool rightLimbMotorMovementDirection;        //true - down, false - up
    bool rightLimbMotorActivated;
    bool rightLimbMotorStop;

    int rightLimbADCPosition;
    int rightLimbADCCurrent;

    //presets
    int rightLimbMotorDutyPresetCur;
    int rightLimbMotorDutyPresetMin;
    int rightLimbMotorDutyPresetMax;

    //------------------------hand yaw
    bool handYawDirection;  //true - CW, false - CCW

    //------------------------hand pitch
    bool handPitchDirection;  //true - up, false - down

    //------------------------forearm roll
    bool forearmRollDirection;  //true - CCW, false - CW
    bool forearmRollMotorState;  //true - on, false - off
    bool forearmRollMotorActivated;
    int forearmRollStepDelay;
    int forearmRollStepSwitchDelay;
    int forearmRollStepPosition;
    bool forearmRollCWLimit;
    bool forearmRollCCWLimit;
    bool forearmRollResettingStepPosition; //CCW limit - 0 position

    //---------------------forearm yaw
    bool forearmYawDirection; //true - CW, false - CCW

    bool armLedsState;

    //left arm readings presets
    bool forearmPositionTrack;
    bool forearmPositionADC;
    bool armPositionTrack;
    bool armPositionADC;
    bool limbPositionTrack;
    bool limbPositionADC;
    bool forearmMotorCurrentTrack;
    bool forearmMotorCurrentADC;
    bool armMotorCurrentTrack;
    bool armMotorCurrentADC;
    bool limbMotorCurrentTrack;
    bool limbMotorCurrentADC;
    bool handYawPositionTrack;
    bool handYawPositionADC;
    bool handPitchPositionTrack;
    bool handPitchPositionADC;
    bool forearmYawPositionTrack;
    bool forearmYawPositionADC;
    bool forearmYawMotorCurrentTrack;
    bool forearmYawMotorCurrentADC;
    bool handYawMotorCurrentTrack;
    bool handYawMotorCurrentADC;
    bool handPitchMotorCurrentTrack;
    bool handPitchMotorCurrentADC;

    //right arm readings
    int forearmADCPosition;
    int armADCPosition;
    int limbADCPosition;
    int forearmMotorADCCurrent;
    int armMotorADCCurrent;
    int limbMotorADCCurrent;
    int handYawADCPosition;
    int handPitchADCPosition;
    int forearmYawADCPosition;
    int forearmYawMotorADCCurrent;
    int handYawMotorADCCurrent;
    int handPitchMotorADCCurrent;

    //hand finger sesnors preset
    bool handSensorsTrack[13];
    int handSensorsADCForce[13];
};

#endif // ARMCONTROLRIGHT_H
