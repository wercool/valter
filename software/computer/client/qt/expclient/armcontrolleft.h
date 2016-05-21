#ifndef ARMCONTROLLEFT_H
#define ARMCONTROLLEFT_H

#include "ivaltermodule.h"

using namespace std;

class ArmControlLeft: public IValterModule
{
public:
    static ArmControlLeft *getInstance();

    static string getControlDeviceId();

public:
    // IValterModule interface
    void stopAll();
    void resetToDefault();
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();
    void initTcpInterface();
    void initTcpCommandAcceptorInterface();
    void processControlDeviceResponse(string response);

    int getLeftForearmMotorDuty() const;
    void setLeftForearmMotorDuty(int value);

    int getLeftForearmMotorDutyMax() const;
    void setLeftForearmMotorDutyMax(int value);

    int getLeftForearmMotorAcceleration() const;
    void setLeftForearmMotorAcceleration(int value);

    int getLeftForearmMotorDeceleration() const;
    void setLeftForearmMotorDeceleration(int value);

    bool getLeftForearmMotorAccelerating() const;
    void setLeftForearmMotorAccelerating(bool value);

    bool getLeftForearmMotorDecelerating() const;
    void setLeftForearmMotorDecelerating(bool value);

    bool getLeftForearmMotorMovementDirection() const;
    bool setLeftForearmMotorMovementDirection(bool value);

    bool getLeftForearmMotorActivated() const;
    void setLeftForearmMotorActivated(bool value);

    bool getLeftForearmMotorStop() const;
    void setLeftForearmMotorStop(bool value);

    int getLeftForearmADCPosition() const;
    void setLeftForearmADCPosition(int value);

    int getLeftForearmADCCurrent() const;
    void setLeftForearmADCCurrent(int value);

    int getLeftForearmMotorDutyPresetCur() const;
    void setLeftForearmMotorDutyPresetCur(int value);

    int getLeftForearmMotorDutyPresetMin() const;
    void setLeftForearmMotorDutyPresetMin(int value);

    int getLeftForearmMotorDutyPresetMax() const;
    void setLeftForearmMotorDutyPresetMax(int value);

    int getLeftArmMotorDuty() const;
    void setLeftArmMotorDuty(int value);

    int getLeftArmMotorDutyMax() const;
    void setLeftArmMotorDutyMax(int value);

    int getLeftArmMotorAcceleration() const;
    void setLeftArmMotorAcceleration(int value);

    int getLeftArmMotorDeceleration() const;
    void setLeftArmMotorDeceleration(int value);

    bool getLeftArmMotorAccelerating() const;
    void setLeftArmMotorAccelerating(bool value);

    bool getLeftArmMotorDecelerating() const;
    void setLeftArmMotorDecelerating(bool value);

    bool getLeftArmMotorMovementDirection() const;
    bool setLeftArmMotorMovementDirection(bool value);

    bool getLeftArmMotorActivated() const;
    void setLeftArmMotorActivated(bool value);

    bool getLeftArmMotorStop() const;
    void setLeftArmMotorStop(bool value);

    int getLeftArmADCPosition() const;
    void setLeftArmADCPosition(int value);

    int getLeftArmADCCurrent() const;
    void setLeftArmADCCurrent(int value);

    int getLeftArmMotorDutyPresetCur() const;
    void setLeftArmMotorDutyPresetCur(int value);

    int getLeftArmMotorDutyPresetMin() const;
    void setLeftArmMotorDutyPresetMin(int value);

    int getLeftArmMotorDutyPresetMax() const;
    void setLeftArmMotorDutyPresetMax(int value);

    int getLeftLimbMotorDuty() const;
    void setLeftLimbMotorDuty(int value);

    int getLeftLimbMotorDutyMax() const;
    void setLeftLimbMotorDutyMax(int value);

    int getLeftLimbMotorAcceleration() const;
    void setLeftLimbMotorAcceleration(int value);

    int getLeftLimbMotorDeceleration() const;
    void setLeftLimbMotorDeceleration(int value);

    bool getLeftLimbMotorAccelerating() const;
    void setLeftLimbMotorAccelerating(bool value);

    bool getLeftLimbMotorDecelerating() const;
    void setLeftLimbMotorDecelerating(bool value);

    bool getLeftLimbMotorMovementDirection() const;
    bool setLeftLimbMotorMovementDirection(bool value);

    bool getLeftLimbMotorActivated() const;
    void setLeftLimbMotorActivated(bool value);

    bool getLeftLimbMotorStop() const;
    void setLeftLimbMotorStop(bool value);

    int getLeftLimbADCPosition() const;
    void setLeftLimbADCPosition(int value);

    int getLeftLimbADCCurrent() const;
    void setLeftLimbADCCurrent(int value);

    int getLeftLimbMotorDutyPresetCur() const;
    void setLeftLimbMotorDutyPresetCur(int value);

    int getLeftLimbMotorDutyPresetMin() const;
    void setLeftLimbMotorDutyPresetMin(int value);

    int getLeftLimbMotorDutyPresetMax() const;
    void setLeftLimbMotorDutyPresetMax(int value);

    bool prepareLeftForearmMovement();
    bool prepareLeftArmMovement();
    bool prepareLeftLimbMovement();

    //------------------------hand yaw
    bool getHandYawDirection() const;
    void setHandYawDirection(bool value);
    void handYaw(bool state);

    //------------------------hand pitch
    bool getHandPitchDirection() const;
    void setHandPitchDirection(bool value);
    void handPitch(bool state);

    //------------------------forearm roll
    bool getForearmRollDirection() const;
    void setForearmRollDirection(bool value);

    bool getForearmRollMotorState() const;
    void setForearmRollMotorState(bool value);
    void setForearmRollMotorOnOff(bool value);

    bool getForearmRollMotorActivated() const;
    void setForearmRollMotorActivated(bool value);

    int getForearmRollStepSwitchDelay() const;
    void setForearmRollStepSwitchDelay(int value);

    int getForearmRollStepDelay() const;
    void setForearmRollStepDelay(int value);

    int getForearmRollStepPosition() const;
    void setForearmRollStepPosition(int value);

    bool getForearmRollCWLimit() const;
    void setForearmRollCWLimit(bool value);

    bool getForearmRollCCWLimit() const;
    void setForearmRollCCWLimit(bool value);

    bool getForearmRollResettingStepPosition() const;
    void setForearmRollResettingStepPosition(bool value);

    void turnArmLedsOnOff();

    bool getArmLedsState() const;
    void setArmLedsState(bool value);

    //left arm readings presets

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

    //left arm readings

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

    //----------------forearm aw
    bool getForearmYawDirection() const;
    void setForearmYawDirection(bool value);
    void forearmYaw(bool state);

    void stopAllWatchers();
    void startAllWatchers();

    //fingers controllers
    void releaseFinger(unsigned int idx);
    void releaseAllFingers();
    void fingersToInitialPositions();
    void setFingerPosition(unsigned int idx, unsigned int position);
    void fingersGrasp();
    void fingersSqueeze();

    unsigned int getFingerInitialPosition(unsigned int idx);
    unsigned int getFingerGraspedPosition(unsigned int idx);

    bool getDefaultsLoading() const;
    void setDefaultsLoading(bool value);

    unsigned int getFinger6TipThreshold() const;
    void setFinger6TipThreshold(unsigned int value);

    unsigned int getFinger6PhalanxThreshold() const;
    void setFinger6PhalanxThreshold(unsigned int value);

    unsigned int getFinger7PhalanxThreshold() const;
    void setFinger7PhalanxThreshold(unsigned int value);

    unsigned int getFinger7TipThreshold() const;
    void setFinger7TipThreshold(unsigned int value);

    unsigned int getFinger8PhalanxThreshold() const;
    void setFinger8PhalanxThreshold(unsigned int value);

    unsigned int getFinger9PhalanxThreshold() const;
    void setFinger9PhalanxThreshold(unsigned int value);

    unsigned int getFinger10TipThreshold() const;
    void setFinger10TipThreshold(unsigned int value);

    unsigned int getFinger10PhalanxThreshold() const;
    void setFinger10PhalanxThreshold(unsigned int value);

    unsigned int getFinger11TipThreshold() const;
    void setFinger11TipThreshold(unsigned int value);

    unsigned int getFinger11PhalanxThreshold() const;
    void setFinger11PhalanxThreshold(unsigned int value);

    unsigned int getPalmUpperThreshold() const;
    void setPalmUpperThreshold(unsigned int value);

    unsigned int getPalmLowerThreshold() const;
    void setPalmLowerThreshold(unsigned int value);

    unsigned int getPalmJambThreshold() const;
    void setPalmJambThreshold(unsigned int value);

    unsigned int getFinger6TipReading() const;
    void setFinger6TipReading(unsigned int value);

    unsigned int getFinger6PhalanxReading() const;
    void setFinger6PhalanxReading(unsigned int value);

    unsigned int getFinger7PhalanxReading() const;
    void setFinger7PhalanxReading(unsigned int value);

    unsigned int getFinger7TipReading() const;
    void setFinger7TipReading(unsigned int value);

    unsigned int getFinger8PhalanxReading() const;
    void setFinger8PhalanxReading(unsigned int value);

    unsigned int getFinger9PhalanxReading() const;
    void setFinger9PhalanxReading(unsigned int value);

    unsigned int getFinger10TipReading() const;
    void setFinger10TipReading(unsigned int value);

    unsigned int getFinger10PhalanxReading() const;
    void setFinger10PhalanxReading(unsigned int value);

    unsigned int getFinger11TipReading() const;
    void setFinger11TipReading(unsigned int value);

    unsigned int getFinger11PhalanxReading() const;
    void setFinger11PhalanxReading(unsigned int value);

    unsigned int getPalmUpperReading() const;
    void setPalmUpperReading(unsigned int value);

    unsigned int getPalmLowerReading() const;
    void setPalmLowerReading(unsigned int value);

    unsigned int getPalmJambReading() const;
    void setPalmJambReading(unsigned int value);

    float getFinger6TipReadingRelative() const;
    void setFinger6TipReadingRelative(float value);

    float getFinger6PhalanxReadingRelative() const;
    void setFinger6PhalanxReadingRelative(float value);

    float getFinger7PhalanxReadingRelative() const;
    void setFinger7PhalanxReadingRelative(float value);

    float getFinger7TipReadingRelative() const;
    void setFinger7TipReadingRelative(float value);

    float getFinger8PhalanxReadingRelative() const;
    void setFinger8PhalanxReadingRelative(float value);

    float getFinger9PhalanxReadingRelative() const;
    void setFinger9PhalanxReadingRelative(float value);

    float getFinger10TipReadingRelative() const;
    void setFinger10TipReadingRelative(float value);

    float getFinger10PhalanxReadingRelative() const;
    void setFinger10PhalanxReadingRelative(float value);

    float getFinger11TipReadingRelative() const;
    void setFinger11TipReadingRelative(float value);

    float getFinger11PhalanxReadingRelative() const;
    void setFinger11PhalanxReadingRelative(float value);

    float getPalmUpperReadingRelative() const;
    void setPalmUpperReadingRelative(float value);

    float getPalmLowerReadingRelative() const;
    void setPalmLowerReadingRelative(float value);

    float getPalmJambReadingRelative() const;
    void setPalmJambReadingRelative(float value);

private:
    ArmControlLeft();
    static ArmControlLeft* pArmControlLeft;       // ARM-CONTROL-LEFT's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    bool defaultsLoading;

    void processMessagesQueueWorker();

    void leftForearmWorker();
    void leftArmWorker();
    void leftLimbWorker();
    void leftForearmRollWorker();
    void leftArmSensorReadingsWorker();

    //---------------left forearm
    int leftForearmMotorDuty;
    int leftForearmMotorDutyMax;
    int leftForearmMotorAcceleration;
    int leftForearmMotorDeceleration;

    bool leftForearmMotorAccelerating;
    bool leftForearmMotorDecelerating;

    bool leftForearmMotorMovementDirection;        //true - down, false - up
    bool leftForearmMotorActivated;
    bool leftForearmMotorStop;

    int leftForearmADCPosition;
    int leftForearmADCCurrent;

    //presets
    int leftForearmMotorDutyPresetCur;
    int leftForearmMotorDutyPresetMin;
    int leftForearmMotorDutyPresetMax;

    //---------------left arm
    int leftArmMotorDuty;
    int leftArmMotorDutyMax;
    int leftArmMotorAcceleration;
    int leftArmMotorDeceleration;

    bool leftArmMotorAccelerating;
    bool leftArmMotorDecelerating;

    bool leftArmMotorMovementDirection;        //true - down, false - up
    bool leftArmMotorActivated;
    bool leftArmMotorStop;

    int leftArmADCPosition;
    int leftArmADCCurrent;

    //presets
    int leftArmMotorDutyPresetCur;
    int leftArmMotorDutyPresetMin;
    int leftArmMotorDutyPresetMax;

    //---------------left limb
    int leftLimbMotorDuty;
    int leftLimbMotorDutyMax;
    int leftLimbMotorAcceleration;
    int leftLimbMotorDeceleration;

    bool leftLimbMotorAccelerating;
    bool leftLimbMotorDecelerating;

    bool leftLimbMotorMovementDirection;        //true - down, false - up
    bool leftLimbMotorActivated;
    bool leftLimbMotorStop;

    int leftLimbADCPosition;
    int leftLimbADCCurrent;

    //presets
    int leftLimbMotorDutyPresetCur;
    int leftLimbMotorDutyPresetMin;
    int leftLimbMotorDutyPresetMax;

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

    //left arm readings
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

    unsigned int fingerInitialPositions[6];
    unsigned int fingerGraspedPositions[6];

    //fingers thresholds (finger sensors)
    unsigned int finger6TipThreshold;
    unsigned int finger6PhalanxThreshold;
    unsigned int finger7PhalanxThreshold;
    unsigned int finger7TipThreshold;
    unsigned int finger8PhalanxThreshold;
    unsigned int finger9PhalanxThreshold;
    unsigned int finger10TipThreshold;
    unsigned int finger10PhalanxThreshold;
    unsigned int finger11TipThreshold;
    unsigned int finger11PhalanxThreshold;
    unsigned int palmUpperThreshold;
    unsigned int palmLowerThreshold;
    unsigned int palmJambThreshold;

    unsigned int finger6TipReading;
    unsigned int finger6PhalanxReading;
    unsigned int finger7PhalanxReading;
    unsigned int finger7TipReading;
    unsigned int finger8PhalanxReading;
    unsigned int finger9PhalanxReading;
    unsigned int finger10TipReading;
    unsigned int finger10PhalanxReading;
    unsigned int finger11TipReading;
    unsigned int finger11PhalanxReading;
    unsigned int palmUpperReading;
    unsigned int palmLowerReading;
    unsigned int palmJambReading;

    float finger6TipReadingRelative;
    float finger6PhalanxReadingRelative;
    float finger7PhalanxReadingRelative;
    float finger7TipReadingRelative;
    float finger8PhalanxReadingRelative;
    float finger9PhalanxReadingRelative;
    float finger10TipReadingRelative;
    float finger10PhalanxReadingRelative;
    float finger11TipReadingRelative;
    float finger11PhalanxReadingRelative;
    float palmUpperReadingRelative;
    float palmLowerReadingRelative;
    float palmJambReadingRelative;
};

#endif // ARMCONTROLLEFT_H
