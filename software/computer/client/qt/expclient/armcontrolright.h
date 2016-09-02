#ifndef ARMCONTROLRIGHT_H
#define ARMCONTROLRIGHT_H

#include "tasks/itask.h"
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
    void processControlDeviceResponse(string response);
    unsigned int executeTask(std::string taskScriptLine);

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
    float getForearmPosition() const;

    int getArmADCPosition() const;
    void setArmADCPosition(int value);
    float getArmPosition() const;

    int getLimbADCPosition() const;
    void setLimbADCPosition(int value);
    float getLimbPosition() const;

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

    unsigned int getFinger0TipThreshold() const;
    void setFinger0TipThreshold(unsigned int value);

    unsigned int getFinger0PhalanxThreshold() const;
    void setFinger0PhalanxThreshold(unsigned int value);

    unsigned int getFinger1PhalanxThreshold() const;
    void setFinger1PhalanxThreshold(unsigned int value);

    unsigned int getFinger1TipThreshold() const;
    void setFinger1TipThreshold(unsigned int value);

    unsigned int getFinger2PhalanxThreshold() const;
    void setFinger2PhalanxThreshold(unsigned int value);

    unsigned int getFinger3PhalanxThreshold() const;
    void setFinger3PhalanxThreshold(unsigned int value);

    unsigned int getFinger4TipThreshold() const;
    void setFinger4TipThreshold(unsigned int value);

    unsigned int getFinger4PhalanxThreshold() const;
    void setFinger4PhalanxThreshold(unsigned int value);

    unsigned int getFinger5PhalanxThreshold() const;
    void setFinger5PhalanxThreshold(unsigned int value);

    unsigned int getFinger5TipThreshold() const;
    void setFinger5TipThreshold(unsigned int value);

    unsigned int getPalmUpperThreshold() const;
    void setPalmUpperThreshold(unsigned int value);

    unsigned int getPalmLowerThreshold() const;
    void setPalmLowerThreshold(unsigned int value);

    unsigned int getPalmJambThreshold() const;
    void setPalmJambThreshold(unsigned int value);

    unsigned int getFinger0TipReading() const;
    void setFinger0TipReading(unsigned int value);

    unsigned int getFinger0PhalanxReading() const;
    void setFinger0PhalanxReading(unsigned int value);

    unsigned int getFinger1PhalanxReading() const;
    void setFinger1PhalanxReading(unsigned int value);

    unsigned int getFinger1TipReading() const;
    void setFinger1TipReading(unsigned int value);

    unsigned int getFinger2PhalanxReading() const;
    void setFinger2PhalanxReading(unsigned int value);

    unsigned int getFinger3PhalanxReading() const;
    void setFinger3PhalanxReading(unsigned int value);

    unsigned int getFinger4TipReading() const;
    void setFinger4TipReading(unsigned int value);

    unsigned int getFinger4PhalanxReading() const;
    void setFinger4PhalanxReading(unsigned int value);

    unsigned int getFinger5PhalanxReading() const;
    void setFinger5PhalanxReading(unsigned int value);

    unsigned int getFinger5TipReading() const;
    void setFinger5TipReading(unsigned int value);

    unsigned int getPalmUpperReading() const;
    void setPalmUpperReading(unsigned int value);

    unsigned int getPalmLowerReading() const;
    void setPalmLowerReading(unsigned int value);

    unsigned int getPalmJambReading() const;
    void setPalmJambReading(unsigned int value);

    float getFinger0TipReadingRelative();
    void setFinger0TipReadingRelative(float value);

    float getFinger0PhalanxReadingRelative();
    void setFinger0PhalanxReadingRelative(float value);

    float getFinger1PhalanxReadingRelative();
    void setFinger1PhalanxReadingRelative(float value);

    float getFinger1TipReadingRelative();
    void setFinger1TipReadingRelative(float value);

    float getFinger2PhalanxReadingRelative();
    void setFinger2PhalanxReadingRelative(float value);

    float getFinger3PhalanxReadingRelative();
    void setFinger3PhalanxReadingRelative(float value);

    float getFinger4TipReadingRelative();
    void setFinger4TipReadingRelative(float value);

    float getFinger4PhalanxReadingRelative();
    void setFinger4PhalanxReadingRelative(float value);

    float getFinger5PhalanxReadingRelative();
    void setFinger5PhalanxReadingRelative(float value);

    float getFinger5TipReadingRelative();
    void setFinger5TipReadingRelative(float value);

    float getPalmUpperReadingRelative();
    void setPalmUpperReadingRelative(float value);

    float getPalmLowerReadingRelative();
    void setPalmLowerReadingRelative(float value);

    float getPalmJambReadingRelative();
    void setPalmJambReadingRelative(float value);

    //right arm parameters
    static const int forearmAngleADCMin;
    static const int forearmAngleADCMax;
    static const int forearmAngleADCZero;
    static const float forearmMaxAngle;

    static const int armAngleADCMin;
    static const int armAngleADCMax;
    static const int armAngleADCZero;
    static const float armMaxAngle;

    static const int limbAngleADCMin;
    static const int limbAngleADCMax;
    static const int limbAngleADCZero;
    static const float limbMaxAngle;

private:
    ArmControlRight();
    static ArmControlRight* pArmControlRight;       // ARM-CONTROL-RIGHT's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    bool defaultsLoading;

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

    unsigned int fingerInitialPositions[6];
    unsigned int fingerGraspedPositions[6];

    //fingers thresholds (finger sensors)
    unsigned int finger0TipThreshold;
    unsigned int finger0PhalanxThreshold;
    unsigned int finger1PhalanxThreshold;
    unsigned int finger1TipThreshold;
    unsigned int finger2PhalanxThreshold;
    unsigned int finger3PhalanxThreshold;
    unsigned int finger4TipThreshold;
    unsigned int finger4PhalanxThreshold;
    unsigned int finger5PhalanxThreshold;
    unsigned int finger5TipThreshold;
    unsigned int palmUpperThreshold;
    unsigned int palmLowerThreshold;
    unsigned int palmJambThreshold;

    unsigned int finger0TipReading;
    unsigned int finger0PhalanxReading;
    unsigned int finger1PhalanxReading;
    unsigned int finger1TipReading;
    unsigned int finger2PhalanxReading;
    unsigned int finger3PhalanxReading;
    unsigned int finger4TipReading;
    unsigned int finger4PhalanxReading;
    unsigned int finger5PhalanxReading;
    unsigned int finger5TipReading;
    unsigned int palmUpperReading;
    unsigned int palmLowerReading;
    unsigned int palmJambReading;

    float finger0TipReadingRelative;
    float finger0PhalanxReadingRelative;
    float finger1PhalanxReadingRelative;
    float finger1TipReadingRelative;
    float finger2PhalanxReadingRelative;
    float finger3PhalanxReadingRelative;
    float finger4TipReadingRelative;
    float finger4PhalanxReadingRelative;
    float finger5PhalanxReadingRelative;
    float finger5TipReadingRelative;
    float palmUpperReadingRelative;
    float palmLowerReadingRelative;
    float palmJambReadingRelative;

    /**************************************************** TASKS ******************************************************/
    std::map<std::string, function<ITask*(void)>> tasks;
};

#endif // ARMCONTROLRIGHT_H
