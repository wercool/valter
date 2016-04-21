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

private:
    ArmControlLeft();
    static ArmControlLeft* pArmControlLeft;       // ARM-CONTROL-LEFT's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

    void leftForearmWorker();
    void leftArmWorker();
    void leftLimbWorker();

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
};

#endif // ARMCONTROLLEFT_H
