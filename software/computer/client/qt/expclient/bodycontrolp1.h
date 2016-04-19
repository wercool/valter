#ifndef BODYCONTROLP1_H
#define BODYCONTROLP1_H

#include <string>

#include "ivaltermodule.h"

using namespace std;

class BodyControlP1: public IValterModule
{
public:
    static BodyControlP1 *getInstance();

    static string getControlDeviceId();

    void stopAll();
    void resetToDefault();
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();

    bool getHeadYawDirection() const;
    void setHeadYawDirection(bool value);

    bool getHeadYawMotorActivated() const;
    void setHeadYawMotorActivated(bool value);

    int getHeadYawStepDelay() const;
    void setHeadYawStepDelay(int value);

    int getHeadYawStepSwitchDelay() const;
    void setHeadYawStepSwitchDelay(int value);

    bool getHeadPitchDirection() const;
    void setHeadPitchDirection(bool value);

    bool getHeadPitchMotorActivated() const;
    void setHeadPitchMotorActivated(bool value);

    int getHeadPitchStepDelay() const;
    void setHeadPitchStepDelay(int value);

    int getHeadPitchStepSwitchDelay() const;
    void setHeadPitchStepSwitchDelay(int value);

    bool getHead24VState() const;
    void setHead24VState(bool value);
    void setHead24VOnOff(bool value);

    bool getHeadYawMotorState() const;
    void setHeadYawMotorState(bool value);
    void setHeadYawMotorOnOff(bool value);

    bool getHeadPitchMotorState() const;
    void setHeadPitchMotorState(bool value);
    void setHeadPitchMotorOnOff(bool value);

    int getBodyPitchMotorDuty() const;
    void setBodyPitchMotorDuty(int value);

    int getBodyPitchMotorDutyMax() const;
    void setBodyPitchMotorDutyMax(int value);

    int getBodyPitchMotorAcceleration() const;
    void setBodyPitchMotorAcceleration(int value);

    int getBodyPitchMotorDeceleration() const;
    void setBodyPitchMotorDeceleration(int value);

    bool getBodyPitchMotorAccelerating() const;
    void setBodyPitchMotorAccelerating(bool value);

    bool getBodyPitchMotorDecelerating() const;
    void setBodyPitchMotorDecelerating(bool value);

    bool getBodyPitchMovementDirection() const;
    bool setBodyPitchMovementDirection(bool value);

    bool getBodyPitchMotorActivated() const;
    void setBodyPitchMotorActivated(bool value);

    bool getBodyPitchMotorStop() const;
    void setBodyPitchMotorStop(bool value);

    int getBodyPitchMotorDutyPresetCur() const;
    void setBodyPitchMotorDutyPresetCur(int value);

    int getBodyPitchMotorDutyPresetMin() const;
    void setBodyPitchMotorDutyPresetMin(int value);

    int getBodyPitchMotorDutyPresetMax() const;
    void setBodyPitchMotorDutyPresetMax(int value);

    int getRightArmYawMotorDuty() const;
    void setRightArmYawMotorDuty(int value);

    int getRightArmYawMotorDutyMax() const;
    void setRightArmYawMotorDutyMax(int value);

    int getRightArmYawMotorAcceleration() const;
    void setRightArmYawMotorAcceleration(int value);

    int getRightArmYawMotorDeceleration() const;
    void setRightArmYawMotorDeceleration(int value);

    bool getRightArmYawMotorAccelerating() const;
    void setRightArmYawMotorAccelerating(bool value);

    bool getRightArmYawMotorDecelerating() const;
    void setRightArmYawMotorDecelerating(bool value);

    bool getRightArmYawMovementDirection() const;
    bool setRightArmYawMovementDirection(bool value);

    bool getRightArmYawMotorActivated() const;
    void setRightArmYawMotorActivated(bool value);

    bool getRightArmYawMotorStop() const;
    void setRightArmYawMotorStop(bool value);

    int getRightArmYawMotorDutyPresetCur() const;
    void setRightArmYawMotorDutyPresetCur(int value);

    int getRightArmYawMotorDutyPresetMin() const;
    void setRightArmYawMotorDutyPresetMin(int value);

    int getRightArmYawMotorDutyPresetMax() const;
    void setRightArmYawMotorDutyPresetMax(int value);

    int getLeftArmYawMotorDuty() const;
    void setLeftArmYawMotorDuty(int value);

    int getLeftArmYawMotorDutyMax() const;
    void setLeftArmYawMotorDutyMax(int value);

    int getLeftArmYawMotorAcceleration() const;
    void setLeftArmYawMotorAcceleration(int value);

    int getLeftArmYawMotorDeceleration() const;
    void setLeftArmYawMotorDeceleration(int value);

    bool getLeftArmYawMotorAccelerating() const;
    void setLeftArmYawMotorAccelerating(bool value);

    bool getLeftArmYawMotorDecelerating() const;
    void setLeftArmYawMotorDecelerating(bool value);

    bool getLeftArmYawMovementDirection() const;
    void setLeftArmYawMovementDirection(bool value);

    bool getLeftArmYawMotorActivated() const;
    void setLeftArmYawMotorActivated(bool value);

    bool getLeftArmYawMotorStop() const;
    void setLeftArmYawMotorStop(bool value);

    int getLeftArmYawMotorDutyPresetCur() const;
    void setLeftArmYawMotorDutyPresetCur(int value);

    int getLeftArmYawMotorDutyPresetMin() const;
    void setLeftArmYawMotorDutyPresetMin(int value);

    int getLeftArmYawMotorDutyPresetMax() const;
    void setLeftArmYawMotorDutyPresetMax(int value);

    bool prepareBodyPitchMovement();
    bool prepareRightArmYawMovement();

private:
    BodyControlP1();
    static BodyControlP1* pBodyControlP1;         // BODY-CONTROL-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();
    void headYawWorker();
    void headPitchWorker();

    void bodyPitchWorker();
    void rightArmYawWorker();
    void leftArmYawWorker();

    bool head24VState;
    bool headYawMotorState;
    bool headPitchMotorState;

    bool headYawDirection;  //true - turn right, false - turn left
    bool headYawMotorActivated;
    int headYawStepDelay; //us
    int headYawStepSwitchDelay; //us

    bool headPitchDirection;  //true - pitch down, false - pitch up
    bool headPitchMotorActivated;
    int headPitchStepDelay; //us
    int headPitchStepSwitchDelay; //us

    //------------------------------------------------body pitch
    int bodyPitchMotorDuty;
    int bodyPitchMotorDutyMax;
    int bodyPitchMotorAcceleration;
    int bodyPitchMotorDeceleration;

    bool bodyPitchMotorAccelerating;
    bool bodyPitchMotorDecelerating;

    bool bodyPitchMovementDirection;        //true - down, false - up
    bool bodyPitchMotorActivated;
    bool bodyPitchMotorStop;

    //presets
    int bodyPitchMotorDutyPresetCur;
    int bodyPitchMotorDutyPresetMin;
    int bodyPitchMotorDutyPresetMax;


    //------------------------------------------------right arm yaw
    int rightArmYawMotorDuty;
    int rightArmYawMotorDutyMax;
    int rightArmYawMotorAcceleration;
    int rightArmYawMotorDeceleration;

    bool rightArmYawMotorAccelerating;
    bool rightArmYawMotorDecelerating;

    bool rightArmYawMovementDirection;  //true - open, false - close
    bool rightArmYawMotorActivated;
    bool rightArmYawMotorStop;

    //presets
    int rightArmYawMotorDutyPresetCur;
    int rightArmYawMotorDutyPresetMin;
    int rightArmYawMotorDutyPresetMax;

    //------------------------------------------------left arm yaw
    int leftArmYawMotorDuty;
    int leftArmYawMotorDutyMax;
    int leftArmYawMotorAcceleration;
    int leftArmYawMotorDeceleration;

    bool leftArmYawMotorAccelerating;
    bool leftArmYawMotorDecelerating;

    bool leftArmYawMovementDirection;   //true - open, false - close
    bool leftArmYawMotorActivated;
    bool leftArmYawMotorStop;

    //presets
    int leftArmYawMotorDutyPresetCur;
    int leftArmYawMotorDutyPresetMin;
    int leftArmYawMotorDutyPresetMax;
};


#endif // BODYCONTROLP1_H
