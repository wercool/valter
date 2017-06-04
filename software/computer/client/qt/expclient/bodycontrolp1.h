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
    void initTcpInterface();
    void initTcpCommandAcceptorInterface();
    void processControlDeviceResponse(string response);
    unsigned int executeTask(std::string taskScriptLine);

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
    bool setLeftArmYawMovementDirection(bool value);

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
    bool prepareLeftArmYawMovement();

    bool getBodyPitchPositionTrack() const;
    void setBodyPitchPositionTrack(bool value);

    bool getBodyPitchPositionADC() const;
    void setBodyPitchPositionADC(bool value);

    bool getBodyPitchCurrentTrack() const;
    void setBodyPitchCurrentTrack(bool value);

    bool getBodyPitchCurrentADC() const;
    void setBodyPitchCurrentADC(bool value);

    bool getRightArmYawPositionTrack() const;
    void setRightArmYawPositionTrack(bool value);

    bool getRightArmYawPositionADC() const;
    void setRightArmYawPositionADC(bool value);

    bool getRightArmYawCurrentTrack() const;
    void setRightArmYawCurrentTrack(bool value);

    bool getRightArmYawCurrentADC() const;
    void setRightArmYawCurrentADC(bool value);

    bool getLeftArmYawPositionTrack() const;
    void setLeftArmYawPositionTrack(bool value);

    bool getLeftArmYawPositionADC() const;
    void setLeftArmYawPositionADC(bool value);

    bool getLeftArmYawCurrentTrack() const;
    void setLeftArmYawCurrentTrack(bool value);

    bool getLeftArmYawCurrentADC() const;
    void setLeftArmYawCurrentADC(bool value);

    bool getHeadPitchPositionTrack() const;
    void setHeadPitchPositionTrack(bool value);

    bool getHeadPitchPositionADC() const;
    void setHeadPitchPositionADC(bool value);

    bool getHeadYawPositionTrack() const;
    void setHeadYawPositionTrack(bool value);

    bool getHeadYawPositionADC() const;
    void setHeadYawPositionADC(bool value);

    int getBodyPitchADCPosition() const;
    void setBodyPitchADCPosition(int value);

    int getRightArmYawADCPosition() const;
    void setRightArmYawADCPosition(int value);
    double getRightArmYawPosition() const;
    void setRightArmYawPosition(double value);

    int getLeftArmYawADCPosition() const;
    void setLeftArmYawADCPosition(int value);
    double getLeftArmYawPosition() const;
    void setLeftArmYawPosition(double value);


    int getHeadYawADCPosition() const;
    void setHeadYawADCPosition(int value);
    double getHeadYawPosition() const;
    void setHeadYawPosition(double value);
    int getHeadYawStepPostion() const;
    void setHeadYawStepPostion(int value);

    int getHeadPitchADCPosition() const;
    void setHeadPitchADCPosition(int value);
    double getHeadPitchPosition() const;
    void setHeadPitchPosition(double value);
    int getHeadPitchStepPosition() const;
    void setHeadPitchStepPosition(int value);

    int getBodyPitchADCCurrent() const;
    void setBodyPitchADCCurrent(int value);

    int getRightArmYawADCCurrent() const;
    void setRightArmYawADCCurrent(int value);

    int getLeftArmYawADCCurrent() const;
    void setLeftArmYawADCCurrent(int value);

    void shiftRegEnable();
    void shiftRegDisable();
    void shiftRegReset();
    void stopShiftRegReset();

    bool getPowerSource5V5State() const;
    void setPowerSource5V5State(bool value);
    void setPowerSource5VOnOff(bool value);

    bool getWifiPowerState() const;
    void setWifiPowerState(bool value);
    void setWifiPowerOnOff(bool value);

    bool getRightArm24VPowerSourceState() const;
    void setRightArm24VPowerSourceState(bool value);
    void setRightArm24VPowerOnOff(bool value);

    bool getLeftArm24VPowerSourceState() const;
    void setLeftArm24VPowerSourceState(bool value);
    void setLeftArm24VPowerOnOff(bool value);

    bool getHeadLedState() const;
    void setHeadLedState(bool value);
    void setHeadLedOnOff(bool value);

    bool getLeftAccumulatorConnectedState() const;
    void setLeftAccumulatorConnectedState(bool value);
    void setLeftAccumulatorOnOff(bool value);

    bool getRightAccumulatorConnectedState() const;
    void setRightAccumulatorConnectedState(bool value);
    void setRightAccumulatorOnOff(bool value);

    bool getRightArm12VPowerSourceState() const;
    void setRightArm12VPowerSourceState(bool value);
    void setRightArm12VPowerOnOff(bool value);

    bool getLeftArm12VPowerSourceState() const;
    void setLeftArm12VPowerSourceState(bool value);
    void setLeftArm12VPowerOnOff(bool value);

    bool getKinect1PowerState() const;
    void setKinect1PowerState(bool value);
    void setKinect1PowerOnOff(bool value);

    bool getKinect2PowerState() const;
    void setKinect2PowerState(bool value);
    void setKinect2PowerOnOff(bool value);

    void headYawMoveSteps(bool direction, int stepTime, int steps); //direction true - right, false - left
    void headPitchMoveSteps(bool direction, int stepTime, int steps);//direction true - down, false - up

    void requestHeadYawPosition();
    void requestHeadPitchPosition();

    void releaseBodyCamera();
    void setBodyCameraPosition(unsigned int position);

    int getBodyCameraLowerPosition() const;
    void setBodyCameraLowerPosition(int value);

    int getBodyCameraCenterPosition() const;
    void setBodyCameraCenterPosition(int value);

    int getBodyCameraUpperPosition() const;
    void setBodyCameraUpperPosition(int value);

    //body joints parameters
    static const int rightArmYawAngleADCMin;
    static const int rightArmYawAngleADCMax;
    static const int rightArmYawAngleADCZero;
    static const float rightArmYawMaxAngle;
    static const float rightArmYawDegreesDiv;

    static const int leftArmYawAngleADCMin;
    static const int leftArmYawAngleADCMax;
    static const int leftArmYawAngleADCZero;
    static const float leftArmYawMaxAngle;
    static const float leftArmYawDegreesDiv;

    static const int headYawAngleADCMin;
    static const int headYawAngleADCMax;
    static const int headYawAngleADCZero;
    static const float headYawMaxAngle;
    static const float headYawDegreesDiv;

    static const int headPitchAngleADCMin;
    static const int headPitchAngleADCMax;
    static const int headPitchAngleADCZero;
    static const float headPitchMaxAngle;
    static const float headPitchDegreesDiv;


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

    void sendCDRToCentralCommandHostWorker(string response);

    //------------------------------------------------------head yaw and pitch
    bool head24VState;
    bool headYawMotorState;
    bool headPitchMotorState;

    bool headYawDirection;  //true - turn right, false - turn left
    bool headYawMotorActivated;
    int headYawStepDelay; //us
    int headYawStepSwitchDelay; //us
    int headYawADCPosition;
    int headYawStepPostion;
    double headYawPosition;

    bool headPitchDirection;  //true - pitch down, false - pitch up
    bool headPitchMotorActivated;
    int headPitchStepDelay; //us
    int headPitchStepSwitchDelay; //us
    int headPitchADCPosition;
    int headPitchStepPosition;
    double headPitchPosition;

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

    int bodyPitchADCPosition;
    int bodyPitchADCCurrent;

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

    int rightArmYawADCPosition;
    int rightArmYawADCCurrent;

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

    int leftArmYawADCPosition;
    int leftArmYawADCCurrent;

    //presets
    int leftArmYawMotorDutyPresetCur;
    int leftArmYawMotorDutyPresetMin;
    int leftArmYawMotorDutyPresetMax;

    //body control p1 sensors reading settings
    bool bodyPitchPositionTrack;
    bool bodyPitchPositionADC;
    bool bodyPitchCurrentTrack;
    bool bodyPitchCurrentADC;
    bool rightArmYawPositionTrack;
    bool rightArmYawPositionADC;
    bool rightArmYawCurrentTrack;
    bool rightArmYawCurrentADC;
    bool leftArmYawPositionTrack;
    bool leftArmYawPositionADC;
    bool leftArmYawCurrentTrack;
    bool leftArmYawCurrentADC;
    bool headPitchPositionTrack;
    bool headPitchPositionADC;
    bool headYawPositionTrack;
    bool headYawPositionADC;

    //5.5V
    bool powerSource5V5State;
    //wifi
    bool wifiPowerState;

    //right arm roll 12V state
    bool rightArm12VPowerSourceState;

    //left arm roll 12V state
    bool leftArm12VPowerSourceState;

    //right arm roll 24V state
    bool rightArm24VPowerSourceState;

    //left arm roll 24V state
    bool leftArm24VPowerSourceState;

    //head led
    bool headLedState;

    //left accumulator connected state
    bool leftAccumulatorConnectedState;

    //right accumulator connected state
    bool rightAccumulatorConnectedState;

    bool kinect1PowerState;
    bool kinect2PowerState;

    int bodyCameraLowerPosition;
    int bodyCameraCenterPosition;
    int bodyCameraUpperPosition;

    double rightArmYawPosition;
    double leftArmYawPosition;

    /**************************************************** TASKS ******************************************************/
    std::map<std::string, function<ITask*(void)>> tasks;
};


#endif // BODYCONTROLP1_H
