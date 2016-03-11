#ifndef PLATFORMCONTROLP1_H
#define PLATFORMCONTROLP1_H

#include <string>

#include "ivaltermodule.h"

using namespace std;

class PlatformControlP1: public IValterModule
{
public:
    static PlatformControlP1 *getInstance();

    static string getControlDeviceId();

    void stopAll();
    void resetToDefault();
    void spawnProcessMessagesQueueWorkerThread();
    void loadDefaults();

    void resetValuesToDefault();

    bool preparePlatformMovement();

    bool getPower5VOnState() const;
    void setPower5VOnState(bool value);

    bool getMainAccumulatorRelayOnState() const;
    void setMainAccumulatorRelayOnState(bool value);

    bool getLeftAccumulatorRelayOnState() const;
    void setLeftAccumulatorRelayOnState(bool value);

    bool getRightAccumulatorRelayOnState() const;
    void setRightAccumulatorRelayOnState(bool value);

    int getLeftMotorDuty() const;
    void setLeftMotorDuty(int value);

    int getRightMotorDuty() const;
    void setRightMotorDuty(int value);

    bool getLeftAccumulatorConnected() const;
    void setLeftAccumulatorConnected(bool value);

    bool getRightAccumulatorConnected() const;
    void setRightAccumulatorConnected(bool value);

    int getLeftMotorDutyMax() const;
    void setLeftMotorDutyMax(int value);

    int getRightMotorDutyMax() const;
    void setRightMotorDutyMax(int value);

    bool getPower220VACAvailable() const;
    void setPower220VACAvailable(bool value);

    bool getCharger35Ah() const;
    void setCharger35Ah(bool value);

    bool getCharger120Ah() const;
    void setCharger120Ah(bool value);

    bool getChargingInProgress() const;
    void setChargingInProgress(bool value);

    bool getChargingComplete() const;
    void setChargingComplete(bool value);

    int getCurChannel1Input() const;
    void setCurChannel1Input(int value);

    bool getScan220ACAvailable() const;
    void setScan220ACAvailable(bool value);

    int getChargerVoltageADC() const;
    void setChargerVoltageADC(int value);

    float getChargerVoltageVolts() const;
    void setChargerVoltageVolts(float value);

    int getCurChannel2Input() const;
    void setCurChannel2Input(int value);

    bool getChargerConnected() const;
    void setChargerConnected(bool value);

    void chargerButtonPress();

    bool mainAccumulatorON();

    int getChargerButtonPressStep() const;
    void setChargerButtonPressStep(int value);

    bool getChargerMode() const;
    void setChargerMode(bool value);

    bool getLeftMotorDirection() const;
    bool setLeftMotorDirection(bool value);

    bool getRightMotorDirection() const;
    bool setRightMotorDirection(bool value);

    bool getLeftMotorStop() const;
    void setLeftMotorStop(bool value);

    bool getRightMotorStop() const;
    void setRightMotorStop(bool value);

    bool getPlatformEmergencyStop() const;
    void setPlatformEmergencyStop(bool value);

    bool getLeftMotorActivated() const;
    void setLeftMotorActivated(bool value);

    bool getRightMotorActivated() const;
    void setRightMotorActivated(bool value);

    int getPlatformDeceleration() const;
    void setPlatformDeceleration(int value);

    int getPlatformAcceleration() const;
    void setPlatformAcceleration(int value);

    bool getLeftMotorAccelerating() const;
    void setLeftMotorAccelerating(bool value);

    bool getRightMotorAccelerating() const;
    void setRightMotorAccelerating(bool value);

    bool getLeftMotorDecelerating() const;
    void setLeftMotorDecelerating(bool value);

    bool getRightMotorDecelerating() const;
    void setRightMotorDecelerating(bool value);

    int getTurretMotorDutyMax() const;
    void setTurretMotorDutyMax(int value);

    int getTurretMotorDuty() const;
    void setTurretMotorDuty(int value);

    bool getTurretMotorStop() const;
    void setTurretMotorStop(bool value);

    bool getTurretMotorActivated() const;
    void setTurretMotorActivated(bool value);

    int getTurretDeceleration() const;
    void setTurretDeceleration(int value);

    int getTurretAcceleration() const;
    void setTurretAcceleration(int value);

    bool getTurretMotorAccelerating() const;
    void setTurretMotorAccelerating(bool value);

    bool getTurretMotorDecelerating() const;
    void setTurretMotorDecelerating(bool value);

    bool getTurretEmergencyStop() const;
    void setTurretEmergencyStop(bool value);

    bool getTurretMotorDirection() const;
    bool setTurretMotorDirection(bool value);

    int getLeftMotorDutyPresetMin() const;
    void setLeftMotorDutyPresetMin(int value);

    int getLeftMotorDutyPresetMax() const;
    void setLeftMotorDutyPresetMax(int value);

    int getLeftMotorDutyPresetCur() const;
    void setLeftMotorDutyPresetCur(int value);

    int getRightMotorDutyPresetMin() const;
    void setRightMotorDutyPresetMin(int value);

    int getRightMotorDutyPresetMax() const;
    void setRightMotorDutyPresetMax(int value);

    int getRightMotorDutyPresetCur() const;
    void setRightMotorDutyPresetCur(int value);

    int getPlatformDecelerationPresetMin() const;
    void setPlatformDecelerationPresetMin(int value);

    int getPlatformDecelerationPresetMax() const;
    void setPlatformDecelerationPresetMax(int value);

    int getPlatformDecelerationPresetCur() const;
    void setPlatformDecelerationPresetCur(int value);

    int getPlatformAccelerationPresetMin() const;
    void setPlatformAccelerationPresetMin(int value);

    int getPlatformAccelerationPresetMax() const;
    void setPlatformAccelerationPresetMax(int value);

    int getPlatformAccelerationPresetCur() const;
    void setPlatformAccelerationPresetCur(int value);

    int getTurretMotorDutyPresetMin() const;
    void setTurretMotorDutyPresetMin(int value);

    int getTurretMotorDutyPresetMax() const;
    void setTurretMotorDutyPresetMax(int value);

    int getTurretMotorDutyPresetCur() const;
    void setTurretMotorDutyPresetCur(int value);

    int getTurretDecelerationPresetMin() const;
    void setTurretDecelerationPresetMin(int value);

    int getTurretDecelerationPresetMax() const;
    void setTurretDecelerationPresetMax(int value);

    int getTurretDecelerationPresetCur() const;
    void setTurretDecelerationPresetCur(int value);

    int getTurretAccelerationPresetMin() const;
    void setTurretAccelerationPresetMin(int value);

    int getTurretAccelerationPresetMax() const;
    void setTurretAccelerationPresetMax(int value);

    int getTurretAccelerationPresetCur() const;
    void setTurretAccelerationPresetCur(int value);

    int getMotorsPWMFrequncy() const;
    void setMotorsPWMFrequncy(int value);

    bool getWheelMotorsAmperes() const;
    void setWheelMotorsAmperes(bool value);

    int getTurretMotorCurrent() const;
    void setTurretMotorCurrent(int value);

    bool getTurretMotorsAmperes() const;
    void setTurretMotorsAmperes(bool value);

    int getLeftMotorCurrentADC() const;
    void setLeftMotorCurrentADC(int value);

    int getTurretMotorCurrentADC() const;
    void setTurretMotorCurrentADC(int value);

    int getRightMotorCurrentADC() const;
    void setRightMotorCurrentADC(int value);

    float getLeftMotorCurrentAmps() const;
    void setLeftMotorCurrentAmps(float value);

    float getRightMotorCurrentAmps() const;
    void setRightMotorCurrentAmps(float value);

    float getTurretMotorCurrentAmps() const;
    void setTurretMotorCurrentAmps(float value);

    bool getLeftMotorCurrentRead() const;
    void setLeftMotorCurrentRead(bool value);

    bool getRightMotorCurrentRead() const;
    void setRightMotorCurrentRead(bool value);

    bool getTurretMotorCurrentRead() const;
    void setTurretMotorCurrentRead(bool value);

    int getLeftWheelEncoder() const;
    void setLeftWheelEncoder(int value);

    int getRightWheelEncoder() const;
    void setRightWheelEncoder(int value);

    bool getLeftWheelEncoderRead() const;
    void setLeftWheelEncoderRead(bool value);

    bool getRightWheelEncoderRead() const;
    void setRightWheelEncoderRead(bool value);

    bool getLeftWheelEncoderAutoreset() const;
    void setLeftWheelEncoderAutoreset(bool value);

    bool getRightWheelEncoderAutoreset() const;
    void setRightWheelEncoderAutoreset(bool value);

    void resetLeftWheelEncoder();
    void resetRightWheelEncoder();

private:
    PlatformControlP1();
    static PlatformControlP1* pPlatformControlP1;       // PLATFORM-CONTROL-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

    //PlatforomControlP1 strictly specific methods
    void scanFor220VACAvailable();
    bool scan220ACAvailable;
    bool chargerMode; //[false] - turning off; [true] - set charging
    int chargerButtonPressStep;
    void chargerModeSetting();

    //motors
    //platform
    void platformMovementWorker();
    void platformMovementDynamics();
    bool platformEmergencyStop;
    bool leftMotorDirection; //[true] - forward; [false] - backward
    bool rightMotorDirection; //[true] - forward; [false] - backward
    //turret
    void turretRotationWorker();
    void turretRotationDynamics();
    bool turretEmergencyStop;
    bool turretMotorDirection; //[true] - right; [false] - left

    //PlatforomControlP1 strictly specific properties
    bool power5VOnState;
    bool leftAccumulatorConnected;
    bool rightAccumulatorConnected;
    bool mainAccumulatorRelayOnState;
    bool leftAccumulatorRelayOnState;
    bool rightAccumulatorRelayOnState;
    bool power220VACAvailable;
    int chargerVoltageADC;
    float chargerVoltageVolts;
    bool chargerConnected;
    bool charger35Ah;
    bool charger120Ah;
    bool chargingInProgress;
    bool chargingComplete;
    int curChannel1Input;
    int curChannel2Input;

    int leftMotorDutyMax;
    int rightMotorDutyMax;
    int leftMotorDuty;
    int rightMotorDuty;
    bool leftMotorStop;
    bool rightMotorStop;
    bool leftMotorActivated;
    bool rightMotorActivated;
    int platformDeceleration;
    int platformAcceleration;
    bool leftMotorAccelerating;
    bool rightMotorAccelerating;
    bool leftMotorDecelerating;
    bool rightMotorDecelerating;

    int turretMotorDutyMax;
    int turretMotorDuty;
    bool turretMotorStop;
    bool turretMotorActivated;
    int turretDeceleration;
    int turretAcceleration;
    bool turretMotorAccelerating;
    bool turretMotorDecelerating;

    int motorsPWMFrequncy;

    int leftMotorCurrentADC;
    int rightMotorCurrentADC;
    int turretMotorCurrentADC;
    float leftMotorCurrentAmps;
    float rightMotorCurrentAmps;
    float turretMotorCurrentAmps;
    bool leftMotorCurrentRead;
    bool rightMotorCurrentRead;
    bool turretMotorCurrentRead;

    int leftWheelEncoder;
    int rightWheelEncoder;
    bool leftWheelEncoderRead;
    bool rightWheelEncoderRead;
    bool leftWheelEncoderAutoreset;
    bool rightWheelEncoderAutoreset;

    //preset defaults
    int leftMotorDutyPresetMin;
    int leftMotorDutyPresetMax;
    int leftMotorDutyPresetCur;
    int rightMotorDutyPresetMin;
    int rightMotorDutyPresetMax;
    int rightMotorDutyPresetCur;
    int platformDecelerationPresetMin;
    int platformDecelerationPresetMax;
    int platformDecelerationPresetCur;
    int platformAccelerationPresetMin;
    int platformAccelerationPresetMax;
    int platformAccelerationPresetCur;
    int turretMotorDutyPresetMin;
    int turretMotorDutyPresetMax;
    int turretMotorDutyPresetCur;
    int turretDecelerationPresetMin;
    int turretDecelerationPresetMax;
    int turretDecelerationPresetCur;
    int turretAccelerationPresetMin;
    int turretAccelerationPresetMax;
    int turretAccelerationPresetCur;
};

#endif // PLATFORMCONTROLP1_H
