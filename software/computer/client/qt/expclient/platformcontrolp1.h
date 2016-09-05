#ifndef PLATFORMCONTROLP1_H
#define PLATFORMCONTROLP1_H

#include <string>

#include "tasks/itask.h"
#include "ivaltermodule.h"

using namespace std;

class PlatformControlP1: public IValterModule
{
public:
    static PlatformControlP1 *getInstance();

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

    bool preparePlatformMovement();

    void toggle5VSource(bool state);
    bool getPower5VOnState() const;
    void setPower5VOnState(bool value);

    void toggleMainAccumulatorRelayState(bool state);
    bool getMainAccumulatorRelayOnState() const;
    void setMainAccumulatorRelayOnState(bool value);

    void toggleLeftAccumulatorRelay(bool state);
    bool getLeftAccumulatorRelayOnState() const;
    void setLeftAccumulatorRelayOnState(bool value);

    void toggleRightAccumulatorRelay(bool state);
    bool getRightAccumulatorRelayOnState() const;
    void setRightAccumulatorRelayOnState(bool value);

    int getLeftMotorDuty() const;
    void setLeftMotorDuty(int value);

    int getRightMotorDuty() const;
    void setRightMotorDuty(int value);

    void toggleLeftAccumulator(bool state);
    bool getLeftAccumulatorConnected() const;
    void setLeftAccumulatorConnected(bool value);

    void toggleRightAccumulator(bool state);
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

    bool getTurretPositionRead() const;
    void setTurretPositionRead(bool value);

    int getTurretPositionADC() const;
    void setTurretPositionADC(int value);

    float getTurretPositionDeg() const;
    void setTurretPositionDeg(float value);

    float getTurretPositionRad() const;
    void setTurretPositionRad(float value);

    bool getLeftWheelEncoderGetOnce() const;
    void setLeftWheelEncoderGetOnce(bool value);

    bool getRightWheelEncoderGetOnce() const;
    void setRightWheelEncoderGetOnce(bool value);

    bool getTurretPositionGetOnce() const;
    void setTurretPositionGetOnce(bool value);

    int getMainAccumulatorVoltageADC() const;
    void setMainAccumulatorVoltageADC(int value);

    int getLeftAccumulatorVoltageADC() const;
    void setLeftAccumulatorVoltageADC(int value);

    int getRightAccumulatorVoltageADC() const;
    void setRightAccumulatorVoltageADC(int value);

    int getMainAccumulatorAmperageTotalADC() const;
    void setMainAccumulatorAmperageTotalADC(int value);

    int getPlatformAmperageADC() const;
    void setPlatformAmperageADC(int value);

    int getBodyAmperageADC() const;
    void setBodyAmperageADC(int value);

    int getLeftAccumulatorAmperageADC() const;
    void setLeftAccumulatorAmperageADC(int value);

    int getRightAccumulatorAmperageADC() const;
    void setRightAccumulatorAmperageADC(int value);

    float getMainAccumulatorVoltageVolts() const;
    void setMainAccumulatorVoltageVolts(float value);

    float getLeftAccumulatorVoltageVolts() const;
    void setLeftAccumulatorVoltageVolts(float value);

    float getRightAccumulatorVoltageVolts() const;
    void setRightAccumulatorVoltageVolts(float value);

    float getMainAccumulatorAmperageTotalAmps() const;
    void setMainAccumulatorAmperageTotalAmps(float value);

    float getPlatformAmperageAmps() const;
    void setPlatformAmperageAmps(float value);

    float getBodyAmperageAmps() const;
    void setBodyAmperageAmps(float value);

    float getLeftAccumulatorAmperageAmps() const;
    void setLeftAccumulatorAmperageAmps(float value);

    float getRightAccumulatorAmperageAmps() const;
    void setRightAccumulatorAmperageAmps(float value);

    bool getMainAccumulatorVoltageRead() const;
    void setMainAccumulatorVoltageRead(bool value);

    bool getLeftAccumulatorVoltageRead() const;
    void setLeftAccumulatorVoltageRead(bool value);

    bool getRightAccumulatorVoltageRead() const;
    void setRightAccumulatorVoltageRead(bool value);

    bool getMainAccumulatorAmperageTotalRead() const;
    void setMainAccumulatorAmperageTotalRead(bool value);

    bool getPlatformAmperageRead() const;
    void setPlatformAmperageRead(bool value);

    bool getBodyAmperageRead() const;
    void setBodyAmperageRead(bool value);

    bool getLeftAccumulatorAmperageRead() const;
    void setLeftAccumulatorAmperageRead(bool value);

    bool getRightAccumulatorAmperageRead() const;
    void setRightAccumulatorAmperageRead(bool value);

    bool getChargerVoltageRead() const;
    void setChargerVoltageRead(bool value);

    bool getMainAccumulatorVoltageReadADCPreset() const;
    void setMainAccumulatorVoltageReadADCPreset(bool value);

    bool getLeftAccumulatorVoltageReadADCPreset() const;
    void setLeftAccumulatorVoltageReadADCPreset(bool value);

    bool getRightAccumulatorVoltageReadADCPreset() const;
    void setRightAccumulatorVoltageReadADCPreset(bool value);

    bool getMainAccumulatorAmperageTotalReadADCPreset() const;
    void setMainAccumulatorAmperageTotalReadADCPreset(bool value);

    bool getPlatformAmperageReadADCPreset() const;
    void setPlatformAmperageReadADCPreset(bool value);

    bool getBodyAmperageReadADCPreset() const;
    void setBodyAmperageReadADCPreset(bool value);

    bool getLeftAccumulatorAmperageReadADCPreset() const;
    void setLeftAccumulatorAmperageReadADCPreset(bool value);

    bool getRightAccumulatorAmperageReadADCPreset() const;
    void setRightAccumulatorAmperageReadADCPreset(bool value);

    bool getChargerVoltageReadADCPreset() const;
    void setChargerVoltageReadADCPreset(bool value);

    int getAdditionalReadingsDelayPresetMin() const;
    void setAdditionalReadingsDelayPresetMin(int value);

    int getAdditionalReadingsDelayPresetMax() const;
    void setAdditionalReadingsDelayPresetMax(int value);

    int getAdditionalReadingsDelayCur() const;
    void setAdditionalReadingsDelayCur(int value);

    /**************************************************** VALUES MAP ******************************************************/
    std::map<std::string, std::string> values;
    string getValue(string key);

    //Valter base platform characteristics
    static const int vagueEncoderTicksPerMeter;
    static const int vagueEncoderTicksPer360Turn;

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

    void additionalReadingsScanWorker();

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
    bool turretPositionGetOnce;

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
    bool leftWheelEncoderGetOnce;
    bool rightWheelEncoderGetOnce;

    bool turretPositionRead;
    int turretPositionADC;
    float turretPositionDeg;
    float turretPositionRad;


    //additional readings
    int mainAccumulatorVoltageADC;
    int leftAccumulatorVoltageADC;
    int rightAccumulatorVoltageADC;
    int mainAccumulatorAmperageTotalADC;
    int platformAmperageADC;
    int bodyAmperageADC;
    int leftAccumulatorAmperageADC;
    int rightAccumulatorAmperageADC;

    float mainAccumulatorVoltageVolts;
    float leftAccumulatorVoltageVolts;
    float rightAccumulatorVoltageVolts;
    float mainAccumulatorAmperageTotalAmps;
    float platformAmperageAmps;
    float bodyAmperageAmps;
    float leftAccumulatorAmperageAmps;
    float rightAccumulatorAmperageAmps;

    bool mainAccumulatorVoltageRead;
    bool leftAccumulatorVoltageRead;
    bool rightAccumulatorVoltageRead;
    bool mainAccumulatorAmperageTotalRead;
    bool platformAmperageRead;
    bool bodyAmperageRead;
    bool leftAccumulatorAmperageRead;
    bool rightAccumulatorAmperageRead;
    bool chargerVoltageRead;

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

    bool mainAccumulatorVoltageReadADCPreset;
    bool leftAccumulatorVoltageReadADCPreset;
    bool rightAccumulatorVoltageReadADCPreset;
    bool mainAccumulatorAmperageTotalReadADCPreset;
    bool platformAmperageReadADCPreset;
    bool bodyAmperageReadADCPreset;
    bool leftAccumulatorAmperageReadADCPreset;
    bool rightAccumulatorAmperageReadADCPreset;
    bool chargerVoltageReadADCPreset;

    int additionalReadingsDelayPresetMin;
    int additionalReadingsDelayPresetMax;
    int additionalReadingsDelayCur;

    /**************************************************** TASKS ******************************************************/
    std::map<std::string, function<ITask*(void)>> tasks;
};

#endif // PLATFORMCONTROLP1_H
