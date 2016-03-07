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

    void resetValuesToDefault();

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

private:
    PlatformControlP1();
    static PlatformControlP1* pPlatformControlP1;       // PLATFORM-CONTROL-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;

    void processMessagesQueueWorker();

    //PlatforomControlP1 strictly specific methods
    void scanFor220VACAvailable();
    bool scan220ACAvailable;
    bool chargerMode; //0 - turning off; 1 - set charging
    int chargerButtonPressStep;
    void chargerModeSetting();

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

};

#endif // PLATFORMCONTROLP1_H
