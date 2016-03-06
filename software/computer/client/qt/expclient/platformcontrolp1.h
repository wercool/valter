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

private:
    PlatformControlP1();
    static PlatformControlP1* pPlatformControlP1;		// PLATFORM-CONTROL-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;

    bool power5VOnState;
    bool leftAccumulatorConnected;
    bool rightAccumulatorConnected;
    bool mainAccumulatorRelayOnState;
    bool leftAccumulatorRelayOnState;
    bool rightAccumulatorRelayOnState;
    int leftMotorDuty;
    int rightMotorDuty;
};

#endif // PLATFORMCONTROLP1_H
