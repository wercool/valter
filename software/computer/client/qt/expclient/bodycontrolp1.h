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

    bool getYawDirection() const;
    void setYawDirection(bool value);

    bool getYawMotorActivated() const;
    void setYawMotorActivated(bool value);

    int getYawStepDelay() const;
    void setYawStepDelay(int value);

    int getYawStepSwitchDelay() const;
    void setYawStepSwitchDelay(int value);

private:
    BodyControlP1();
    static BodyControlP1* pBodyControlP1;         // BODY-CONTROL-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;

    void processMessagesQueueWorker();
    void headYawWorker();
    void headPitchWorker();

    bool yawDirection;  //true - turn right, false - turn left
    bool yawMotorActivated;
    int yawStepDelay; //us
    int yawStepSwitchDelay; //us
};


#endif // BODYCONTROLP1_H
