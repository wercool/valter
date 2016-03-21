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

private:
    BodyControlP1();
    static BodyControlP1* pBodyControlP1;         // BODY-CONTROL-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
};


#endif // BODYCONTROLP1_H
