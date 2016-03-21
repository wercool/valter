#ifndef PLATFORMCONTROLP2_H
#define PLATFORMCONTROLP2_H

#include <string>

#include "ivaltermodule.h"

using namespace std;

class PlatformControlP2: public IValterModule
{
public:
    static PlatformControlP2 *getInstance();

    static string getControlDeviceId();

    void stopAll();
    void resetToDefault();
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();

private:
    PlatformControlP2();
    static PlatformControlP2* pPlatformControlP2;       // PLATFORM-CONTROL-P2's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
};

#endif // PLATFORMCONTROLP2_H
