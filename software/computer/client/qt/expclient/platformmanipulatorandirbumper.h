#ifndef PLATFORMMANIPULATORANDIRBUMPER_H
#define PLATFORMMANIPULATORANDIRBUMPER_H

#include <string>

#include "ivaltermodule.h"

using namespace std;

class PlatformManipulatorAndIRBumper: public IValterModule
{
public:
    static PlatformManipulatorAndIRBumper *getInstance();

    static string getControlDeviceId();

    void stopAll();
    void resetToDefault();
    void spawnProcessMessagesQueueWorkerThread();
    void loadDefaults();

private:
    PlatformManipulatorAndIRBumper();
    static PlatformManipulatorAndIRBumper* pPlatformManipulatorAndIRBumper;         // PLATFORM-MANIPULATOR-AND-IR-BUMPER's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
};

#endif // PLATFORMMANIPULATORANDIRBUMPER_H
