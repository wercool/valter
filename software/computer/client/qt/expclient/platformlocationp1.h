#ifndef PLATFORMLOCATIONP1_H
#define PLATFORMLOCATIONP1_H

#include <string>

#include "ivaltermodule.h"

using namespace std;

class PlatformLocationP1: public IValterModule
{
public:
    static PlatformLocationP1 *getInstance();

    static string getControlDeviceId();

    void stopAll();
    void resetToDefault();
    void spawnProcessMessagesQueueWorkerThread();
    void loadDefaults();


    bool getRedLedState(int index);
    void setRedLedState(int index, bool value);
    bool getGreenLedState(int index);
    void setGreenLedState(int index, bool value);

private:
    PlatformLocationP1();
    static PlatformLocationP1* pPlatformLocationP1;         // PLATFORM-LOCATION-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

    bool redLedArray[12] = { false };
    bool greenLedArray[12] = { false };
};

#endif // PLATFORMLOCATIONP1_H
