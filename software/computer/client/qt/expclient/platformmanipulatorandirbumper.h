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
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();

    bool prepareManLink1Movement();

    //manipulator
    const static double rootX;
    const static double rootY;
    const static double man_l1;
    const static double man_l2;
    const static double man_l1_l2;
    const static double man_l2_l3;
    const static double man_l3;
    static double man_a;
    static double man_b;
    static double man_g;

    bool getLink1MovementDirection() const;
    bool setLink1MovementDirection(bool value);

    bool getLink1MotorActivated() const;
    void setLink1MotorActivated(bool value);

private:
    PlatformManipulatorAndIRBumper();
    static PlatformManipulatorAndIRBumper* pPlatformManipulatorAndIRBumper;         // PLATFORM-MANIPULATOR-AND-IR-BUMPER's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

    bool link1MovementDirection;
    bool link1MotorActivated;
};

#endif // PLATFORMMANIPULATORANDIRBUMPER_H
