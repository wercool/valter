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

    bool getLink1MotorStop() const;
    void setLink1MotorStop(bool value);

    int getLink1MotorDuty() const;
    void setLink1MotorDuty(int value);

    int getLink1MotorDutyMax() const;
    void setLink1MotorDutyMax(int value);

    int getLink1MotorAcceleration() const;
    void setLink1MotorAcceleration(int value);

    int getLink1MotorDeceleration() const;
    void setLink1MotorDeceleration(int value);

    int getLink1MotorDutyPresetCur() const;
    void setLink1MotorDutyPresetCur(int value);

    int getLink1MotorDutyPresetMin() const;
    void setLink1MotorDutyPresetMin(int value);

    int getLink1MotorDutyPresetMax() const;
    void setLink1MotorDutyPresetMax(int value);

    int getLink2MotorDuty() const;
    void setLink2MotorDuty(int value);

    int getLink2MotorDutyMax() const;
    void setLink2MotorDutyMax(int value);

    int getLink2MotorAcceleration() const;
    void setLink2MotorAcceleration(int value);

    int getLink2MotorDeceleration() const;
    void setLink2MotorDeceleration(int value);

    bool getLink2MovementDirection() const;
    void setLink2MovementDirection(bool value);

    bool getLink2MotorActivated() const;
    void setLink2MotorActivated(bool value);

    bool getLink2MotorStop() const;
    void setLink2MotorStop(bool value);

    int getLink2MotorDutyPresetCur() const;
    void setLink2MotorDutyPresetCur(int value);

    int getLink2MotorDutyPresetMin() const;
    void setLink2MotorDutyPresetMin(int value);

    int getLink2MotorDutyPresetMax() const;
    void setLink2MotorDutyPresetMax(int value);

    bool getPower24VOnOff() const;
    void setPower24VOnOff(bool value);
    void setPower24VOn();
    void setPower24VOff();

    bool getLink1MotorAccelerating() const;
    void setLink1MotorAccelerating(bool value);

    bool getLink1MotorDecelerating() const;
    void setLink1MotorDecelerating(bool value);

private:
    PlatformManipulatorAndIRBumper();
    static PlatformManipulatorAndIRBumper* pPlatformManipulatorAndIRBumper;         // PLATFORM-MANIPULATOR-AND-IR-BUMPER's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();
    void manLink1MovementWorker();

    bool power24VOnOff;

    //------------------------------------------------man link1
    int link1MotorDuty;
    int link1MotorDutyMax;
    int link1MotorAcceleration;
    int link1MotorDeceleration;

    bool link1MotorAccelerating;
    bool link1MotorDecelerating;

    bool link1MovementDirection;
    bool link1MotorActivated;
    bool link1MotorStop;

    //presets
    int link1MotorDutyPresetCur;
    int link1MotorDutyPresetMin;
    int link1MotorDutyPresetMax;

    //------------------------------------------------man link2
    int link2MotorDuty;
    int link2MotorDutyMax;
    int link2MotorAcceleration;
    int link2MotorDeceleration;

    bool link2MovementDirection;
    bool link2MotorActivated;
    bool link2MotorStop;

    //presets
    int link2MotorDutyPresetCur;
    int link2MotorDutyPresetMin;
    int link2MotorDutyPresetMax;
};

#endif // PLATFORMMANIPULATORANDIRBUMPER_H
