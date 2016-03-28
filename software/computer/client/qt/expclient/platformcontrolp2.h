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

    bool getReadLeftEncoder() const;
    void setReadLeftEncoder(bool value);

    bool getReadRightEncoder() const;
    void setReadRightEncoder(bool value);

    int getLeftEncoder() const;
    void setLeftEncoder(int value);

    int getRightEncoder() const;
    void setRightEncoder(int value);

    bool getLeftEncoderAutoreset() const;
    void setLeftEncoderAutoreset(bool value);

    bool getRightEncoderAutoreset() const;
    void setRightEncoderAutoreset(bool value);

    int getChargerMotorDutyPresetMin() const;
    void setChargerMotorDutyPresetMin(int value);

    int getChargerMotorDutyPresetMax() const;
    void setChargerMotorDutyPresetMax(int value);

    int getChargerMotorDutyPresetCur() const;
    void setChargerMotorDutyPresetCur(int value);

    int getChargerMotorPushDurationPresetMin() const;
    void setChargerMotorPushDurationPresetMin(int value);

    int getChargerMotorPushDurationPresetMax() const;
    void setChargerMotorPushDurationPresetMax(int value);

    int getChargerMotorPushDurationPresetCur() const;
    void setChargerMotorPushDurationPresetCur(int value);

    int getChargerMotorPushDuration() const;
    void setChargerMotorPushDuration(int value);

    bool getEncodersWorkerActivated() const;
    void setEncodersWorkerActivated(bool value);

    bool getIrScanningWorkerActivated() const;
    void setIrScanningWorkerActivated(bool value);

    void spawnEncodersWorker();
    void spawnIRScanningWorker();

    int getChargerMotorDuty() const;
    void setChargerMotorDuty(int value);

    int getBeepDuration() const;
    void setBeepDuration(int value);

    int getBeepDurationPresetMin() const;
    void setBeepDurationPresetMin(int value);

    int getBeepDurationPresetMax() const;
    void setBeepDurationPresetMax(int value);

    bool getChargerLeds() const;
    void setChargerLeds(bool value);

    bool getBottomFrontLeds() const;
    void setBottomFrontLeds(bool value);

    bool getBottomRearLeds() const;
    void setBottomRearLeds(bool value);

private:
    PlatformControlP2();
    static PlatformControlP2* pPlatformControlP2;       // PLATFORM-CONTROL-P2's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    bool readLeftEncoder;
    bool readRightEncoder;
    int leftEncoder;
    int rightEncoder;
    bool leftEncoderAutoreset;
    bool rightEncoderAutoreset;

    int chargerMotorDuty;
    int chargerMotorDutyPresetMin;
    int chargerMotorDutyPresetMax;
    int chargerMotorDutyPresetCur;

    int chargerMotorPushDuration;
    int chargerMotorPushDurationPresetMin;
    int chargerMotorPushDurationPresetMax;

    bool encodersWorkerActivated;
    bool irScanningWorkerActivated;

    void encodersWorker();
    void irScanningWorker();

    int beepDuration;
    int beepDurationPresetMin;
    int beepDurationPresetMax;

    bool chargerLeds;
    bool bottomFrontLeds;
    bool bottomRearLeds;
};

#endif // PLATFORMCONTROLP2_H
