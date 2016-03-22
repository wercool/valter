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
    void loadDefaults();
    void setModuleInitialState();
    void spawnProcessMessagesQueueWorkerThread();

    bool getRedLedState(int index);
    void setRedLedState(int index, bool value);
    bool getGreenLedState(int index);
    void setGreenLedState(int index, bool value);
    void setRedLedsState();

    void setReadIRSensor(int index, bool value);
    bool getReadIRSensor(int index);
    void setReadIRSensorADCPreset(int index, bool value);
    bool getReadIRSensorADCPreset(int index);

    void setReadUSSensor(int index, bool value);
    bool getReadUSSensor(int index);
    void setReadUSSensorTicksPreset(int index, bool value);
    bool getReadUSSensorTicksPreset(int index);

    void setIRSensorADC(int channel, int value);
    void setUSSensorTicks(int channel, int value);
    int getIRSensorADC(int channel);
    int getUSSensorTicks(int channel);
    float getIRSensorMeters(int channel);
    float getUSSensorMeters(int channel);

    void readSensorsWorker();

    bool getLedStatesSet() const;
    void setLedStatesSet(bool value);

    void spawnLEDStatesWorker();

    int getUsSignalDuty() const;
    void setUsSignalDuty(int value);

    int getUsSignalBurst() const;
    void setUsSignalBurst(int value);

    int getUsSignalDelay() const;
    void setUsSignalDelay(int value);

    int getRelativeUSSensorVoltage() const;
    void setRelativeUSSensorVoltageUp();
    void setRelativeUSSensorVoltageDown();

private:
    PlatformLocationP1();
    static PlatformLocationP1* pPlatformLocationP1;         // PLATFORM-LOCATION-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();
    void LEDStatesWorker();

    bool ledStatesSet;

    bool redLedArray[12];
    bool greenLedArray[12];

    bool readIRSensor[12];
    bool readIRSensorADCPreset[12];
    bool readUSSensor[12];
    bool readUSSensorADCPreset[12];

    int irSensorADC[12];
    int usSensorADC[12];

    float irSensorMetersThreshold;
    float usSensorMetersThreshold;

    int relativeUSSensorVoltage;
    int usSignalDuty;
    int usSignalBurst;
    int usSignalDelay;
};

#endif // PLATFORMLOCATIONP1_H
