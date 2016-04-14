#ifndef PLATFORMLOCATIONP1_H
#define PLATFORMLOCATIONP1_H

#include <string>
#include <map>

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
    void setLeds();

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

    void spawnLeftSonarWorker();
    void spawnRightSonarWorker();

    bool getLeftSonarActivated() const;
    void setLeftSonarActivated(bool value);

    bool getRightSonarActivated() const;
    void setRightSonarActivated(bool value);

    bool getLeftSonarDirection() const;
    void setLeftSonarDirection(bool value);

    bool getRightSonarDirection() const;
    void setRightSonarDirection(bool value);

    int getLeftSonarMinAngle() const;
    void setLeftSonarMinAngle(int value);

    int getLeftSonarMaxAngle() const;
    void setLeftSonarMaxAngle(int value);

    int getRightSonarMinAngle() const;
    void setRightSonarMinAngle(int value);

    int getRightSonarMaxAngle() const;
    void setRightSonarMaxAngle(int value);

    int getLeftSonarMinRotationDuty() const;
    void setLeftSonarMinRotationDuty(int value);

    int getLeftSonarMaxRotationDuty() const;
    void setLeftSonarMaxRotationDuty(int value);

    int getRightSonarMinRotationDuty() const;
    void setRightSonarMinRotationDuty(int value);

    int getRightSonarMaxRotationDuty() const;
    void setRightSonarMaxRotationDuty(int value);

    bool getLeftSonarIntentionalAngleSet() const;
    void setLeftSonarIntentionalAngleSet(bool value);

    bool getRightSonarIntentionalAngleSet() const;
    void setRightSonarIntentionalAngleSet(bool value);

    void addLeftSonarScan(signed int angle, float distance);
    void addRightSonarScan(signed int angle, float distance);
    float getLeftSonarScan(signed int angle);
    float getRightSonarScan(signed int angle);
    void clearLeftSonarScans();
    void clearRightSonarScans();

    signed int getLeftSonarAngle() const;
    void setLeftSonarAngle(signed int value);

    signed int getRightSonarAngle() const;
    void setRightSonarAngle(signed int value);

    void spawnAccelerometerWorker();
    void spawnMagnetometerWorker();

    bool getAccelerometerWorkerActivated() const;
    void setAccelerometerWorkerActivated(bool value);

    bool getMagnetometerWorkerActivated() const;
    void setMagnetometerWorkerActivated(bool value);

    void setAccelerometerReadings(int x, int y, int z);
    void setMagnetometerReadings(int x, int y, int z);

    void setAccelerometerNaturalReadings();
    void setMagnetometerNaturalReadings();

    int *getAccelerometerReadings();
    int *getMagnetometerReadings();

    float *getAccelerometerNaturalReadings();
    float *getMagnetometerNaturalReadings();

    float getCompassHeading() const;
    void setCompassHeading(float value);

    void compassAcquireHeading();

    void spawnCompassHeadingWorker();

    bool getCompassHeadingWorkerActivated() const;
    void setCompassHeadingWorkerActivated(bool value);

    //compass
    float getHeading();

    template <typename T> struct xyzvector
    {
        T x, y, z;
    };

    template <typename Ta, typename Tb> float vector_dot(const xyzvector<Ta> *a, const xyzvector<Tb> *b)
    {
        return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
    }

    template <typename Ta, typename Tb, typename To> void vector_cross(const xyzvector<Ta> *a, const xyzvector<Tb> *b, xyzvector<To> *out)
    {
        out->x = (a->y * b->z) - (a->z * b->y);
        out->y = (a->z * b->x) - (a->x * b->z);
        out->z = (a->x * b->y) - (a->y * b->x);
    }

    void vector_normalize(xyzvector<float> *a)
    {
        float mag = sqrt(vector_dot(a, a));
        a->x /= mag;
        a->y /= mag;
        a->z /= mag;
    }

    //platform location p1 Shift Register U13, U14
    bool getShiftRegBit(int index);
    void setShiftRegBit(int index, bool state);

    void setLLLedState(bool state);
    void setLRLedState(bool state);
    void setRLLedState(bool state);
    void setRRLedState(bool state);
    void setAllSonarsLedsState(bool state);
    void setManLedState(bool state);

    void setAllLEDsOn();
    void setAllLEDsOff();

    void enableSensors();
    void disableSensors();

private:
    PlatformLocationP1();
    static PlatformLocationP1* pPlatformLocationP1;         // PLATFORM-LOCATION-P1's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

    void readSensorsWorker();

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

    void leftSonarWorker();
    bool leftSonarActivated;
    void rightSonarWorker();
    bool rightSonarActivated;

    signed int leftSonarAngle;
    signed int rightSonarAngle;
    bool leftSonarDirection;  //true - CCW, false - CW
    bool rightSonarDirection; //true - CW, false - CCW

    map<signed int, float>leftSonarScans;  //<angle, distance>
    map<signed int, float>rightSonarScans; //<angle, distance>

    std::mutex leftSonarScans_mutex;
    std::mutex rightSonarScans_mutex;


    bool leftSonarIntentionalAngleSet;
    bool rightSonarIntentionalAngleSet;

    int leftSonarMinAngle;
    int leftSonarMaxAngle;
    int rightSonarMinAngle;
    int rightSonarMaxAngle;
    int leftSonarMinRotationDuty;
    int leftSonarMaxRotationDuty;
    int rightSonarMinRotationDuty;
    int rightSonarMaxRotationDuty;

    void accelerometerWorker();
    bool accelerometerWorkerActivated;
    void magnetometerWorker();
    bool magnetometerWorkerActivated;

    int accelerometerReading[3];
    int magnetometerReading[3];

    float accelerometerNaturalReading[3];
    float magnetometerNaturalReading[3];

    float compassHeading;

    void compassHeadingWorker();
    bool compassHeadingWorkerActivated;

    bool shiftRegArray[16];

};

#endif // PLATFORMLOCATIONP1_H
