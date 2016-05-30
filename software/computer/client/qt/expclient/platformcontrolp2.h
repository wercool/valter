#ifndef PLATFORMCONTROLP2_H
#define PLATFORMCONTROLP2_H

#include <string>
#include <map>

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
    void initTcpInterface();
    void initTcpCommandAcceptorInterface();
    void processControlDeviceResponse(string response);
    unsigned int executeTask(std::string taskScriptLine);

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

    void resetLeftEncoder();
    void resetRightEncoder();

    void getLeftEncoderOnce(bool value);
    void getRightEncoderOnce(bool value);

    bool leftEncoderGetOnce;
    bool rightEncoderGetOnce;

    void addIRScannerScan(signed int angle, float distance);
    float getIRScannerScan(signed int angle);
    void clearIRScannerScans();

    signed int getIRScannerAngle() const;
    void setIRScannerAngle(signed int value);

    bool getIRScannerDirection() const;
    void setIRScannerDirection(bool value);

    int getIRScannerMinAngle() const;
    void setIRScannerMinAngle(int value);

    int getIRScannerMaxAngle() const;
    void setIRScannerMaxAngle(int value);

    bool getIRScannerIntentionalAngleSet() const;
    void setIRScannerIntentionalAngleSet(bool value);

    void pushChargerMotor();

    bool getChargerMotorDirection() const;
    void setChargerMotorDirection(bool value);

    void spawnChargerMotorRotateWorker();

    bool getChargerMotorRotateWorkerActivated() const;
    void setChargerMotorRotateWorkerActivated(bool value);

    void stopChargerMotor();

    void alarmBeep();
    void ALARMOnOff(bool state);

    void disableIRScannerServo();
    void resetIRScannerServo();

    void requestIrScannerReadingADC();
    int getIrScannerReadingADC() const;
    void setIrScannerReadingADC(int value);

    int getIntetntionalIRScannerReadingReuqest() const;
    void setIntetntionalIRScannerReadingReuqest(int value);

private:
    PlatformControlP2();
    static PlatformControlP2* pPlatformControlP2;       // PLATFORM-CONTROL-P2's singleton instance
    static bool instanceFlag;
    static const string controlDeviceId;
    static const string defaultsFilePath;

    void processMessagesQueueWorker();

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

    bool chargerMotorDirection;

    bool encodersWorkerActivated;
    bool irScanningWorkerActivated;
    bool chargerMotorRotateWorkerActivated;

    void encodersWorker();
    void irScanningWorker();
    void chargerMotorRotateWorker();

    int beepDuration;
    int beepDurationPresetMin;
    int beepDurationPresetMax;

    bool chargerLeds;
    bool bottomFrontLeds;
    bool bottomRearLeds;

    int iRScannerAngle;

    int irScannerReadingADC;
    bool intetntionalIRScannerReadingReuqest;

    map<signed int, float>IRScannerScans;  //<angle, distance>
    std::mutex IRScannerScans_mutex;

    bool iRScannerDirection;  //true - CCW, false - CW

    int iRScannerMinAngle;
    int iRScannerMaxAngle;
    int iRScannerIntentionalAngleSet;

    bool ALARM;
};

#endif // PLATFORMCONTROLP2_H
