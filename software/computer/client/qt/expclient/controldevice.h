#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <unistd.h>
#include <thread>
#include <cassert>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

#include "serial/include/serial/serial.h"


using namespace std;

class ControlDevice
{
public:
    ControlDevice();
    static void listDevices();
    static void scanControlDevices();

    void reScanThisControlDevice();
    static void USBSysDeviceReset(string sysDevicePath);

    static const string StatusReady;
    static const string StatusActive;
    static const uint32_t DefaultBaudRate;

    static const uint32_t wdResetTime = 2500;

    serial::Serial *getControlDevicePort() const;
    void setControlDevicePort(serial::Serial *value);
    string getControlDeviceId() const;
    void setControlDeviceId(const string &value);
    string getStatus() const;
    void setStatus(const string &value);

    void activate();
    void deactivate();

    std::thread *getControlDeviceThread() const;
    void setControlDeviceThread(std::thread *value);
    void spawnControlDeviceThreadWorker();

    void addRequest(string msg);
    void addResponse(string msg);
    string pullRequest();
    string pullResponse();
    int responsesAvailable();
    int requestsAwainting();
    void clearMessageQueue();

    void addMsgToDataExchangeLog(string msg);
    string getMsgFromDataExchangeLog();
    int dataExchangeLogAvailable();
    void clearDataExchangeLog();

    static const int maxLogBufferSize = 100;
    static const string controlDeviceSysPathCmd;

    string getSysDevicePath() const;
    void setSysDevicePath(const string &value);

    bool getResetWDTimer() const;
    void setResetWDTimer(bool value);

    bool getRescanningAfterPossibleReset() const;
    void setRescanningAfterPossibleReset(bool value);

    bool getFailedAfterRescanning() const;
    void setFailedAfterRescanning(bool value);

    unsigned int getRescanNum() const;
    void setRescanNum(unsigned int value);

    bool getIntentionalWDTimerResetOnAT91SAM7s() const;
    void setIntentionalWDTimerResetOnAT91SAM7s(bool value);

    bool getAutoReActivation() const;
    void setAutoReActivation(bool value);

    unsigned char getWdTimerNotResetCnt() const;
    void setWdTimerNotResetCnt(unsigned char value);

private:
    string controlDeviceId;
    serial::Serial *controlDevicePort;
    static string sanitizeConrtolDeviceResponse(string &msg);
    string status;
    string sysDevicePath;

    void controlDeviceThreadWorker();
    std::thread *controlDeviceThread;

    void resetUSBSysDevice();
    bool resetWDTimer;
    bool intentionalWDTimerResetOnAT91SAM7s;
    unsigned char wdTimerNotResetCnt;

    bool rescanningAfterPossibleReset;
    bool failedAfterRescanning;
    unsigned int rescanNum;
    bool autoReActivation;

    vector<string> responses;
    vector<string> requests;
    vector<string> dataExchangeLog;

    std::mutex responses_mutex;
    std::mutex requests_mutex;
    std::mutex dataExchangeLog_mutex;
};

#endif // CONTROLDEVICE_H
