#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <unistd.h>
#include <thread>
#include <list>

#include "serial/include/serial/serial.h"


using namespace std;

class ControlDevice
{
public:
    ControlDevice();
    static void listDevices();
    static void scanControlDevices();

    static const string StatusReady;
    static const string StatusActive;
    static const uint32_t DefaultBaudRate;

    serial::Serial *getControlDevicePort() const;
    void setControlDevicePort(serial::Serial *value);

    string getControlDeviceId() const;
    void setControlDeviceId(const string &value);

    string getStatus() const;
    void setStatus(const string &value);

    std::thread *getControlDeviceThread() const;
    void setControlDeviceThread(std::thread *value);
    void spawnControlDeviceThreadWorker();

    void addRequest(string msg);
    void addResponse(string msg);
    string pullRequest();
    string pullResponse();
    int responsesAvailable();
    int requestsAwainting();

    void addMsgToDataExchangeLog(string msg);
    string getMsgFromDataExchangeLog();
    int dataExchangeLogAvailable();
    void clearDataExchangeLog();

private:
    string controlDeviceId;
    serial::Serial *controlDevicePort;
    static string sanitizeConrtolDeviceResponse(string &msg);
    string status;

    void controlDeviceThreadWorker();
    std::thread *controlDeviceThread;

    list<string> responses;
    list<string> requests;
    list<string> dataExchangeLog;
};

#endif // CONTROLDEVICE_H
