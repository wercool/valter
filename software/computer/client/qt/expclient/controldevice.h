#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <unistd.h>
#include <thread>

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

    std::thread *getReadControlDeviceOutputThread() const;
    void setReadControlDeviceOutputThread(std::thread *value);

    void spawnReadControlDeviceOutputThreadWorker();

private:
    string controlDeviceId;
    serial::Serial *controlDevicePort;
    static string sanitizeConrtolDeviceResponse(string &msg);
    string status;

    void readControlDeviceOutputThreadWorker();
    std::thread *readControlDeviceOutputThread;
};

#endif // CONTROLDEVICE_H
