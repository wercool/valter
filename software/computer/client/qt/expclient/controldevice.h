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
    static void listDevices(bool fullInfo = false);
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

private:
    string controlDeviceId;
    serial::Serial *controlDevicePort;
    static string sanitizeConrtolDeviceResponse(string &msg);
    string status;
};

#endif // CONTROLDEVICE_H
