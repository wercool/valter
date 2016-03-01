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
    static vector<ControlDevice> scanControlDevices();

    serial::PortInfo getControlDevicePortInfo() const;
    void setControlDevicePortInfo(const serial::PortInfo &value);

    std::string getStatus() const;
    void setStatus(const std::string &value);

    static const string StatusActive;
    static const string StatusBusy;
    static const string StatusReady;

    static const uint32_t DefaultBaudRate;

    serial::Serial *getControlDevicePort() const;
    void setControlDevicePort(serial::Serial *value);

    std::string getControlDeviceName() const;
    void setControlDeviceName(const std::string &value);

    void runReadControlDeviceOutputWorker();

private:
    serial::PortInfo controlDevicePortInfo;
    std::string status;
    serial::Serial *controlDevicePort;
    std::string static sanitizeConrtolDeviceResponse(std::string &msg);
    std::string controlDeviceName;

    std::thread *readControlDeviceOutputThread;
    void readControlDeviceOutputWorker();
};

#endif // CONTROLDEVICE_H
