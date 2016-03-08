#ifndef IVALTERMODULE_H
#define IVALTERMODULE_H

#include "controldevice.h"

class IValterModule
{
public:
    virtual ~IValterModule() {}

    IValterModule()
    {
        stopAllProcesses = false;
    }

    ControlDevice *getControlDevice()
    {
        return controlDevice;
    }
    void setControlDevice(ControlDevice *value)
    {
        controlDevice = value;
        controlDeviceIsSet = true;
    }

    bool controlDeviceIsSet;
    bool stopAllProcesses;

    virtual void stopAll() = 0;
    virtual void resetToDefault() = 0;
    virtual void spawnProcessMessagesQueueWorkerThread() = 0;
    virtual void loadDefaults() = 0;

    void sendCommand(string cmd)
    {
        if (controlDeviceIsSet)
        {
            if (getControlDevice()->getStatus() == ControlDevice::StatusActive)
            {
                getControlDevice()->addRequest(cmd);
            }
        }
    }

    std::thread *getProcessMessagesQueueWorkerThread()
    {
        return processMessagesQueueWorkerThread;
    }

    void setProcessMessagesQueueWorkerThread(std::thread *value)
    {
        processMessagesQueueWorkerThread = value;
    }

    bool getControlDeviceIsSet()
    {
        return controlDeviceIsSet;
    }

    map<string, string> getDefaults() const
    {
        return defaults;
    }

    void setDefaults(const map<string, string> &value)
    {
        defaults = value;
    }

    void addDefault(string valueName, string value)
    {
        defaults[valueName] = value;
    }

    string getDefault(string valueName)
    {
        return defaults[valueName];
    }

private:
    ControlDevice *controlDevice;
    std::thread *processMessagesQueueWorkerThread;
    map<string, string> defaults;

};

#endif // IVALTERMODULE_H
