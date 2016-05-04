#ifndef IVALTERMODULE_H
#define IVALTERMODULE_H

#include "tcpinterface.h"
#include "controldevice.h"
#include <mutex>

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
        reloadDefaults = false;
    }

    bool controlDeviceIsSet;
    bool stopAllProcesses;

    TCPInterface *tcpInterface = NULL;

    virtual void stopAll() = 0;
    virtual void resetToDefault() = 0;
    virtual void loadDefaults() = 0;
    virtual void setModuleInitialState() = 0;
    virtual void spawnProcessMessagesQueueWorkerThread() = 0;
    virtual void initTcpInterface() = 0;

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

    bool getReloadDefaults() const
    {
        return reloadDefaults;
    }

    void setReloadDefaults(bool value)
    {
        reloadDefaults = value;
    }

    void addActionToDelayedGUIActions(int action)
    {
        std::lock_guard<std::mutex> guard(delayedGUIActions_mutex);
        delayedGUIActions.push_back(action);
    }

    int getActionFromDelayedGUIActions()
    {
        std::lock_guard<std::mutex> guard(delayedGUIActions_mutex);
        int action = (int)delayedGUIActions.front();
        delayedGUIActions.pop_front();
        return action;
    }

    bool areActionsInDelayedGUIActions()
    {
        if (delayedGUIActions.size() > 0)
        {
           return true;
        }
        else
        {
            return false;
        }
    }

    static const enum {RELOAD_DEFAULTS = 1} GUIACTION;

private:
    ControlDevice *controlDevice;
    std::thread *processMessagesQueueWorkerThread;
    map<string, string> defaults;
    bool reloadDefaults;
    list<int> delayedGUIActions;
    std::mutex delayedGUIActions_mutex;

};

#endif // IVALTERMODULE_H
