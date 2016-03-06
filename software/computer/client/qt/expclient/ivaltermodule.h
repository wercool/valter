#ifndef IVALTERMODULE_H
#define IVALTERMODULE_H

#include "controldevice.h"

class IValterModule
{
public:
    virtual ~IValterModule() {}

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

    virtual void stopAll() = 0;
    virtual void resetToDefault() = 0;
    void resetValuesToDefault();
private:
    ControlDevice *controlDevice;
};

#endif // IVALTERMODULE_H
