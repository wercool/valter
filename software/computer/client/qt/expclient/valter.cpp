//Qt specific
#include <QtDebug>


#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "controldevice.h"

Valter* Valter::pValter = NULL;
bool Valter::instanceFlag = false;

Valter::Valter()
{
    controlDeviceIds = {"PLATFORM-CONTROL-P1", "BODY-CONTROL-P1"};
}

map<string, ControlDevice *> Valter::getControlDevicesMap() const
{
    return controlDevicesMap;
}

void Valter::setControlDevicesMap(const map<string, ControlDevice *> &value)
{
    controlDevicesMap = value;
}

Valter* Valter::getInstance()
{
    if(! instanceFlag)
    {
        pValter = new Valter();
        instanceFlag = true;
        return pValter;
    }
    else
    {
        return pValter;
    }
}

string Valter::getVersion()
{
    return "0.0.1";
}

void Valter::scanControlDevices()
{
    ControlDevice::scanControlDevices();
}

void Valter::addControlDevice(string controlDeviceId, string port)
{
    serial::Serial *controlDevicePort = new serial::Serial(port, ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));
    ControlDevice *controlDevice = new ControlDevice();
    controlDevice->setControlDeviceId(controlDeviceId);
    controlDevice->setControlDevicePort(controlDevicePort);
    controlDevice->getControlDevicePort()->close();
    controlDevice->setStatus(ControlDevice::StatusReady);
    addControlDeviceToControlDevicesMap(controlDevice);
}

void Valter::addControlDeviceToControlDevicesMap(ControlDevice *controlDevice)
{
    controlDevicesMap[controlDevice->getControlDeviceId()] = controlDevice;
}

ControlDevice *Valter::getControlDeviceById(string controlDeviceId)
{
    return controlDevicesMap[controlDeviceId];
}

void Valter::closeAllControlDevicePorts()
{

    map<string, ControlDevice*> controlDevicesMap = getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        if (controlDevice->getControlDevicePort()->isOpen())
        {
            controlDevice->getControlDevicePort()->close();
        }
    }
}
