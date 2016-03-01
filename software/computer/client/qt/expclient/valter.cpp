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
    qDebug("Valter's model version: %s", getVersion().c_str());
    listControlDevices();
}

vector<ControlDevice> Valter::getControlDevices() const
{
    return controlDevices;
}

void Valter::setControlDevices(const vector<ControlDevice> &value)
{
    controlDevices = value;
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

void Valter::listControlDevices(bool fullInfo)
{
    ControlDevice::listDevices(fullInfo);
}

void Valter::scanControlDevices()
{
    controlDevices = ControlDevice::scanControlDevices();
}
