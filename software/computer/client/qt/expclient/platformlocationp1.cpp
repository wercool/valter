#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformlocationp1.h"

PlatformLocationP1 *PlatformLocationP1::pPlatformLocationP1= NULL;
bool PlatformLocationP1::instanceFlag = false;
const string PlatformLocationP1::controlDeviceId = "PLATFORM-LOCATION-P1";

PlatformLocationP1::PlatformLocationP1()
{
    Valter::log(PlatformLocationP1::controlDeviceId + " singleton started");
    setReloadDefaults(false);
    controlDeviceIsSet = false;
}

string PlatformLocationP1::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformLocationP1 *PlatformLocationP1::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformLocationP1 = new PlatformLocationP1();
        instanceFlag = true;
        return pPlatformLocationP1;
    }
    else
    {
        return pPlatformLocationP1;
    }
}

void PlatformLocationP1::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformLocationP1::controlDeviceId.c_str()));
    }
}

void PlatformLocationP1::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }
}

void PlatformLocationP1::spawnProcessMessagesQueueWorkerThread()
{
    //setProcessMessagesQueueWorkerThread(new std::thread(&PlatformLocationP1::processMessagesQueueWorker, this));
}

void PlatformLocationP1::loadDefaults()
{

}
