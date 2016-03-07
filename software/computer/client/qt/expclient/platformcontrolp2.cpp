#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformcontrolp2.h"

PlatformControlP2 *PlatformControlP2::pPlatformControlP2 = NULL;
bool PlatformControlP2::instanceFlag = false;
const string PlatformControlP2::controlDeviceId = "PLATFORM-CONTROL-P2";

PlatformControlP2::PlatformControlP2()
{
    Valter::log(PlatformControlP2::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;
}

string PlatformControlP2::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformControlP2 *PlatformControlP2::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformControlP2 = new PlatformControlP2();
        instanceFlag = true;
        return pPlatformControlP2;
    }
    else
    {
        return pPlatformControlP2;
    }
}

void PlatformControlP2::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformControlP2::controlDeviceId.c_str()));
    }
}

void PlatformControlP2::resetToDefault()
{
    getControlDevice()->addMsgToDataExchangeLog(Valter::format_string("%s Module Reset to default!", PlatformControlP2::controlDeviceId.c_str()));
}

void PlatformControlP2::spawnProcessMessagesQueueWorkerThread()
{
    //setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP2::processMessagesQueueWorker, this));
}
