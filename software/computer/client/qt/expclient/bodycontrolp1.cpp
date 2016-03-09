#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "bodycontrolp1.h"

BodyControlP1 *BodyControlP1::pBodyControlP1 = NULL;
bool BodyControlP1::instanceFlag = false;
const string BodyControlP1::controlDeviceId = "BODY-CONTROL-P1";

BodyControlP1::BodyControlP1()
{
    Valter::log(BodyControlP1::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;
}

string BodyControlP1::getControlDeviceId()
{
    return controlDeviceId;
}


BodyControlP1 *BodyControlP1::getInstance()
{
    if(!instanceFlag)
    {
        pBodyControlP1 = new BodyControlP1();
        instanceFlag = true;
        return pBodyControlP1;
    }
    else
    {
        return pBodyControlP1;
    }
}

void BodyControlP1::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", BodyControlP1::controlDeviceId.c_str()));
    }
}

void BodyControlP1::resetToDefault()
{
    getControlDevice()->addMsgToDataExchangeLog(Valter::format_string("%s Module Reset to default!", BodyControlP1::controlDeviceId.c_str()));
}

void BodyControlP1::spawnProcessMessagesQueueWorkerThread()
{
    //setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP2::processMessagesQueueWorker, this));
}

void BodyControlP1::loadDefaults()
{

}
