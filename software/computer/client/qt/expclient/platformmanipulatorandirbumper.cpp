#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformmanipulatorandirbumper.h"

PlatformManipulatorAndIRBumper *PlatformManipulatorAndIRBumper::pPlatformManipulatorAndIRBumper = NULL;
bool PlatformManipulatorAndIRBumper::instanceFlag = false;
const string PlatformManipulatorAndIRBumper::controlDeviceId = "PLATFORM-MANIPULATOR-AND-IR-BUMPER";

PlatformManipulatorAndIRBumper::PlatformManipulatorAndIRBumper()
{
    Valter::log(PlatformManipulatorAndIRBumper::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;
}

string PlatformManipulatorAndIRBumper::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformManipulatorAndIRBumper *PlatformManipulatorAndIRBumper::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformManipulatorAndIRBumper = new PlatformManipulatorAndIRBumper();
        instanceFlag = true;
        return pPlatformManipulatorAndIRBumper;
    }
    else
    {
        return pPlatformManipulatorAndIRBumper;
    }
}

void PlatformManipulatorAndIRBumper::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformManipulatorAndIRBumper::controlDeviceId.c_str()));
    }
}

void PlatformManipulatorAndIRBumper::resetToDefault()
{
    getControlDevice()->addMsgToDataExchangeLog(Valter::format_string("%s Module Reset to default!", PlatformManipulatorAndIRBumper::controlDeviceId.c_str()));
}

void PlatformManipulatorAndIRBumper::setModuleInitialState()
{

}

void PlatformManipulatorAndIRBumper::spawnProcessMessagesQueueWorkerThread()
{
    //setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP2::processMessagesQueueWorker, this));
}

void PlatformManipulatorAndIRBumper::loadDefaults()
{

}
