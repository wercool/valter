#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformmanipulatorandirbumper.h"

PlatformManipulatorAndIRBumper *PlatformManipulatorAndIRBumper::pPlatformManipulatorAndIRBumper = NULL;
bool PlatformManipulatorAndIRBumper::instanceFlag = false;
const string PlatformManipulatorAndIRBumper::controlDeviceId = "PLATFORM-MANIPULATOR-AND-IR-BUMPER";
const string PlatformManipulatorAndIRBumper::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/platform-manipulator-and-ir-bumper";

const double PlatformManipulatorAndIRBumper::rootX = 220;
const double PlatformManipulatorAndIRBumper::rootY = 130;
const double PlatformManipulatorAndIRBumper::man_l1      = 120;
const double PlatformManipulatorAndIRBumper::man_l2      = 128;
const double PlatformManipulatorAndIRBumper::man_l1_l2   = 30;
const double PlatformManipulatorAndIRBumper::man_l2_l3   = 10;
const double PlatformManipulatorAndIRBumper::man_l3      = 40;
double PlatformManipulatorAndIRBumper::man_a    = 0.0;
double PlatformManipulatorAndIRBumper::man_b    = 0.0;
double PlatformManipulatorAndIRBumper::man_g    = 0.0;

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
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }
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
