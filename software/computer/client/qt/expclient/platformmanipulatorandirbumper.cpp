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

    link1MovementDirection = false;
    link1MotorActivated = false;
}

void PlatformManipulatorAndIRBumper::setModuleInitialState()
{

}

void PlatformManipulatorAndIRBumper::loadDefaults()
{

}

void PlatformManipulatorAndIRBumper::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&PlatformManipulatorAndIRBumper::processMessagesQueueWorker, this));
    getControlDevice()->addMsgToDataExchangeLog("PlatformManipulatorAndIRBumper Module process messages queue worker started...");
}

void PlatformManipulatorAndIRBumper::processMessagesQueueWorker()
{
    if (getControlDeviceIsSet())
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {

            }
            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        getControlDevice()->addMsgToDataExchangeLog("PlatformManipulatorAndIRBumper Module process messages queue worker stopped!");
    }
}

bool PlatformManipulatorAndIRBumper::prepareManLink1Movement()
{
    return true;
}

//setters and getters
bool PlatformManipulatorAndIRBumper::getLink1MotorActivated() const
{
    return link1MotorActivated;
}

void PlatformManipulatorAndIRBumper::setLink1MotorActivated(bool value)
{
    link1MotorActivated = value;
}

bool PlatformManipulatorAndIRBumper::getLink1MovementDirection() const
{
    return link1MovementDirection;
}

bool PlatformManipulatorAndIRBumper::setLink1MovementDirection(bool value)
{
    link1MovementDirection = value;
    return true;
}

string PlatformManipulatorAndIRBumper::getControlDeviceId()
{
    return controlDeviceId;
}
