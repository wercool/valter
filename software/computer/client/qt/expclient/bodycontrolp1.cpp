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

    resetToDefault();
    loadDefaults();

    new std::thread(&BodyControlP1::headYawWorker, this);
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
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    yawMotorActivated       = false;
    yawDirection            = false;
    yawStepDelay            = 2500;
    yawStepSwitchDelay      = 100;
}

void BodyControlP1::setModuleInitialState()
{

}

void BodyControlP1::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&BodyControlP1::processMessagesQueueWorker, this));
    getControlDevice()->addMsgToDataExchangeLog("BodyControlP1 Module process messages queue worker started...");
}

void BodyControlP1::loadDefaults()
{

}


void BodyControlP1::processMessagesQueueWorker()
{
    if (getControlDeviceIsSet())
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {
                string response = getControlDevice()->pullResponse();
            }
        }
    }
}

void BodyControlP1::headYawWorker()
{
    while (!stopAllProcesses)
    {
        if (getYawMotorActivated())
        {
            sendCommand(Valter::format_string("HEADYAW#%d", getYawStepSwitchDelay()));
            this_thread::sleep_for(std::chrono::microseconds(getYawStepDelay()));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void BodyControlP1::headPitchWorker()
{
    while (!stopAllProcesses)
    {
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int BodyControlP1::getYawStepSwitchDelay() const
{
    return yawStepSwitchDelay;
}

void BodyControlP1::setYawStepSwitchDelay(int value)
{
    yawStepSwitchDelay = value;
}

int BodyControlP1::getYawStepDelay() const
{
    return yawStepDelay;
}

void BodyControlP1::setYawStepDelay(int value)
{
    yawStepDelay = value;
}

bool BodyControlP1::getYawMotorActivated() const
{
    return yawMotorActivated;
}

void BodyControlP1::setYawMotorActivated(bool value)
{
    yawMotorActivated = value;
}

bool BodyControlP1::getYawDirection() const
{
    return yawDirection;
}

void BodyControlP1::setYawDirection(bool value)
{
    //true - turn right, false - turn left
    yawDirection = value;
    if (yawDirection)
    {
        sendCommand("HEADYAWRIGHT");
    }
    else
    {
        sendCommand("HEADYAWLEFT");
    }
}
