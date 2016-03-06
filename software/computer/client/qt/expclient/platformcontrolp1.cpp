#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformcontrolp1.h"

PlatformControlP1 *PlatformControlP1::pPlatformControlP1 = NULL;
bool PlatformControlP1::instanceFlag = false;
const string PlatformControlP1::controlDeviceId = "PLATFORM-CONTROL-P1";

PlatformControlP1::PlatformControlP1()
{
    Valter::log(PlatformControlP1::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;
    resetValuesToDefault();
}
bool PlatformControlP1::getRightAccumulatorConnected() const
{
    return rightAccumulatorConnected;
}

void PlatformControlP1::setRightAccumulatorConnected(bool value)
{
    rightAccumulatorConnected = value;
}

bool PlatformControlP1::getLeftAccumulatorConnected() const
{
    return leftAccumulatorConnected;
}

void PlatformControlP1::setLeftAccumulatorConnected(bool value)
{
    leftAccumulatorConnected = value;
}

bool PlatformControlP1::getRightAccumulatorRelayOnState() const
{
    return rightAccumulatorRelayOnState;
}

void PlatformControlP1::setRightAccumulatorRelayOnState(bool value)
{
    rightAccumulatorRelayOnState = value;
}

bool PlatformControlP1::getLeftAccumulatorRelayOnState() const
{
    return leftAccumulatorRelayOnState;
}

void PlatformControlP1::setLeftAccumulatorRelayOnState(bool value)
{
    leftAccumulatorRelayOnState = value;
}

bool PlatformControlP1::getPower5VOnState() const
{
    return power5VOnState;
}

void PlatformControlP1::setPower5VOnState(bool value)
{
    power5VOnState = value;
}

bool PlatformControlP1::getMainAccumulatorRelayOnState() const
{
    return mainAccumulatorRelayOnState;
}

void PlatformControlP1::setMainAccumulatorRelayOnState(bool value)
{
    mainAccumulatorRelayOnState = value;
}

int PlatformControlP1::getRightMotorDuty() const
{
    return rightMotorDuty;
}

void PlatformControlP1::setRightMotorDuty(int value)
{
    rightMotorDuty = value;
}


string PlatformControlP1::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformControlP1 *PlatformControlP1::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformControlP1 = new PlatformControlP1();
        instanceFlag = true;
        return pPlatformControlP1;
    }
    else
    {
        return pPlatformControlP1;
    }
}

int PlatformControlP1::getLeftMotorDuty() const
{
    return leftMotorDuty;
}

void PlatformControlP1::setLeftMotorDuty(int value)
{
    leftMotorDuty = value;
}

void PlatformControlP1::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformControlP1::controlDeviceId.c_str()));
    }
}

void PlatformControlP1::resetToDefault()
{
    resetValuesToDefault();
    getControlDevice()->addMsgToDataExchangeLog(Valter::format_string("%s Module Reset to default!", PlatformControlP1::controlDeviceId.c_str()));
}

void PlatformControlP1::resetValuesToDefault()
{
    power5VOnState                      = false;
    leftAccumulatorConnected            = false;
    rightAccumulatorConnected           = false;
    mainAccumulatorRelayOnState         = false;
    leftAccumulatorRelayOnState         = false;
    rightAccumulatorRelayOnState        = false;
    leftMotorDuty                       = 1;
    rightMotorDuty                      = 1;
}
