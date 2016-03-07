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
    new std::thread(&PlatformControlP1::lookFor220VACAvailable, this);
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

void PlatformControlP1::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP1::processMessagesQueueWorker, this));
    getControlDevice()->addMsgToDataExchangeLog("PlatformControlP1 Module process messages queue worker started...");
}

void PlatformControlP1::resetValuesToDefault()
{
    power5VOnState                      = false;
    leftAccumulatorConnected            = false;
    rightAccumulatorConnected           = false;
    mainAccumulatorRelayOnState         = false;
    leftAccumulatorRelayOnState         = false;
    rightAccumulatorRelayOnState        = false;
    power220VACAvailable                = false;
    charger35Ah                         = false;
    charger120Ah                        = false;
    chargingInProgress                  = false;
    chargingComplete                    = false;
    curChannel1Input                    = false;
    leftMotorDutyMax                    = 1;
    rightMotorDutyMax                   = 1;
    leftMotorDuty                       = 1;
    rightMotorDuty                      = 1;
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }
}

void PlatformControlP1::processMessagesQueueWorker()
{
    if (this->controlDeviceIsSet)
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {
                string response = getControlDevice()->pullResponse();

                if (response.compare("DC/DC 5V ENABLED") == 0)
                {
                    setPower5VOnState(true);
                    continue;
                }
                if (response.compare("DC/DC 5V DISABLED") == 0)
                {
                    setPower5VOnState(false);
                    continue;
                }
                if (response.compare("LEFT ACCUMULATOR CONNECTED") == 0)
                {
                    setLeftAccumulatorConnected(true);
                    continue;
                }
                if (response.compare("LEFT ACCUMULATOR DISCONNECTED") == 0)
                {
                    setLeftAccumulatorConnected(false);
                    continue;
                }
                if (response.compare("RIGHT ACCUMULATOR CONNECTED") == 0)
                {
                    setRightAccumulatorConnected(true);
                    continue;
                }
                if (response.compare("RIGHT ACCUMULATOR DISCONNECTED") == 0)
                {
                    setRightAccumulatorConnected(false);
                    continue;
                }
                if (response.find("INPUT1 CHANNEL [8]: ") !=std::string::npos) //charger voltage
                {
                    int substr_pos = response.find(":") + 1;
                    string value_str = response.substr(substr_pos);
                    int value = atoi(value_str.c_str());
                    if (value > 1000)
                    {
                        setPower220VACAvailable(true);
                    }
                    else
                    {
                        setPower220VACAvailable(false);
                    }
                    continue;
                }
            }
            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        getControlDevice()->addMsgToDataExchangeLog("PlatformControlP1 Module process messages queue worker stopped!");
    }
}

void PlatformControlP1::lookFor220VACAvailable()
{
    while (!stopAllProcesses)
    {
        int _curChannel1Input = getCurChannel1Input();
        setCurChannel1Input(8);
        sendCommand("SETINPUT1CHANNEL8");
        sendCommand("GETINPUT1");
        sendCommand(Valter::format_string("SETINPUT1CHANNEL%d", _curChannel1Input));
        setCurChannel1Input(_curChannel1Input);
        this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
int PlatformControlP1::getCurChannel1Input() const
{
    return curChannel1Input;
}

void PlatformControlP1::setCurChannel1Input(int value)
{
    curChannel1Input = value;
}


bool PlatformControlP1::getChargingComplete() const
{
    return chargingComplete;
}

void PlatformControlP1::setChargingComplete(bool value)
{
    chargingComplete = value;
}

bool PlatformControlP1::getChargingInProgress() const
{
    return chargingInProgress;
}

void PlatformControlP1::setChargingInProgress(bool value)
{
    chargingInProgress = value;
}

bool PlatformControlP1::getCharger120Ah() const
{
    return charger120Ah;
}

void PlatformControlP1::setCharger120Ah(bool value)
{
    charger120Ah = value;
}

bool PlatformControlP1::getCharger35Ah() const
{
    return charger35Ah;
}

void PlatformControlP1::setCharger35Ah(bool value)
{
    charger35Ah = value;
}

bool PlatformControlP1::getPower220VACAvailable() const
{
    return power220VACAvailable;
}

void PlatformControlP1::setPower220VACAvailable(bool value)
{
    power220VACAvailable = value;
}

int PlatformControlP1::getRightMotorDutyMax() const
{
    return rightMotorDutyMax;
}

void PlatformControlP1::setRightMotorDutyMax(int value)
{
    rightMotorDutyMax = value;
}

int PlatformControlP1::getLeftMotorDutyMax() const
{
    return leftMotorDutyMax;
}

void PlatformControlP1::setLeftMotorDutyMax(int value)
{
    leftMotorDutyMax = value;
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

int PlatformControlP1::getLeftMotorDuty() const
{
    return leftMotorDuty;
}

void PlatformControlP1::setLeftMotorDuty(int value)
{
    leftMotorDuty = value;
}
