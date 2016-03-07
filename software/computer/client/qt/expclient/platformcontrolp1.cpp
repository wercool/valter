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
    new std::thread(&PlatformControlP1::scanFor220VACAvailable, this);
    chargerButtonPressStep = 0;
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
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

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
    scan220ACAvailable                  = false;
    chargerConnected                    = false;

    curChannel1Input                    = 0;
    curChannel2Input                    = 0;
    leftMotorDutyMax                    = 1;
    rightMotorDutyMax                   = 1;
    leftMotorDuty                       = 1;
    rightMotorDuty                      = 1;

    chargerVoltageADC                   = 0;
    chargerVoltageVolts                 = 0.0;

    if (chargerButtonPressStep != 0)
    {
        setChargerMode(false);
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
                    setChargerVoltageADC(value);
                    continue;
                }
                if (response.compare("MAIN ACCUMULATOR RELAY SET ON") == 0)
                {
                    setMainAccumulatorRelayOnState(true);
                    continue;
                }
                if (response.compare("MAIN ACCUMULATOR RELAY SET OFF") == 0)
                {
                    setMainAccumulatorRelayOnState(false);
                    continue;
                }
                if (response.compare("LEFT ACCUMULATOR RELAY SET ON") == 0)
                {
                    setLeftAccumulatorRelayOnState(true);
                    continue;
                }
                if (response.compare("LEFT ACCUMULATOR RELAY SET OFF") == 0)
                {
                    setLeftAccumulatorRelayOnState(false);
                    continue;
                }
                if (response.compare("RIGHT ACCUMULATOR RELAY SET ON") == 0)
                {
                    setRightAccumulatorRelayOnState(true);
                    continue;
                }
                if (response.compare("RIGHT ACCUMULATOR RELAY SET OFF") == 0)
                {
                    setRightAccumulatorRelayOnState(false);
                    continue;
                }
                if (response.find("INPUT2 CHANNEL [8]: ") !=std::string::npos) //charger connected
                {
                    int substr_pos = response.find(":") + 1;
                    string value_str = response.substr(substr_pos);
                    int value = atoi(value_str.c_str());
                    if (value > 1000)
                    {
                        setChargerConnected(true);
                    }
                    else
                    {
                        setChargerConnected(false);
                    }
                    continue;
                }
                if (response.find("INPUT2 CHANNEL [9]: ") !=std::string::npos) //14.4V / 0.8A 1.2-35Ah
                {
                    int substr_pos = response.find(":") + 1;
                    string value_str = response.substr(substr_pos);
                    int value = atoi(value_str.c_str());
                    if (value > 1000)
                    {
                        setCharger35Ah(true);
                    }
                    else
                    {
                        setCharger35Ah(false);
                    }
                    continue;
                }
                if (response.find("INPUT2 CHANNEL [10]: ") !=std::string::npos) //14.4V / 3.6A 35-120Ah
                {
                    int substr_pos = response.find(":") + 1;
                    string value_str = response.substr(substr_pos);
                    int value = atoi(value_str.c_str());
                    if (value > 1000)
                    {
                        setCharger120Ah(true);
                    }
                    else
                    {
                        setCharger120Ah(false);
                    }
                    continue;
                }
                if (response.find("INPUT2 CHANNEL [6]: ") !=std::string::npos) //Charging in progress
                {
                    int substr_pos = response.find(":") + 1;
                    string value_str = response.substr(substr_pos);
                    int value = atoi(value_str.c_str());
                    if (value > 600)
                    {
                        setChargingInProgress(true);
                    }
                    else
                    {
                        setChargingInProgress(false);
                    }
                    continue;
                }
                if (response.find("INPUT2 CHANNEL [7]: ") !=std::string::npos) //Charging complete
                {
                    int substr_pos = response.find(":") + 1;
                    string value_str = response.substr(substr_pos);
                    int value = atoi(value_str.c_str());
                    if (value > 700)
                    {
                        setChargingComplete(true);
                    }
                    else
                    {
                        setChargingComplete(false);
                    }
                    continue;
                }
            }
            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        getControlDevice()->addMsgToDataExchangeLog("PlatformControlP1 Module process messages queue worker stopped!");
    }
}

void PlatformControlP1::scanFor220VACAvailable()
{
    while (!stopAllProcesses)
    {
        if (getScan220ACAvailable())
        {
            int _curChannel1Input = getCurChannel1Input();
            int _curChannel2Input = getCurChannel2Input();

            setCurChannel1Input(8);
            sendCommand("SETINPUT1CHANNEL8");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            setCurChannel2Input(8);
            sendCommand("SETINPUT2CHANNEL8");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            sendCommand("GETINPUT2");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            setCurChannel2Input(9);
            sendCommand("SETINPUT2CHANNEL9");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            sendCommand("GETINPUT2");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            setCurChannel2Input(10);
            sendCommand("SETINPUT2CHANNEL10");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            sendCommand("GETINPUT2");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            setCurChannel2Input(6);
            sendCommand("SETINPUT2CHANNEL6");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            sendCommand("GETINPUT2");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            setCurChannel2Input(7);
            sendCommand("SETINPUT2CHANNEL7");
            this_thread::sleep_for(std::chrono::milliseconds(100));
            sendCommand("GETINPUT2");
            this_thread::sleep_for(std::chrono::milliseconds(100));

            sendCommand(Valter::format_string("SETINPUT1CHANNEL%d", _curChannel1Input));
            setCurChannel1Input(_curChannel1Input);
            sendCommand(Valter::format_string("SETINPUT2CHANNEL%d", _curChannel2Input));
            setCurChannel2Input(_curChannel2Input);
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PlatformControlP1::chargerModeSetting()
{
    if (!getChargerMode())
    {
        while (getChargerButtonPressStep() != 0)
        {
            chargerButtonPress();
            this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    else
    {
        while (getChargerButtonPressStep() != 2)
        {
            chargerButtonPress();
            this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}
bool PlatformControlP1::getChargerMode() const
{
    return chargerMode;
}

void PlatformControlP1::setChargerMode(bool value)
{
    chargerMode = value;
    new std::thread(&PlatformControlP1::chargerModeSetting, this);
}

int PlatformControlP1::getChargerButtonPressStep() const
{
    return chargerButtonPressStep;
}

void PlatformControlP1::setChargerButtonPressStep(int value)
{
    chargerButtonPressStep = value;
}

bool PlatformControlP1::getChargerConnected() const
{
    return chargerConnected;
}

void PlatformControlP1::setChargerConnected(bool value)
{
    chargerConnected = value;
}

void PlatformControlP1::chargerButtonPress()
{
    if (controlDeviceIsSet)
    {
        if (getControlDevice()->getControlDevicePort()->isOpen())
        {
            sendCommand("CHARGERBUTTONPRESS");
            if (chargerButtonPressStep + 1 < 4)
            {
                chargerButtonPressStep++;
            }
            else
            {
                chargerButtonPressStep = 0;
            }
        }
    }
}

bool PlatformControlP1::mainAccumulatorON()
{
    if (getPower220VACAvailable())
    {
        sendCommand("MAINACCUMULATORRELAYON");
        return true;
    }
    else
    {
        return false;
    }
}

int PlatformControlP1::getCurChannel2Input() const
{
    return curChannel2Input;
}

void PlatformControlP1::setCurChannel2Input(int value)
{
    curChannel2Input = value;
}

float PlatformControlP1::getChargerVoltageVolts() const
{
    return chargerVoltageVolts;
}

void PlatformControlP1::setChargerVoltageVolts(float value)
{
    chargerVoltageVolts = value;
}

int PlatformControlP1::getChargerVoltageADC() const
{
    return chargerVoltageADC;
}

void PlatformControlP1::setChargerVoltageADC(int value)
{
    chargerVoltageADC = value;
    setChargerVoltageVolts(value * 0.1);
}

bool PlatformControlP1::getScan220ACAvailable() const
{
    return scan220ACAvailable;
}

void PlatformControlP1::setScan220ACAvailable(bool value)
{
    scan220ACAvailable = value;
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
