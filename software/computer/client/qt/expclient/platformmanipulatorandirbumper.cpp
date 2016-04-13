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
    resetToDefault();
    loadDefaults();
    this->controlDeviceIsSet = false;

    new std::thread(&PlatformManipulatorAndIRBumper::manLink1MovementWorker, this);
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

    power24VOnOff           = false;

    //link1
    link1MovementDirection  = false;
    link1MotorActivated     = false;
    link1MotorStop          = true;

    link1MotorDuty          = 1;
    link1MotorDutyMax       = 30;
    link1MotorAcceleration  = 2;
    link1MotorDeceleration  = 2;

    link1MotorAccelerating  = false;
    link1MotorDecelerating  = false;

    link1MotorDutyPresetCur = 30;
    link1MotorDutyPresetMin = 1;
    link1MotorDutyPresetMax = 1;

    //link2
    link2MovementDirection  = false;
    link2MotorActivated     = false;
    link2MotorStop          = false;

    link2MotorDuty          = 1;
    link2MotorDutyMax       = 30;
    link2MotorAcceleration  = 2;
    link2MotorDeceleration  = 2;

    link2MotorDutyPresetCur = false;
    link2MotorDutyPresetMin = false;
    link2MotorDutyPresetMax = false;
}

void PlatformManipulatorAndIRBumper::setModuleInitialState()
{

}

void PlatformManipulatorAndIRBumper::loadDefaults()
{
    ifstream defaultsFile(PlatformManipulatorAndIRBumper::defaultsFilePath);
    string line;
    while (getline(defaultsFile, line, '\n'))
    {
        if (line.substr(0, 2).compare("//") != 0)
        {
            char *lineStrPtr = Valter::stringToCharPtr(line);
            string defaultValueName(strtok(lineStrPtr, ":" ));
            string defaultValue(strtok(NULL, ":" ));
            addDefault(defaultValueName, defaultValue);
        }
    }
    defaultsFile.close();

    string defaultValue;
    char *defaultValuePtr;
    int curValue;
    int minValue;
    int maxValue;

    //link1MotorDuty
    defaultValue = getDefault("link1MotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLink1MotorDutyPresetMin(minValue);
    setLink1MotorDutyPresetMax(maxValue);
    setLink1MotorDutyPresetCur(curValue);
    setLink1MotorDutyMax(getLink1MotorDutyPresetMax());

    //link1MotorAcceleration
    defaultValue = getDefault("link1MotorAcceleration");
    link1MotorAcceleration = stof(Valter::stringToCharPtr(defaultValue));

    //link1MotorDeceleration
    defaultValue = getDefault("link1MotorDeceleration");
    link1MotorDeceleration = stof(Valter::stringToCharPtr(defaultValue));

    //link2MotorDuty
    defaultValue = getDefault("link2MotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLink2MotorDutyPresetMin(minValue);
    setLink2MotorDutyPresetMax(maxValue);
    setLink2MotorDutyPresetCur(curValue);
    setLink2MotorDutyMax(getLink2MotorDutyPresetMax());

    //link2MotorAcceleration
    defaultValue = getDefault("link2MotorAcceleration");
    link2MotorAcceleration = stof(Valter::stringToCharPtr(defaultValue));

    //link2MotorDeceleration
    defaultValue = getDefault("link2MotorDeceleration");
    link2MotorDeceleration = stof(Valter::stringToCharPtr(defaultValue));
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
                string response = getControlDevice()->pullResponse();

                if (response.find("24V DCDC Voltage Regulator is ON") != std::string::npos) //24V ON
                {
                    setPower24VOn();
                    continue;
                }
                if (response.find("24V DCDC Voltage Regulator is OFF") != std::string::npos) //24V OFF
                {
                    setPower24VOff();
                    continue;
                }
            }
            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        getControlDevice()->addMsgToDataExchangeLog("PlatformManipulatorAndIRBumper Module process messages queue worker stopped!");
    }
}

void PlatformManipulatorAndIRBumper::manLink1MovementWorker()
{
    while (!stopAllProcesses)
    {
        if (!getLink1MotorStop())
        {
            int curLink1MotorDuty = getLink1MotorDuty();
            bool acceleration, deceleration;
            if (getLink1MotorActivated())
            {
                acceleration = getLink1MotorAccelerating();
                if (curLink1MotorDuty + getLink1MotorAcceleration() < getLink1MotorDutyMax())
                {
                    curLink1MotorDuty += getLink1MotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curLink1MotorDuty = getLink1MotorDutyMax();
                    acceleration = false;
                }
                if (getLink1MotorAccelerating())
                {
                    setLink1MotorDuty(curLink1MotorDuty);
                    sendCommand(Valter::format_string("SETLINK1DRIVEDUTY#%d", curLink1MotorDuty));
                }
                setLink1MotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getLink1MotorDecelerating();
                if (curLink1MotorDuty - getLink1MotorDeceleration() > 1)
                {
                    curLink1MotorDuty -= getLink1MotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curLink1MotorDuty = 1;
                    deceleration = false;
                    setLink1MotorStop(true);
                }

                if (getLink1MotorDecelerating())
                {
                    setLink1MotorDuty(curLink1MotorDuty);
                    sendCommand(Valter::format_string("SETLINK1DRIVEDUTY#%d", curLink1MotorDuty));
                }
                setLink1MotorDecelerating(deceleration);
                if (getLink1MotorStop())
                {
                    sendCommand("LINK1STOP");
                }
            }
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool PlatformManipulatorAndIRBumper::getLink1MotorDecelerating() const
{
    return link1MotorDecelerating;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDecelerating(bool value)
{
    link1MotorDecelerating = value;
}

bool PlatformManipulatorAndIRBumper::getLink1MotorAccelerating() const
{
    return link1MotorAccelerating;
}

void PlatformManipulatorAndIRBumper::setLink1MotorAccelerating(bool value)
{
    link1MotorAccelerating = value;
}

bool PlatformManipulatorAndIRBumper::getPower24VOnOff() const
{
    return power24VOnOff;
}

void PlatformManipulatorAndIRBumper::setPower24VOnOff(bool value)
{
    if (value)
    {
        sendCommand("DCDC24VENABLEON");
    }
    else
    {
        sendCommand("DCDC24VENABLEOFF");
    }
}

void PlatformManipulatorAndIRBumper::setPower24VOn()
{
    power24VOnOff = true;
}

void PlatformManipulatorAndIRBumper::setPower24VOff()
{
    power24VOnOff = false;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyPresetMax() const
{
    return link2MotorDutyPresetMax;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyPresetMax(int value)
{
    link2MotorDutyPresetMax = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyPresetMin() const
{
    return link2MotorDutyPresetMin;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyPresetMin(int value)
{
    link2MotorDutyPresetMin = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyPresetCur() const
{
    return link2MotorDutyPresetCur;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyPresetCur(int value)
{
    link2MotorDutyPresetCur = value;
}

bool PlatformManipulatorAndIRBumper::getLink2MotorStop() const
{
    return link2MotorStop;
}

void PlatformManipulatorAndIRBumper::setLink2MotorStop(bool value)
{
    link2MotorStop = value;
}

bool PlatformManipulatorAndIRBumper::getLink2MotorActivated() const
{
    return link2MotorActivated;
}

void PlatformManipulatorAndIRBumper::setLink2MotorActivated(bool value)
{
    link2MotorActivated = value;
}

bool PlatformManipulatorAndIRBumper::getLink2MovementDirection() const
{
    return link2MovementDirection;
}

void PlatformManipulatorAndIRBumper::setLink2MovementDirection(bool value)
{
    link2MovementDirection = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDeceleration() const
{
    return link2MotorDeceleration;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDeceleration(int value)
{
    link2MotorDeceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorAcceleration() const
{
    return link2MotorAcceleration;
}

void PlatformManipulatorAndIRBumper::setLink2MotorAcceleration(int value)
{
    link2MotorAcceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyMax() const
{
    return link2MotorDutyMax;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyMax(int value)
{
    link2MotorDutyMax = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDuty() const
{
    return link2MotorDuty;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDuty(int value)
{
    link2MotorDuty = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyPresetMax() const
{
    return link1MotorDutyPresetMax;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyPresetMax(int value)
{
    link1MotorDutyPresetMax = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyPresetMin() const
{
    return link1MotorDutyPresetMin;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyPresetMin(int value)
{
    link1MotorDutyPresetMin = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyPresetCur() const
{
    return link1MotorDutyPresetCur;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyPresetCur(int value)
{
    link1MotorDutyPresetCur = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDeceleration() const
{
    return link1MotorDeceleration;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDeceleration(int value)
{
    link1MotorDeceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorAcceleration() const
{
    return link1MotorAcceleration;
}

void PlatformManipulatorAndIRBumper::setLink1MotorAcceleration(int value)
{
    link1MotorAcceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyMax() const
{
    return link1MotorDutyMax;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyMax(int value)
{
    link1MotorDutyMax = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDuty() const
{
    return link1MotorDuty;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDuty(int value)
{
    link1MotorDuty = value;
}

bool PlatformManipulatorAndIRBumper::getLink1MotorStop() const
{
    return link1MotorStop;
}

void PlatformManipulatorAndIRBumper::setLink1MotorStop(bool value)
{
    link1MotorStop = value;
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
    if (value)//if activated motor is not stopped
    {
        setLink1MotorStop(false);
    }
}

bool PlatformManipulatorAndIRBumper::getLink1MovementDirection() const
{
    return link1MovementDirection;
}

bool PlatformManipulatorAndIRBumper::setLink1MovementDirection(bool value)
{
    if (getLink1MotorStop())
    {
        link1MovementDirection = value;
        if (link1MovementDirection) //up
        {
            sendCommand("LINK1GETUP");
        }
        else
        {
            sendCommand("LINK1GETDOWN");
        }
        return true;
    }
    else
    {
        if (getLink1MovementDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

string PlatformManipulatorAndIRBumper::getControlDeviceId()
{
    return controlDeviceId;
}
