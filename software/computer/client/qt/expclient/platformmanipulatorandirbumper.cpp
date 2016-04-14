#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformmanipulatorandirbumper.h"

#include "platformmanipulatorandirbumper.code.utils.cpp"

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

    resetToDefault();
    loadDefaults();

    new std::thread(&PlatformManipulatorAndIRBumper::manLink1MovementWorker, this);
    new std::thread(&PlatformManipulatorAndIRBumper::manLink2MovementWorker, this);
    new std::thread(&PlatformManipulatorAndIRBumper::manipulatorReadingsWorker, this);
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

    link2MotorAccelerating  = false;
    link2MotorDecelerating  = false;

    link2MotorDutyPresetCur = false;
    link2MotorDutyPresetMin = false;
    link2MotorDutyPresetMax = false;

    manGripperRotationMotorDuty         = 1;

    link1PositionTrack      = true;
    link1PositionADC        = true;
    link2PositionTrack      = true;
    link2PositionADC        = true;
    link1CurrentTrack       = true;
    link1CurrentADC         = true;
    link2CurrentTrack       = true;
    link2CurrentADC         = true;

    link1Position   = 0;
    link2Position   = 0;
    link1Current    = 0;
    link2Current    = 0;

    link1ADCPosition    = 0;
    link2ADCPosition    = 0;
    link1ADCCurrent     = 0;
    link2ADCCurrent     = 0;
}

void PlatformManipulatorAndIRBumper::setModuleInitialState()
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
                if (response.find("LINK1:") != std::string::npos)//LINK1 position (rotation)
                {
                    int value_str_pos = response.find_first_of(":") + 1;
                    string value_str = response.substr(value_str_pos);
                    int  value = atoi(value_str.c_str());
                    setLink1ADCPosition(value);
                }
                if (response.find("LINK2:") != std::string::npos)//LINK2 position (rotation)
                {
                    int value_str_pos = response.find_first_of(":") + 1;
                    string value_str = response.substr(value_str_pos);
                    int  value = atoi(value_str.c_str());
                    setLink2ADCPosition(value);
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

void PlatformManipulatorAndIRBumper::manLink2MovementWorker()
{
    while (!stopAllProcesses)
    {
        if (!getLink2MotorStop())
        {
            int curLink2MotorDuty = getLink2MotorDuty();
            bool acceleration, deceleration;
            if (getLink2MotorActivated())
            {
                acceleration = getLink2MotorAccelerating();
                if (curLink2MotorDuty + getLink2MotorAcceleration() < getLink2MotorDutyMax())
                {
                    curLink2MotorDuty += getLink2MotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curLink2MotorDuty = getLink2MotorDutyMax();
                    acceleration = false;
                }
                if (getLink2MotorAccelerating())
                {
                    setLink2MotorDuty(curLink2MotorDuty);
                    sendCommand(Valter::format_string("SETLINK2DRIVEDUTY#%d", curLink2MotorDuty));
                }
                setLink2MotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getLink2MotorDecelerating();
                if (curLink2MotorDuty - getLink2MotorDeceleration() > 1)
                {
                    curLink2MotorDuty -= getLink2MotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curLink2MotorDuty = 1;
                    deceleration = false;
                    setLink2MotorStop(true);
                }

                if (getLink2MotorDecelerating())
                {
                    setLink2MotorDuty(curLink2MotorDuty);
                    sendCommand(Valter::format_string("SETLINK2DRIVEDUTY#%d", curLink2MotorDuty));
                }
                setLink2MotorDecelerating(deceleration);
                if (getLink2MotorStop())
                {
                    sendCommand("LINK2STOP");
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

void PlatformManipulatorAndIRBumper::manipulatorReadingsWorker()
{
    while (!stopAllProcesses)
    {
        if (controlDeviceIsSet)
        {
            if (getControlDevice()->getStatus() == ControlDevice::StatusActive)
            {
                if (getLink1PositionTrack())
                {
                    sendCommand("LINK1POS");
                }
                if (getLink2PositionTrack())
                {
                    sendCommand("LINK2POS");
                }
                this_thread::sleep_for(std::chrono::milliseconds(25));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

int PlatformManipulatorAndIRBumper::getLink2ADCCurrent() const
{
    return link2ADCCurrent;
}

void PlatformManipulatorAndIRBumper::setLink2ADCCurrent(int value)
{
    link2ADCCurrent = value;
}

int PlatformManipulatorAndIRBumper::getLink1ADCCurrent() const
{
    return link1ADCCurrent;
}

void PlatformManipulatorAndIRBumper::setLink1ADCCurrent(int value)
{
    link1ADCCurrent = value;
}

int PlatformManipulatorAndIRBumper::getLink2ADCPosition() const
{
    return link2ADCPosition;
}

void PlatformManipulatorAndIRBumper::setLink2ADCPosition(int value)
{
    link2ADCPosition = value;
}

int PlatformManipulatorAndIRBumper::getLink1ADCPosition() const
{
    return link1ADCPosition;
}

void PlatformManipulatorAndIRBumper::setLink1ADCPosition(int value)
{
    link1ADCPosition = value;
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

bool PlatformManipulatorAndIRBumper::setLink2MovementDirection(bool value)
{
    if (getLink2MotorStop())
    {
        link2MovementDirection = value;
        if (link2MovementDirection) //up
        {
            sendCommand("LINK2GETUP");
        }
        else
        {
            sendCommand("LINK2GETDOWN");
        }
        return true;
    }
    else
    {
        if (getLink2MovementDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

void PlatformManipulatorAndIRBumper::manLink3TiltUp()
{
    sendCommand("GRIPPERTILTGETUP");
}

void PlatformManipulatorAndIRBumper::manLink3TiltDown()
{
    sendCommand("GRIPPERTILTGETDOWN");
}

void PlatformManipulatorAndIRBumper::manLink3Stop()
{
    sendCommand("GRIPPERTILTSTOP");
}

void PlatformManipulatorAndIRBumper::manGripperOpen()
{
    sendCommand("GRIPPERPOSITIONOPEN");
}

void PlatformManipulatorAndIRBumper::manGripperClose()
{
    sendCommand("GRIPPERPOSITIONCLOSE");
}

void PlatformManipulatorAndIRBumper::manGripperStop()
{
    sendCommand("GRIPPERPOSITIONSTOP");
}

void PlatformManipulatorAndIRBumper::manGripperRotateCW()
{
    sendCommand(Valter::format_string("SETGRIPPERROTATEDRIVEDUTY#%d", getManGripperRotationMotorDuty()));
    sendCommand("GRIPPERROTATECW");
}

void PlatformManipulatorAndIRBumper::manGripperRotateCCW()
{
    sendCommand(Valter::format_string("SETGRIPPERROTATEDRIVEDUTY#%d", getManGripperRotationMotorDuty()));
    sendCommand("GRIPPERROTATECCW");
}

void PlatformManipulatorAndIRBumper::manGripperRotateStop()
{
    sendCommand("GRIPPERROTATESTOP");
}
