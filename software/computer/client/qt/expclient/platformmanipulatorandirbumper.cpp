#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformmanipulatorandirbumper.h"

#include "platformmanipulatorandirbumper.utils.cpp"
#include "tcphandlers/platformmanipulatorandirbumper.tcphandler.cpp"


PlatformManipulatorAndIRBumper *PlatformManipulatorAndIRBumper::pPlatformManipulatorAndIRBumper = NULL;
bool PlatformManipulatorAndIRBumper::instanceFlag = false;
const string PlatformManipulatorAndIRBumper::controlDeviceId = "PLATFORM-MANIPULATOR-AND-IR-BUMPER";
const string PlatformManipulatorAndIRBumper::defaultsFilePath = "settings/platform-manipulator-and-ir-bumper-defaults";

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
    Valter::log(PlatformManipulatorAndIRBumper::controlDeviceId + " singleton initialized");
    this->controlDeviceIsSet = false;

    initTcpInterface();

    resetToDefault();
    loadDefaults();

    new std::thread(&PlatformManipulatorAndIRBumper::manLink1MovementWorker, this);
    new std::thread(&PlatformManipulatorAndIRBumper::manLink2MovementWorker, this);
    new std::thread(&PlatformManipulatorAndIRBumper::manipulatorReadingsWorker, this);
    new std::thread(&PlatformManipulatorAndIRBumper::irBumperReadingWorker, this);
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

    for (int i = 0; i < 16; i++)
    {
        irBumperTrack[i] = true;
        irBumperTicks[i] = true;
        irBumperTicksReading[i] = 0;
    }

    irBumperInitialized = false;
    irBumperEnabled     = false;

    irBumperFrequency   = 38000;
    irBumperDuty        = 50;
}

void PlatformManipulatorAndIRBumper::setModuleInitialState()
{

}



void PlatformManipulatorAndIRBumper::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&PlatformManipulatorAndIRBumper::processMessagesQueueWorker, this));
    getControlDevice()->addMsgToDataExchangeLog("PlatformManipulatorAndIRBumper Module process messages queue worker started...");
}

void PlatformManipulatorAndIRBumper::initTcpInterface()
{
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(33336);
        setTcpInterface(tcpInterface);
        initTcpCommandAcceptorInterface();
    }
}

void PlatformManipulatorAndIRBumper::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new PlatformManipulatorAndIRBumperTCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
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

                processControlDeviceResponse(response);

                getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~%s", response.c_str()));

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        getControlDevice()->addMsgToDataExchangeLog("PlatformManipulatorAndIRBumper Module process messages queue worker stopped!");
    }
}

void PlatformManipulatorAndIRBumper::processControlDeviceResponse(string response)
{
    if (response.find("24V DCDC Voltage Regulator is ON") != std::string::npos) //24V ON
    {
        setPower24VOn();
        return;
    }
    if (response.find("24V DCDC Voltage Regulator is OFF") != std::string::npos) //24V OFF
    {
        setPower24VOff();
        return;
    }
    if (response.find("LINK1:") != std::string::npos)//LINK1 position (rotation)
    {
        int value_str_pos = response.find_first_of(":") + 1;
        string value_str = response.substr(value_str_pos);
        int  value = atoi(value_str.c_str());
        setLink1ADCPosition(value);
        return;
    }
    if (response.find("LINK2:") != std::string::npos)//LINK2 position (rotation)
    {
        int value_str_pos = response.find_first_of(":") + 1;
        string value_str = response.substr(value_str_pos);
        int  value = atoi(value_str.c_str());
        setLink2ADCPosition(value);
        return;
    }
    if (getLink1CurrentTrack())
    {
        if (response.find("LINK1CURRENT:") != std::string::npos)//LINK1 current
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            int  value = atoi(value_str.c_str());
            setLink1ADCCurrent(value);
            return;
        }
    }
    if (getLink2CurrentTrack())
    {
        if (response.find("LINK2CURRENT:") != std::string::npos)//LINK2 current
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            int  value = atoi(value_str.c_str());
            setLink2ADCCurrent(value);
            return;
        }
    }
    if (getGripperTiltTrack())
    {
        if (response.find("IN1:0") != std::string::npos)//gripper tilt CH0
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setGripperADCTilt(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getGripperRotationTrack())
    {
        if (response.find("IN1:1") != std::string::npos)//gripper rotation CH1
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setGripperADCRotation(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getGripperPositionTrack())
    {
        if (response.find("IN2:2") != std::string::npos)//gripper position CH2
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setGripperADCPosition(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getIRBumperTracked())
    {
        if (response.find("IRB:") != std::string::npos)//IR Bumper response
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setIrBumperTicksReading(atoi(value_str_values[0].c_str()), atoi(value_str_values[3].c_str()));
            return;
        }
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
                }
                setLink1MotorDecelerating(deceleration);
                if (getLink1MotorStop())
                {
                    sendCommand("LINK1STOP");
                    setLink1MotorAccelerating(false);
                    setLink1MotorDecelerating(false);
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
                }
                setLink2MotorDecelerating(deceleration);
                if (getLink2MotorStop())
                {
                    sendCommand("LINK2STOP");
                    setLink2MotorAccelerating(false);
                    setLink2MotorDecelerating(false);
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
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("LINK1POS");
                }
                if (getLink2PositionTrack())
                {
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("LINK2POS");
                }
                if (getLink1CurrentTrack())
                {
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("LINK1CURRENT");
                }
                if (getLink2CurrentTrack())
                {
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("LINK2CURRENT");
                }
                if (getGripperTiltTrack())
                {
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("CH0");
                    this_thread::sleep_for(std::chrono::milliseconds(10));
                    sendCommand("GETIN1");
                }
                if (getGripperRotationTrack())
                {
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("CH1");
                    this_thread::sleep_for(std::chrono::milliseconds(10));
                    sendCommand("GETIN1");
                }
                if (getGripperPositionTrack())
                {
                    this_thread::sleep_for(std::chrono::milliseconds(25));
                    sendCommand("CH2");
                    this_thread::sleep_for(std::chrono::milliseconds(10));
                    sendCommand("GETIN1");
                }
                this_thread::sleep_for(std::chrono::milliseconds(50));
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

void PlatformManipulatorAndIRBumper::irBumperReadingWorker()
{
    while (!stopAllProcesses)
    {
        if (controlDeviceIsSet)
        {
            if (getIrBumperEnabled() && getIrBumperInitialized())
            {
                for (int i = 0; i < 16; i++)
                {
                    if (getIRBumperTrack(i))
                    {
                        this_thread::sleep_for(std::chrono::milliseconds(50));
                        sendCommand(Valter::format_string("IRB%d", i));
                        this_thread::sleep_for(std::chrono::milliseconds(50));
                        sendCommand("IRBGET");
                        this_thread::sleep_for(std::chrono::milliseconds(250));
                    }
                }
            }
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int PlatformManipulatorAndIRBumper::getIrBumperDuty() const
{
    return irBumperDuty;
}

void PlatformManipulatorAndIRBumper::setIrBumperDuty(int value)
{
    irBumperDuty = value;
    sendCommand(Valter::format_string("SETIRBUMPERTRDUTY#%d", value));
}

void PlatformManipulatorAndIRBumper::setIrBumperTicksReading(int idx, int value)
{
    irBumperTicksReading[idx] = value;
}

int PlatformManipulatorAndIRBumper::getIrBumperTicksReading(int idx)
{
    return irBumperTicksReading[idx];
}

int PlatformManipulatorAndIRBumper::getIrBumperFrequency() const
{
    return irBumperFrequency;
}

void PlatformManipulatorAndIRBumper::setIrBumperFrequency(int value)
{
    irBumperFrequency = value;
    sendCommand(Valter::format_string("IRBUMPERFREQ#%d", value));
}

bool PlatformManipulatorAndIRBumper::getIrBumperEnabled() const
{
    return irBumperEnabled;
}

void PlatformManipulatorAndIRBumper::setIrBumperEnabled(bool value)
{
    irBumperEnabled = value;
    if (value)
    {
        sendCommand("IRBUMPERENABLE");
    }
    else
    {
        sendCommand("IRBUMPERDISABLE");
    }
}

bool PlatformManipulatorAndIRBumper::getIrBumperInitialized() const
{
    return irBumperInitialized;
}

void PlatformManipulatorAndIRBumper::setIrBumperInitialized(bool value)
{
    irBumperInitialized = value;
    if (value)
    {
        sendCommand("IRBUMPERINIT");
    }
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
