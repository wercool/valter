#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "bodycontrolp1.h"

#include "bodycontrolp1.utils.cpp"
#include "tcphandlers/bodycontrolp1.tcphandler.cpp"

BodyControlP1 *BodyControlP1::pBodyControlP1 = NULL;
bool BodyControlP1::instanceFlag = false;
const string BodyControlP1::controlDeviceId = "BODY-CONTROL-P1";
const string BodyControlP1::defaultsFilePath = "settings/body-control-p1-defaults";

BodyControlP1::BodyControlP1()
{
    Valter::log(BodyControlP1::controlDeviceId + " singleton initialized");
    this->controlDeviceIsSet = false;

    initTcpInterface();

    resetToDefault();
    loadDefaults();

    new std::thread(&BodyControlP1::headYawWorker, this);
    new std::thread(&BodyControlP1::headPitchWorker, this);
    new std::thread(&BodyControlP1::bodyPitchWorker, this);
    new std::thread(&BodyControlP1::rightArmYawWorker, this);
    new std::thread(&BodyControlP1::leftArmYawWorker, this);
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

void BodyControlP1::setModuleInitialState()
{

}

void BodyControlP1::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&BodyControlP1::processMessagesQueueWorker, this));
    getControlDevice()->addMsgToDataExchangeLog("BodyControlP1 Module process messages queue worker started...");
}

void BodyControlP1::initTcpInterface()
{
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(33337);
        setTcpInterface(tcpInterface);
        initTcpCommandAcceptorInterface();
    }
}

void BodyControlP1::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new BodyControlP1TCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
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

                processControlDeviceResponse(response);

                getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~%s", response.c_str()));

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void BodyControlP1::processControlDeviceResponse(string response)
{
    if (response.compare("HEAD 24V ON") == 0)
    {
        setHead24VState(true);
        return;
    }
    if (response.compare("HEAD 24V OFF") == 0)
    {
        setHead24VState(false);
        return;
    }
    if (response.compare("HEAD YAW MOTOR ENABLED") == 0)
    {
        setHeadYawMotorState(true);
        return;
    }
    if (response.compare("HEAD YAW MOTOR DISABLED") == 0)
    {
        setHeadYawMotorState(false);
        return;
    }
    if (response.compare("HEAD PITCH MOTOR ENABLED") == 0)
    {
        setHeadPitchMotorState(true);
        return;
    }
    if (response.compare("HEAD PITCH MOTOR DISABLED") == 0)
    {
        setHeadPitchMotorState(false);
        return;
    }
    if (getBodyPitchPositionTrack())
    {
        if (response.find("BP:") != std::string::npos) //body pitch position
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setBodyPitchADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getRightArmYawPositionTrack())
    {
        if (response.find("RAY:") != std::string::npos) //right arm yaw position
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setRightArmYawADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getLeftArmYawPositionTrack())
    {
        if (response.find("LAY:") != std::string::npos) //left arm yaw position
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setLeftArmYawADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getHeadPitchPositionTrack())
    {
        if (response.find("HPP:") != std::string::npos) //head pitch position
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setHeadPitchADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getHeadYawPositionTrack())
    {
        if (response.find("HYP:") != std::string::npos) //head yaw position
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setHeadYawADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getBodyPitchCurrentTrack())
    {
        if (response.find("CH:2") != std::string::npos) //Body pitch current
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setBodyPitchADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getRightArmYawCurrentTrack())
    {
        if (response.find("CH:1") != std::string::npos) //right arm current
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setRightArmYawADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getLeftArmYawCurrentTrack())
    {
        if (response.find("CH:0") != std::string::npos) //left arm current
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setLeftArmYawADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (response.compare("5V5 SOURCE ON") == 0) //5.5V power source on
    {
        setPowerSource5V5State(true);
        return;
    }
    if (response.compare("5V5 SOURCE OFF") == 0) //5.5V power source off
    {
        setPowerSource5V5State(false);
        return;
    }
    if (response.compare("WIFI ON") == 0) //wifi 12V power source on
    {
        setWifiPowerState(true);
        return;
    }
    if (response.compare("WIFI OFF") == 0) //wifi 12V power source off
    {
        setWifiPowerState(false);
        return;
    }
    if (response.compare("LEFT ARM 24V ON") == 0) //left arm 24V power source on
    {
        setLeftArm24VPowerSourceState(true);
        return;
    }
    if (response.compare("LEFT ARM 24V OFF") == 0) //left arm 24V power source off
    {
        setLeftArm24VPowerSourceState(false);
        return;
    }
    if (response.compare("RIGHT ARM 24V ON") == 0) //right arm 24V power source on
    {
        setRightArm24VPowerSourceState(true);
        return;
    }
    if (response.compare("RIGHT ARM 24V OFF") == 0) //right arm 24V power source off
    {
        setRightArm24VPowerSourceState(false);
        return;
    }
    if (response.compare("RIGHT ACCUMULATOR ON") == 0) //right accumulator connected
    {
        setRightAccumulatorConnectedState(true);
        return;
    }
    if (response.compare("RIGHT ACCUMULATOR OFF") == 0) //right accumulator disconnected
    {
        setRightAccumulatorConnectedState(false);
        return;
    }
    if (response.compare("LEFT ACCUMULATOR ON") == 0) //left accumulator connected
    {
        setLeftAccumulatorConnectedState(true);
        return;
    }
    if (response.compare("LEFT ACCUMULATOR OFF") == 0) //left accumulator disconnected
    {
        setLeftAccumulatorConnectedState(false);
        return;
    }
    if (response.compare("LEFT ARM 12V ON") == 0) //left arm 12V power source on
    {
        setLeftArm12VPowerSourceState(true);
        return;
    }
    if (response.compare("LEFT ARM 12V OFF") == 0) //left arm 12V power source off
    {
        setLeftArm12VPowerSourceState(false);
        return;
    }
    if (response.compare("RIGHT ARM 12V ON") == 0) //right arm 12V power source on
    {
        setRightArm12VPowerSourceState(true);
        return;
    }
    if (response.compare("RIGHT ARM 12V OFF") == 0) //right arm 12V power source off
    {
        setRightArm12VPowerSourceState(false);
        return;
    }
    if (response.compare("KINECT1 ON") == 0) //kinect 1 on
    {
        setKinect1PowerState(true);
        return;
    }
    if (response.compare("KINECT1 OFF") == 0) //kinect 1 off
    {
        setKinect1PowerState(false);
        return;
    }
    if (response.compare("KINECT2 ON") == 0) //kinect 2 on
    {
        setKinect2PowerState(true);
        return;
    }
    if (response.compare("KINECT2 OFF") == 0) //kinect 2 off
    {
        setKinect2PowerState(false);
        return;
    }
}

unsigned int BodyControlP1::executeTask(string taskScriptLine)
{

}

void BodyControlP1::headYawWorker()
{
    while (!stopAllProcesses)
    {
        if (getHeadYawMotorActivated())
        {
            sendCommand(Valter::format_string("HEADYAW#%d", getHeadYawStepSwitchDelay()));
            int headYawStepPostion = getHeadYawStepPostion();
            if (getHeadYawDirection())
            {
                setHeadYawStepPostion(++headYawStepPostion);
            }
            else
            {
                setHeadYawStepPostion(--headYawStepPostion);
            }
            this_thread::sleep_for(std::chrono::microseconds(getHeadYawStepDelay()));
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
        if (getHeadPitchMotorActivated())
        {
            sendCommand(Valter::format_string("HEADPITCH#%d", getHeadPitchStepSwitchDelay()));
            int headPitchStepPostion = getHeadPitchStepPosition();
            if (getHeadPitchDirection())
            {
                setHeadPitchStepPosition(++headPitchStepPostion);
            }
            else
            {
                setHeadPitchStepPosition(--headPitchStepPostion);
            }
            this_thread::sleep_for(std::chrono::microseconds(getHeadPitchStepDelay()));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void BodyControlP1::bodyPitchWorker()
{
    while (!stopAllProcesses)
    {
        if (!getBodyPitchMotorStop())
        {
            if (getBodyPitchPositionTrack())
            {
                sendCommand("GETBODYPITCH");
            }
            int curBodyPitchMotorDuty = getBodyPitchMotorDuty();
            bool acceleration, deceleration;
            if (getBodyPitchMotorActivated())
            {
                acceleration = getBodyPitchMotorAccelerating();
                if (curBodyPitchMotorDuty + getBodyPitchMotorAcceleration() < getBodyPitchMotorDutyMax())
                {
                    curBodyPitchMotorDuty += getBodyPitchMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curBodyPitchMotorDuty = getBodyPitchMotorDutyMax();
                    acceleration = false;
                }
                if (getBodyPitchMotorAccelerating())
                {
                    setBodyPitchMotorDuty(curBodyPitchMotorDuty);
                }
                setBodyPitchMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getBodyPitchMotorDecelerating();
                if (curBodyPitchMotorDuty - getBodyPitchMotorDeceleration() > 1)
                {
                    curBodyPitchMotorDuty -= getBodyPitchMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curBodyPitchMotorDuty = 1;
                    deceleration = false;
                    setBodyPitchMotorStop(true);
                }

                if (getBodyPitchMotorDecelerating())
                {
                    setBodyPitchMotorDuty(curBodyPitchMotorDuty);
                }
                setBodyPitchMotorDecelerating(deceleration);
                if (getBodyPitchMotorStop())
                {
                    sendCommand("BODYPITCHSTOP");
                    setBodyPitchMotorAccelerating(false);
                    setBodyPitchMotorDecelerating(false);
                }
            }
            if (getBodyPitchCurrentTrack())
            {
                sendCommand("CHANNEL2");
                sendCommand("GETCHREAD");
            }
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void BodyControlP1::rightArmYawWorker()
{
    while (!stopAllProcesses)
    {
        if (!getRightArmYawMotorStop())
        {
            if (getRightArmYawPositionTrack())
            {
                sendCommand("GETRIGHTARMYAW");
            }
            int curRightArmYawMotorDuty = getRightArmYawMotorDuty();
            bool acceleration, deceleration;
            if (getRightArmYawMotorActivated())
            {
                acceleration = getRightArmYawMotorAccelerating();
                if (curRightArmYawMotorDuty + getRightArmYawMotorAcceleration() < getRightArmYawMotorDutyMax())
                {
                    curRightArmYawMotorDuty += getRightArmYawMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curRightArmYawMotorDuty = getRightArmYawMotorDutyMax();
                    acceleration = false;
                }
                if (getRightArmYawMotorAccelerating())
                {
                    setRightArmYawMotorDuty(curRightArmYawMotorDuty);
                }
                setRightArmYawMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getRightArmYawMotorDecelerating();
                if (curRightArmYawMotorDuty - getRightArmYawMotorDeceleration() > 1)
                {
                    curRightArmYawMotorDuty -= getRightArmYawMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curRightArmYawMotorDuty = 1;
                    deceleration = false;
                    setRightArmYawMotorStop(true);
                }

                if (getRightArmYawMotorDecelerating())
                {
                    setRightArmYawMotorDuty(curRightArmYawMotorDuty);
                }
                setRightArmYawMotorDecelerating(deceleration);
                if (getRightArmYawMotorStop())
                {
                    sendCommand("RIGHTARMYAWSTOP");
                    setRightArmYawMotorAccelerating(false);
                    setRightArmYawMotorDecelerating(false);
                }
            }
            if (getRightArmYawCurrentTrack())
            {
                sendCommand("CHANNEL1");
                sendCommand("GETCHREAD");
            }
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void BodyControlP1::leftArmYawWorker()
{
    while (!stopAllProcesses)
    {
        if (!getLeftArmYawMotorStop())
        {
            if (getLeftArmYawPositionTrack())
            {
                sendCommand("GETLEFTARMYAW");
            }
            int curLeftArmYawMotorDuty = getLeftArmYawMotorDuty();
            bool acceleration, deceleration;
            if (getLeftArmYawMotorActivated())
            {
                acceleration = getLeftArmYawMotorAccelerating();
                if (curLeftArmYawMotorDuty + getLeftArmYawMotorAcceleration() < getLeftArmYawMotorDutyMax())
                {
                    curLeftArmYawMotorDuty += getLeftArmYawMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curLeftArmYawMotorDuty = getLeftArmYawMotorDutyMax();
                    acceleration = false;
                }
                if (getLeftArmYawMotorAccelerating())
                {
                    setLeftArmYawMotorDuty(curLeftArmYawMotorDuty);
                }
                setLeftArmYawMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getLeftArmYawMotorDecelerating();
                if (curLeftArmYawMotorDuty - getLeftArmYawMotorDeceleration() > 1)
                {
                    curLeftArmYawMotorDuty -= getLeftArmYawMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curLeftArmYawMotorDuty = 1;
                    deceleration = false;
                    setLeftArmYawMotorStop(true);
                }

                if (getLeftArmYawMotorDecelerating())
                {
                    setLeftArmYawMotorDuty(curLeftArmYawMotorDuty);
                }
                setLeftArmYawMotorDecelerating(deceleration);
                if (getLeftArmYawMotorStop())
                {
                    sendCommand("LEFTARMYAWSTOP");
                    setLeftArmYawMotorAccelerating(false);
                    setLeftArmYawMotorDecelerating(false);
                }
            }
            if (getLeftArmYawCurrentTrack())
            {
                sendCommand("CHANNEL0");
                sendCommand("GETCHREAD");
            }
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void BodyControlP1::setKinect2PowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("KINECT2POWER12VON");
    }
    else
    {
        sendCommand("KINECT2POWER12VOFF");
    }
}

void BodyControlP1::headYawMoveSteps(bool direction, int stepTime, int steps)
{
    if (direction)
    {
        sendCommand("HEADYAWRIGHT");
    }
    else
    {
        sendCommand("HEADYAWLEFT");
    }
    sendCommand(Valter::format_string("HEADYAWSTEPTIME#%d", stepTime));
    sendCommand(Valter::format_string("HEADYAWSTEPS#%d", steps));
}

void BodyControlP1::headPitchMoveSteps(bool direction, int stepTime, int steps)
{
    if (direction)
    {
        sendCommand("HEADPITCHDOWN");
    }
    else
    {
        sendCommand("HEADPITCHUP");
    }
    sendCommand(Valter::format_string("HEADPITCHSTEPTIME#%d", stepTime));
    sendCommand(Valter::format_string("HEADPITCHSTEPS#%d", steps));
}

void BodyControlP1::requestHeadYawPosition()
{
    sendCommand("GETHEADYAW");
}

void BodyControlP1::requestHeadPitchPosition()
{
    sendCommand("GETHEADPITCH");
}

void BodyControlP1::releaseBodyCamera()
{
    Valter::getInstance()->executeUscCmdMaestroLinux("--servo 12,0");
}

void BodyControlP1::setBodyCameraPosition(unsigned int position)
{
    Valter::getInstance()->executeUscCmdMaestroLinux(Valter::format_string("--servo 12,%d", position));
}

void BodyControlP1::setKinect1PowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("KINECT1POWER12VON");
    }
    else
    {
        sendCommand("KINECT1POWER12VOFF");
    }
}

void BodyControlP1::setLeftArm12VPowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("LEFTARM12VENABLE");
    }
    else
    {
        sendCommand("LEFTARM12VDISABLE");
    }
}

void BodyControlP1::setRightArm12VPowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("RIGHTARM12VENABLE");
    }
    else
    {
        sendCommand("RIGHTARM12VDISABLE");
    }
}

void BodyControlP1::setRightAccumulatorOnOff(bool value)
{
    if (value)
    {
        sendCommand("RIGHTACCUMULATORON");
    }
    else
    {
        sendCommand("RIGHTACCUMULATOROFF");
    }
}

void BodyControlP1::setLeftAccumulatorOnOff(bool value)
{
    if (value)
    {
        sendCommand("LEFTACCUMULATORON");
    }
    else
    {
        sendCommand("LEFTACCUMULATOROFF");
    }
}

void BodyControlP1::setHeadLedOnOff(bool value)
{
    if (value)
    {
        sendCommand("HEADLEDON");
        setHeadLedState(true);
    }
    else
    {
        sendCommand("HEADLEDOFF");
        setHeadLedState(false);
    }
}

void BodyControlP1::setLeftArm24VPowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("LEFTARM24VON");
    }
    else
    {
        sendCommand("LEFTARM24VOFF");
    }
}

void BodyControlP1::setRightArm24VPowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("RIGHTARM24VON");
    }
    else
    {
        sendCommand("RIGHTARM24VOFF");
    }
}

void BodyControlP1::setWifiPowerOnOff(bool value)
{
    if (value)
    {
        sendCommand("WIFIPOWER12VON");
    }
    else
    {
        sendCommand("WIFIPOWER12VOFF");
    }
}

void BodyControlP1::setPowerSource5VOnOff(bool value)
{
    if (value)
    {
        sendCommand("5V5SOURCEON");
    }
    else
    {
        sendCommand("5V5SOURCEOFF");
    }
}

void BodyControlP1::shiftRegEnable()
{
    sendCommand("SHIFTREGENABLE");
}

void BodyControlP1::shiftRegDisable()
{
    sendCommand("SHIFTREGDISABLE");
}

void BodyControlP1::shiftRegReset()
{
    sendCommand("STOPSHIFTREGRESET");
}

void BodyControlP1::stopShiftRegReset()
{
    sendCommand("STOPSHIFTREGRESET");
}

bool BodyControlP1::prepareBodyPitchMovement()
{
    if (getBodyPitchMotorAccelerating() || getBodyPitchMotorDecelerating())
    {
        return false;
    }
    else
    {
        setBodyPitchMotorDuty(getBodyPitchMotorDutyPresetMin());
        return true;
    }
}

bool BodyControlP1::prepareRightArmYawMovement()
{
    if (getRightArmYawMotorAccelerating() || getRightArmYawMotorDecelerating())
    {
        return false;
    }
    else
    {
        setRightArmYawMotorDuty(getRightArmYawMotorDutyPresetMin());
        return true;
    }
}

bool BodyControlP1::prepareLeftArmYawMovement()
{
    if (getLeftArmYawMotorAccelerating() || getLeftArmYawMotorDecelerating())
    {
        return false;
    }
    else
    {
        setLeftArmYawMotorDuty(getLeftArmYawMotorDutyPresetMin());
        return true;
    }
}

void BodyControlP1::setLeftArmYawMotorActivated(bool value)
{
    leftArmYawMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setLeftArmYawMotorStop(false);
    }
}

bool BodyControlP1::setLeftArmYawMovementDirection(bool value)
{
    if (getLeftArmYawMotorStop())
    {
        leftArmYawMovementDirection = value;
        if (leftArmYawMovementDirection) //open
        {
            sendCommand("LEFTARMYAWOPEN");
        }
        else
        {
            sendCommand("LEFTARMYAWCLOSE");
        }
        return true;
    }
    else
    {
        if (getLeftArmYawMovementDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return true;
}

void BodyControlP1::setLeftArmYawMotorDuty(int value)
{
    leftArmYawMotorDuty = value;
    sendCommand(Valter::format_string("SETLEFTARMYAWDRIVEDUTY#%d", leftArmYawMotorDuty));
}

void BodyControlP1::setRightArmYawMotorActivated(bool value)
{
    rightArmYawMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setRightArmYawMotorStop(false);
    }
}

bool BodyControlP1::getRightArmYawMovementDirection() const
{
    return rightArmYawMovementDirection;
}

bool BodyControlP1::setRightArmYawMovementDirection(bool value)
{   
    if (getRightArmYawMotorStop())
    {
        rightArmYawMovementDirection = value;
        if (rightArmYawMovementDirection) //open
        {
            sendCommand("RIGHTARMYAWOPEN");
        }
        else
        {
            sendCommand("RIGHTARMYAWCLOSE");
        }
        return true;
    }
    else
    {
        if (getRightArmYawMovementDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return true;
}

void BodyControlP1::setRightArmYawMotorDuty(int value)
{
    rightArmYawMotorDuty = value;
    sendCommand(Valter::format_string("SETRIGHTARMYAWDRIVEDUTY#%d", rightArmYawMotorDuty));
}

void BodyControlP1::setBodyPitchMotorActivated(bool value)
{
    bodyPitchMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setBodyPitchMotorStop(false);
    }
}

bool BodyControlP1::getBodyPitchMovementDirection() const
{
    return bodyPitchMovementDirection;
}

bool BodyControlP1::setBodyPitchMovementDirection(bool value)
{    
    if (getBodyPitchMotorStop())
    {
        bodyPitchMovementDirection = value;
        if (bodyPitchMovementDirection) //up
        {
            sendCommand("BODYPITCHGETDOWN");
        }
        else
        {
            sendCommand("BODYPITCHGETUP");
        }
        return true;
    }
    else
    {
        if (getBodyPitchMovementDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return true;
}

void BodyControlP1::setBodyPitchMotorDuty(int value)
{
    bodyPitchMotorDuty = value;
    sendCommand(Valter::format_string("SETBODYPITCHDRIVEDUTY#%d", bodyPitchMotorDuty));
}

void BodyControlP1::setHeadPitchMotorOnOff(bool value)
{
    if (value)
    {
        sendCommand("HEADPITCHENABLE");
    }
    else
    {
        sendCommand("HEADPITCHDISABLE");
    }
}

void BodyControlP1::setHeadYawMotorOnOff(bool value)
{
    if (value)
    {
        sendCommand("HEADYAWENABLE");
    }
    else
    {
        sendCommand("HEADYAWDISABLE");
    }
}

void BodyControlP1::setHead24VOnOff(bool value)
{
    if (value)
    {
        sendCommand("HEAD24VON");
    }
    else
    {
        sendCommand("HEAD24VOFF");
    }
}

void BodyControlP1::setHeadPitchDirection(bool value)
{
    //true - pitch down, false - pitch up
    headPitchDirection = value;
    if (headPitchDirection)
    {
        sendCommand("HEADPITCHDOWN");
    }
    else
    {
        sendCommand("HEADPITCHUP");
    }
}

void BodyControlP1::setHeadYawDirection(bool value)
{
    //true - turn right, false - turn left
    headYawDirection = value;
    if (headYawDirection)
    {
        sendCommand("HEADYAWRIGHT");
    }
    else
    {
        sendCommand("HEADYAWLEFT");
    }
}
