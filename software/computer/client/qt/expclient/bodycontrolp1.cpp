#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "bodycontrolp1.h"

BodyControlP1 *BodyControlP1::pBodyControlP1 = NULL;
bool BodyControlP1::instanceFlag = false;
const string BodyControlP1::controlDeviceId = "BODY-CONTROL-P1";
const string BodyControlP1::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/body-control-p1-defaults";

BodyControlP1::BodyControlP1()
{
    Valter::log(BodyControlP1::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;

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

void BodyControlP1::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    head24VState            = false;
    headYawMotorState       = false;
    headPitchMotorState     = false;

    headYawDirection            = false;  //true - turn right, false - turn left
    headYawMotorActivated       = false;
    headYawStepDelay            = 2500;
    headYawStepSwitchDelay      = 100;

    headPitchDirection          = true;  //true - pitch down, false - pitch up
    headPitchMotorActivated     = false;
    headPitchStepDelay          = 2500; //us
    headPitchStepSwitchDelay    = 100; //us

    //------------------------------------------------body pitch
    bodyPitchMotorDuty          = 1;
    bodyPitchMotorDutyMax       = 30;
    bodyPitchMotorAcceleration  = 2;
    bodyPitchMotorDeceleration  = 2;

    bodyPitchMotorAccelerating  = false;
    bodyPitchMotorDecelerating  = false;

    bodyPitchMovementDirection  = true; //true - pitch down, false - pitch up
    bodyPitchMotorActivated     = false;
    bodyPitchMotorStop          = true;

    //presets
    bodyPitchMotorDutyPresetCur     = 30;
    bodyPitchMotorDutyPresetMin     = 1;
    bodyPitchMotorDutyPresetMax     = 100;

    //------------------------------------------------right arm yaw
    rightArmYawMotorDuty            = 1;
    rightArmYawMotorDutyMax         = 30;
    rightArmYawMotorAcceleration    = 2;
    rightArmYawMotorDeceleration    = 2;

    rightArmYawMotorAccelerating    = false;
    rightArmYawMotorDecelerating    = false;

    rightArmYawMovementDirection    = true; //true - open, false - close
    rightArmYawMotorActivated       = false;
    rightArmYawMotorStop            = false;

    //presets
    rightArmYawMotorDutyPresetCur   = 30;
    rightArmYawMotorDutyPresetMin   = 1;
    rightArmYawMotorDutyPresetMax   = 100;

    //------------------------------------------------left arm yaw
    leftArmYawMotorDuty             = 1;
    leftArmYawMotorDutyMax          = 30;
    leftArmYawMotorAcceleration     = 2;
    leftArmYawMotorDeceleration     = 2;

    leftArmYawMotorAccelerating     = false;
    leftArmYawMotorDecelerating     = false;

    leftArmYawMovementDirection     = true; //true - open, false - close
    leftArmYawMotorActivated        = false;
    leftArmYawMotorStop             = false;

    //presets
    leftArmYawMotorDutyPresetCur    = 30;
    leftArmYawMotorDutyPresetMin    = 1;
    leftArmYawMotorDutyPresetMax    = 100;
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
    ifstream defaultsFile(BodyControlP1::defaultsFilePath);
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

    //bodyPitchMotorDuty
    defaultValue = getDefault("bodyPitchMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setBodyPitchMotorDutyPresetMin(minValue);
    setBodyPitchMotorDutyPresetMax(maxValue);
    setBodyPitchMotorDutyPresetCur(curValue);
    setBodyPitchMotorDutyMax(getBodyPitchMotorDutyPresetCur());

    //bodyPitchMotorDeceleration
    defaultValue = getDefault("bodyPitchMotorDeceleration");
    bodyPitchMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //bodyPitchMotorAcceleration
    defaultValue = getDefault("bodyPitchMotorAcceleration");
    bodyPitchMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightArmYawMotorDuty
    defaultValue = getDefault("rightArmYawMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightArmYawMotorDutyPresetMin(minValue);
    setRightArmYawMotorDutyPresetMax(maxValue);
    setRightArmYawMotorDutyPresetCur(curValue);
    setRightArmYawMotorDutyMax(getRightArmYawMotorDutyPresetCur());

    //rightArmYawMotorDeceleration
    defaultValue = getDefault("rightArmYawMotorDeceleration");
    rightArmYawMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightArmYawMotorAcceleration
    defaultValue = getDefault("rightArmYawMotorAcceleration");
    rightArmYawMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftArmYawMotorDuty
    defaultValue = getDefault("leftArmYawMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftArmYawMotorDutyPresetMin(minValue);
    setLeftArmYawMotorDutyPresetMax(maxValue);
    setLeftArmYawMotorDutyPresetCur(curValue);
    setLeftArmYawMotorDutyMax(getLeftArmYawMotorDutyPresetCur());

    //leftArmYawMotorDeceleration
    defaultValue = getDefault("leftArmYawMotorDeceleration");
    leftArmYawMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftArmYawMotorAcceleration
    defaultValue = getDefault("leftArmYawMotorAcceleration");
    leftArmYawMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));
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

                if (response.compare("HEAD 24V ON") == 0)
                {
                    setHead24VState(true);
                    continue;
                }
                if (response.compare("HEAD 24V OFF") == 0)
                {
                    setHead24VState(false);
                    continue;
                }
                if (response.compare("HEAD YAW MOTOR ENABLED") == 0)
                {
                    setHeadYawMotorState(true);
                    continue;
                }
                if (response.compare("HEAD YAW MOTOR DISABLED") == 0)
                {
                    setHeadYawMotorState(false);
                    continue;
                }
                if (response.compare("HEAD PITCH MOTOR ENABLED") == 0)
                {
                    setHeadPitchMotorState(true);
                    continue;
                }
                if (response.compare("HEAD PITCH MOTOR DISABLED") == 0)
                {
                    setHeadPitchMotorState(false);
                    continue;
                }
            }
            this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

void BodyControlP1::headYawWorker()
{
    while (!stopAllProcesses)
    {
        if (getHeadYawMotorActivated())
        {
            sendCommand(Valter::format_string("HEADYAW#%d", getHeadYawStepSwitchDelay()));
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
            if (getLeftArmYawMotorActivated())
            {

            }
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

int BodyControlP1::getLeftArmYawMotorDutyPresetMax() const
{
    return leftArmYawMotorDutyPresetMax;
}

void BodyControlP1::setLeftArmYawMotorDutyPresetMax(int value)
{
    leftArmYawMotorDutyPresetMax = value;
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

int BodyControlP1::getLeftArmYawMotorDutyPresetMin() const
{
    return leftArmYawMotorDutyPresetMin;
}

void BodyControlP1::setLeftArmYawMotorDutyPresetMin(int value)
{
    leftArmYawMotorDutyPresetMin = value;
}

int BodyControlP1::getLeftArmYawMotorDutyPresetCur() const
{
    return leftArmYawMotorDutyPresetCur;
}

void BodyControlP1::setLeftArmYawMotorDutyPresetCur(int value)
{
    leftArmYawMotorDutyPresetCur = value;
}

bool BodyControlP1::getLeftArmYawMotorStop() const
{
    return leftArmYawMotorStop;
}

void BodyControlP1::setLeftArmYawMotorStop(bool value)
{
    leftArmYawMotorStop = value;
}

bool BodyControlP1::getLeftArmYawMotorActivated() const
{
    return leftArmYawMotorActivated;
}

void BodyControlP1::setLeftArmYawMotorActivated(bool value)
{
    leftArmYawMotorActivated = value;
}

bool BodyControlP1::getLeftArmYawMovementDirection() const
{
    return leftArmYawMovementDirection;
}

void BodyControlP1::setLeftArmYawMovementDirection(bool value)
{
    leftArmYawMovementDirection = value;
}

bool BodyControlP1::getLeftArmYawMotorDecelerating() const
{
    return leftArmYawMotorDecelerating;
}

void BodyControlP1::setLeftArmYawMotorDecelerating(bool value)
{
    leftArmYawMotorDecelerating = value;
}

bool BodyControlP1::getLeftArmYawMotorAccelerating() const
{
    return leftArmYawMotorAccelerating;
}

void BodyControlP1::setLeftArmYawMotorAccelerating(bool value)
{
    leftArmYawMotorAccelerating = value;
}

int BodyControlP1::getLeftArmYawMotorDeceleration() const
{
    return leftArmYawMotorDeceleration;
}

void BodyControlP1::setLeftArmYawMotorDeceleration(int value)
{
    leftArmYawMotorDeceleration = value;
}

int BodyControlP1::getLeftArmYawMotorAcceleration() const
{
    return leftArmYawMotorAcceleration;
}

void BodyControlP1::setLeftArmYawMotorAcceleration(int value)
{
    leftArmYawMotorAcceleration = value;
}

int BodyControlP1::getLeftArmYawMotorDutyMax() const
{
    return leftArmYawMotorDutyMax;
}

void BodyControlP1::setLeftArmYawMotorDutyMax(int value)
{
    leftArmYawMotorDutyMax = value;
}

int BodyControlP1::getLeftArmYawMotorDuty() const
{
    return leftArmYawMotorDuty;
}

void BodyControlP1::setLeftArmYawMotorDuty(int value)
{
    leftArmYawMotorDuty = value;
}

int BodyControlP1::getRightArmYawMotorDutyPresetMax() const
{
    return rightArmYawMotorDutyPresetMax;
}

void BodyControlP1::setRightArmYawMotorDutyPresetMax(int value)
{
    rightArmYawMotorDutyPresetMax = value;
}

int BodyControlP1::getRightArmYawMotorDutyPresetMin() const
{
    return rightArmYawMotorDutyPresetMin;
}

void BodyControlP1::setRightArmYawMotorDutyPresetMin(int value)
{
    rightArmYawMotorDutyPresetMin = value;
}

int BodyControlP1::getRightArmYawMotorDutyPresetCur() const
{
    return rightArmYawMotorDutyPresetCur;
}

void BodyControlP1::setRightArmYawMotorDutyPresetCur(int value)
{
    rightArmYawMotorDutyPresetCur = value;
}

bool BodyControlP1::getRightArmYawMotorStop() const
{
    return rightArmYawMotorStop;
}

void BodyControlP1::setRightArmYawMotorStop(bool value)
{
    rightArmYawMotorStop = value;
}

bool BodyControlP1::getRightArmYawMotorActivated() const
{
    return rightArmYawMotorActivated;
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

bool BodyControlP1::getRightArmYawMotorDecelerating() const
{
    return rightArmYawMotorDecelerating;
}

void BodyControlP1::setRightArmYawMotorDecelerating(bool value)
{
    rightArmYawMotorDecelerating = value;
}

bool BodyControlP1::getRightArmYawMotorAccelerating() const
{
    return rightArmYawMotorAccelerating;
}

void BodyControlP1::setRightArmYawMotorAccelerating(bool value)
{
    rightArmYawMotorAccelerating = value;
}

int BodyControlP1::getRightArmYawMotorDeceleration() const
{
    return rightArmYawMotorDeceleration;
}

void BodyControlP1::setRightArmYawMotorDeceleration(int value)
{
    rightArmYawMotorDeceleration = value;
}

int BodyControlP1::getRightArmYawMotorAcceleration() const
{
    return rightArmYawMotorAcceleration;
}

void BodyControlP1::setRightArmYawMotorAcceleration(int value)
{
    rightArmYawMotorAcceleration = value;
}

int BodyControlP1::getRightArmYawMotorDutyMax() const
{
    return rightArmYawMotorDutyMax;
}

void BodyControlP1::setRightArmYawMotorDutyMax(int value)
{
    rightArmYawMotorDutyMax = value;
}

int BodyControlP1::getRightArmYawMotorDuty() const
{
    return rightArmYawMotorDuty;
}

void BodyControlP1::setRightArmYawMotorDuty(int value)
{
    rightArmYawMotorDuty = value;
    sendCommand(Valter::format_string("SETRIGHTARMYAWDRIVEDUTY#%d", rightArmYawMotorDuty));
}

int BodyControlP1::getBodyPitchMotorDutyPresetMax() const
{
    return bodyPitchMotorDutyPresetMax;
}

void BodyControlP1::setBodyPitchMotorDutyPresetMax(int value)
{
    bodyPitchMotorDutyPresetMax = value;
}

int BodyControlP1::getBodyPitchMotorDutyPresetMin() const
{
    return bodyPitchMotorDutyPresetMin;
}

void BodyControlP1::setBodyPitchMotorDutyPresetMin(int value)
{
    bodyPitchMotorDutyPresetMin = value;
}

int BodyControlP1::getBodyPitchMotorDutyPresetCur() const
{
    return bodyPitchMotorDutyPresetCur;
}

void BodyControlP1::setBodyPitchMotorDutyPresetCur(int value)
{
    bodyPitchMotorDutyPresetCur = value;
}

bool BodyControlP1::getBodyPitchMotorStop() const
{
    return bodyPitchMotorStop;
}

void BodyControlP1::setBodyPitchMotorStop(bool value)
{
    bodyPitchMotorStop = value;
}

bool BodyControlP1::getBodyPitchMotorActivated() const
{
    return bodyPitchMotorActivated;
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

bool BodyControlP1::getBodyPitchMotorDecelerating() const
{
    return bodyPitchMotorDecelerating;
}

void BodyControlP1::setBodyPitchMotorDecelerating(bool value)
{
    bodyPitchMotorDecelerating = value;
}

bool BodyControlP1::getBodyPitchMotorAccelerating() const
{
    return bodyPitchMotorAccelerating;
}

void BodyControlP1::setBodyPitchMotorAccelerating(bool value)
{
    bodyPitchMotorAccelerating = value;
}

int BodyControlP1::getBodyPitchMotorDeceleration() const
{
    return bodyPitchMotorDeceleration;
}

void BodyControlP1::setBodyPitchMotorDeceleration(int value)
{
    bodyPitchMotorDeceleration = value;
}

int BodyControlP1::getBodyPitchMotorAcceleration() const
{
    return bodyPitchMotorAcceleration;
}

void BodyControlP1::setBodyPitchMotorAcceleration(int value)
{
    bodyPitchMotorAcceleration = value;
}

int BodyControlP1::getBodyPitchMotorDutyMax() const
{
    return bodyPitchMotorDutyMax;
}

void BodyControlP1::setBodyPitchMotorDutyMax(int value)
{
    bodyPitchMotorDutyMax = value;
}

int BodyControlP1::getBodyPitchMotorDuty() const
{
    return bodyPitchMotorDuty;
}

void BodyControlP1::setBodyPitchMotorDuty(int value)
{
    bodyPitchMotorDuty = value;
    sendCommand(Valter::format_string("SETBODYPITCHDRIVEDUTY#%d", bodyPitchMotorDuty));
}

bool BodyControlP1::getHeadPitchMotorState() const
{
    return headPitchMotorState;
}

void BodyControlP1::setHeadPitchMotorState(bool value)
{
    headPitchMotorState = value;
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

bool BodyControlP1::getHeadYawMotorState() const
{
    return headYawMotorState;
}

void BodyControlP1::setHeadYawMotorState(bool value)
{
    headYawMotorState = value;
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

bool BodyControlP1::getHead24VState() const
{
    return head24VState;
}

void BodyControlP1::setHead24VState(bool value)
{
    head24VState = value;
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

int BodyControlP1::getHeadPitchStepSwitchDelay() const
{
    return headPitchStepSwitchDelay;
}

void BodyControlP1::setHeadPitchStepSwitchDelay(int value)
{
    headPitchStepSwitchDelay = value;
}

int BodyControlP1::getHeadPitchStepDelay() const
{
    return headPitchStepDelay;
}

void BodyControlP1::setHeadPitchStepDelay(int value)
{
    headPitchStepDelay = value;
}

bool BodyControlP1::getHeadPitchMotorActivated() const
{
    return headPitchMotorActivated;
}

void BodyControlP1::setHeadPitchMotorActivated(bool value)
{
    headPitchMotorActivated = value;
}

bool BodyControlP1::getHeadPitchDirection() const
{
    return headPitchDirection;
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

int BodyControlP1::getHeadYawStepSwitchDelay() const
{
    return headYawStepSwitchDelay;
}

void BodyControlP1::setHeadYawStepSwitchDelay(int value)
{
    headYawStepSwitchDelay = value;
}

int BodyControlP1::getHeadYawStepDelay() const
{
    return headYawStepDelay;
}

void BodyControlP1::setHeadYawStepDelay(int value)
{
    headYawStepDelay = value;
}

bool BodyControlP1::getHeadYawMotorActivated() const
{
    return headYawMotorActivated;
}

void BodyControlP1::setHeadYawMotorActivated(bool value)
{
    headYawMotorActivated = value;
}

bool BodyControlP1::getHeadYawDirection() const
{
    return headYawDirection;
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
