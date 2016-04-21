#include "valter.h"
#include "armcontrolleft.h"

#include "armcontrolleft.utils.cpp"

ArmControlLeft *ArmControlLeft::pArmControlLeft = NULL;
bool ArmControlLeft::instanceFlag = false;
const string ArmControlLeft::controlDeviceId = "ARM-CONTROL-LEFT";
const string ArmControlLeft::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/arm-control-left-defaults";

ArmControlLeft::ArmControlLeft()
{
    Valter::log(ArmControlLeft::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;

    resetToDefault();
    loadDefaults();

    new std::thread(&ArmControlLeft::leftForearmWorker, this);
    new std::thread(&ArmControlLeft::leftArmWorker, this);
    new std::thread(&ArmControlLeft::leftLimbWorker, this);
}

ArmControlLeft *ArmControlLeft::getInstance()
{
    if(!instanceFlag)
    {
        pArmControlLeft = new ArmControlLeft();
        instanceFlag = true;
        return pArmControlLeft;
    }
    else
    {
        return pArmControlLeft;
    }
}

string ArmControlLeft::getControlDeviceId()
{
    return controlDeviceId;
}

void ArmControlLeft::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", ArmControlLeft::controlDeviceId.c_str()));
    }
}

void ArmControlLeft::setModuleInitialState()
{

}

void ArmControlLeft::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&ArmControlLeft::processMessagesQueueWorker, this));
}

void ArmControlLeft::processMessagesQueueWorker()
{
    if (getControlDeviceIsSet())
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {
                string response = getControlDevice()->pullResponse();

                if (response.compare("FOREARM ROLL MOTOR ON") == 0)
                {
                    setForearmRollMotorState(true);
                    continue;
                }
                if (response.compare("FOREARM ROLL MOTOR OFF") == 0)
                {
                    setForearmRollMotorState(false);
                    continue;
                }

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
}

void ArmControlLeft::leftForearmWorker()
{
    while (!stopAllProcesses)
    {
        if (!getLeftForearmMotorStop())
        {
            int curLeftForearmMotorDuty = getLeftForearmMotorDuty();
            bool acceleration, deceleration;
            if (getLeftForearmMotorActivated())
            {
                acceleration = getLeftForearmMotorAccelerating();
                if (curLeftForearmMotorDuty + getLeftForearmMotorAcceleration() < getLeftForearmMotorDutyMax())
                {
                    curLeftForearmMotorDuty += getLeftForearmMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curLeftForearmMotorDuty = getLeftForearmMotorDutyMax();
                    acceleration = false;
                }
                if (getLeftForearmMotorAccelerating())
                {
                    setLeftForearmMotorDuty(curLeftForearmMotorDuty);
                }
                setLeftForearmMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getLeftForearmMotorDecelerating();
                if (curLeftForearmMotorDuty - getLeftForearmMotorDeceleration() > 1)
                {
                    curLeftForearmMotorDuty -= getLeftForearmMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curLeftForearmMotorDuty = 1;
                    deceleration = false;
                    setLeftForearmMotorStop(true);
                }

                if (getLeftForearmMotorDecelerating())
                {
                    setLeftForearmMotorDuty(curLeftForearmMotorDuty);
                }
                setLeftForearmMotorDecelerating(deceleration);
                if (getLeftForearmMotorStop())
                {
                    sendCommand("FOREARMLIFTUPSTOP");
                    setLeftForearmMotorAccelerating(false);
                    setLeftForearmMotorDecelerating(false);
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

void ArmControlLeft::leftArmWorker()
{
    while (!stopAllProcesses)
    {
        if (!getLeftArmMotorStop())
        {
            int curLeftArmMotorDuty = getLeftArmMotorDuty();
            bool acceleration, deceleration;
            if (getLeftArmMotorActivated())
            {
                acceleration = getLeftArmMotorAccelerating();
                if (curLeftArmMotorDuty + getLeftArmMotorAcceleration() < getLeftArmMotorDutyMax())
                {
                    curLeftArmMotorDuty += getLeftArmMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curLeftArmMotorDuty = getLeftArmMotorDutyMax();
                    acceleration = false;
                }
                if (getLeftArmMotorAccelerating())
                {
                    setLeftArmMotorDuty(curLeftArmMotorDuty);
                }
                setLeftArmMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getLeftArmMotorDecelerating();
                if (curLeftArmMotorDuty - getLeftArmMotorDeceleration() > 1)
                {
                    curLeftArmMotorDuty -= getLeftArmMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curLeftArmMotorDuty = 1;
                    deceleration = false;
                    setLeftArmMotorStop(true);
                }

                if (getLeftArmMotorDecelerating())
                {
                    setLeftArmMotorDuty(curLeftArmMotorDuty);
                }
                setLeftArmMotorDecelerating(deceleration);
                if (getLeftArmMotorStop())
                {
                    sendCommand("ARMLIFTUPSTOP");
                    setLeftArmMotorAccelerating(false);
                    setLeftArmMotorDecelerating(false);
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

void ArmControlLeft::leftLimbWorker()
{
    while (!stopAllProcesses)
    {
        if (!getLeftLimbMotorStop())
        {
            int curLeftLimbMotorDuty = getLeftLimbMotorDuty();
            bool acceleration, deceleration;
            if (getLeftLimbMotorActivated())
            {
                acceleration = getLeftLimbMotorAccelerating();
                if (curLeftLimbMotorDuty + getLeftLimbMotorAcceleration() < getLeftLimbMotorDutyMax())
                {
                    curLeftLimbMotorDuty += getLeftLimbMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curLeftLimbMotorDuty = getLeftLimbMotorDutyMax();
                    acceleration = false;
                }
                if (getLeftLimbMotorAccelerating())
                {
                    setLeftLimbMotorDuty(curLeftLimbMotorDuty);
                }
                setLeftLimbMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getLeftLimbMotorDecelerating();
                if (curLeftLimbMotorDuty - getLeftLimbMotorDeceleration() > 1)
                {
                    curLeftLimbMotorDuty -= getLeftLimbMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curLeftLimbMotorDuty = 1;
                    deceleration = false;
                    setLeftLimbMotorStop(true);
                }

                if (getLeftLimbMotorDecelerating())
                {
                    setLeftLimbMotorDuty(curLeftLimbMotorDuty);
                }
                setLeftLimbMotorDecelerating(deceleration);
                if (getLeftLimbMotorStop())
                {
                    sendCommand("LIMBLIFTUPSTOP");
                    setLeftLimbMotorAccelerating(false);
                    setLeftLimbMotorDecelerating(false);
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

bool ArmControlLeft::getForearmRollMotorState() const
{
    return forearmRollMotorState;
}

void ArmControlLeft::setForearmRollMotorState(bool value)
{
    forearmRollMotorState = value;
}

void ArmControlLeft::setForearmRollMotorOnOff(bool value)
{
    if (value)//on
    {
        sendCommand("FOREARMROLLON");
    }
    else
    {
        sendCommand("FOREARMROLLOFF");
    }
}

bool ArmControlLeft::getForearmRollDirection() const
{
    return forearmRollDirection;
}

void ArmControlLeft::setForearmRollDirection(bool value)
{
    forearmRollDirection = value;
    if (value)//true - CW, false - CCW
    {
        sendCommand("FOREARMROLLCW");
    }
    else
    {
        sendCommand("FOREARMROLLCCW");
    }
}

bool ArmControlLeft::getHandPitchDirection() const
{
    return handPitchDirection;
}

void ArmControlLeft::setHandPitchDirection(bool value)
{
    handPitchDirection = value;
    if (value)//up
    {
        sendCommand("HANDPITCHCCW");
    }
    else
    {
        sendCommand("HANDPITCHCW");
    }
}

void ArmControlLeft::handPitch(bool state)
{
    if (state)
    {
        sendCommand("HANDPITCHON");
    }
    else
    {
        sendCommand("HANDPITCHOFF");
    }
}

bool ArmControlLeft::getHandYawDirection() const
{
    return handYawDirection;
}

void ArmControlLeft::setHandYawDirection(bool value)
{
    handYawDirection = value;
    if (value) //CW
    {
        sendCommand("HANDYAWCW");
    }
    else
    {
        sendCommand("HANDYAWCCW");
    }
}

void ArmControlLeft::handYaw(bool state)
{
    if (state)
    {
        sendCommand("HANDYAWON");
    }
    else
    {
        sendCommand("HANDYAWOFF");
    }
}

bool ArmControlLeft::prepareLeftForearmMovement()
{
    if (getLeftForearmMotorAccelerating() || getLeftForearmMotorDecelerating())
    {
        return false;
    }
    else
    {
        setLeftForearmMotorDuty(getLeftForearmMotorDutyPresetMin());
        return true;
    }
}

bool ArmControlLeft::prepareLeftArmMovement()
{
    if (getLeftArmMotorAccelerating() || getLeftArmMotorDecelerating())
    {
        return false;
    }
    else
    {
        setLeftArmMotorDuty(getLeftArmMotorDutyPresetMin());
        return true;
    }
}

bool ArmControlLeft::prepareLeftLimbMovement()
{
    if (getLeftLimbMotorAccelerating() || getLeftLimbMotorDecelerating())
    {
        return false;
    }
    else
    {
        setLeftLimbMotorDuty(getLeftLimbMotorDutyPresetMin());
        return true;
    }
}

int ArmControlLeft::getLeftLimbMotorDutyPresetMin() const
{
    return leftLimbMotorDutyPresetMin;
}

void ArmControlLeft::setLeftLimbMotorDutyPresetMin(int value)
{
    leftLimbMotorDutyPresetMin = value;
}

int ArmControlLeft::getLeftLimbMotorDutyPresetCur() const
{
    return leftLimbMotorDutyPresetCur;
}

void ArmControlLeft::setLeftLimbMotorDutyPresetCur(int value)
{
    leftLimbMotorDutyPresetCur = value;
}

int ArmControlLeft::getLeftLimbADCCurrent() const
{
    return leftLimbADCCurrent;
}

void ArmControlLeft::setLeftLimbADCCurrent(int value)
{
    leftLimbADCCurrent = value;
}

int ArmControlLeft::getLeftLimbADCPosition() const
{
    return leftLimbADCPosition;
}

void ArmControlLeft::setLeftLimbADCPosition(int value)
{
    leftLimbADCPosition = value;
}

bool ArmControlLeft::getLeftLimbMotorStop() const
{
    return leftLimbMotorStop;
}

void ArmControlLeft::setLeftLimbMotorStop(bool value)
{
    leftLimbMotorStop = value;
}

bool ArmControlLeft::getLeftLimbMotorActivated() const
{
    return leftLimbMotorActivated;
}

void ArmControlLeft::setLeftLimbMotorActivated(bool value)
{
    leftLimbMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setLeftLimbMotorStop(false);
    }
}

bool ArmControlLeft::getLeftLimbMotorMovementDirection() const
{
    return leftLimbMotorMovementDirection;
}

bool ArmControlLeft::setLeftLimbMotorMovementDirection(bool value)
{
    if (getLeftLimbMotorStop())
    {
        leftLimbMotorMovementDirection = value;
        if (leftLimbMotorMovementDirection) //down
        {
            sendCommand("LIMBLIFTUPDOWN");
        }
        else
        {
            sendCommand("LIMBLIFTUPUP");
        }
        return true;
    }
    else
    {
        if (getLeftLimbMotorMovementDirection() == value)
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

bool ArmControlLeft::getLeftLimbMotorDecelerating() const
{
    return leftLimbMotorDecelerating;
}

void ArmControlLeft::setLeftLimbMotorDecelerating(bool value)
{
    leftLimbMotorDecelerating = value;
}

bool ArmControlLeft::getLeftLimbMotorAccelerating() const
{
    return leftLimbMotorAccelerating;
}

void ArmControlLeft::setLeftLimbMotorAccelerating(bool value)
{
    leftLimbMotorAccelerating = value;
}

int ArmControlLeft::getLeftLimbMotorDeceleration() const
{
    return leftLimbMotorDeceleration;
}

void ArmControlLeft::setLeftLimbMotorDeceleration(int value)
{
    leftLimbMotorDeceleration = value;
}

int ArmControlLeft::getLeftLimbMotorAcceleration() const
{
    return leftLimbMotorAcceleration;
}

void ArmControlLeft::setLeftLimbMotorAcceleration(int value)
{
    leftLimbMotorAcceleration = value;
}

int ArmControlLeft::getLeftLimbMotorDutyMax() const
{
    return leftLimbMotorDutyMax;
}

void ArmControlLeft::setLeftLimbMotorDutyMax(int value)
{
    leftLimbMotorDutyMax = value;
}

int ArmControlLeft::getLeftLimbMotorDuty() const
{
    return leftLimbMotorDuty;
}

void ArmControlLeft::setLeftLimbMotorDuty(int value)
{
    leftLimbMotorDuty = value;
    sendCommand(Valter::format_string("SETLIMBLIFTUPDUTY#%d", leftLimbMotorDuty));
}

int ArmControlLeft::getLeftArmMotorDutyPresetMax() const
{
    return leftArmMotorDutyPresetMax;
}

void ArmControlLeft::setLeftArmMotorDutyPresetMax(int value)
{
    leftArmMotorDutyPresetMax = value;
}

int ArmControlLeft::getLeftArmMotorDutyPresetMin() const
{
    return leftArmMotorDutyPresetMin;
}

void ArmControlLeft::setLeftArmMotorDutyPresetMin(int value)
{
    leftArmMotorDutyPresetMin = value;
}

int ArmControlLeft::getLeftArmMotorDutyPresetCur() const
{
    return leftArmMotorDutyPresetCur;
}

void ArmControlLeft::setLeftArmMotorDutyPresetCur(int value)
{
    leftArmMotorDutyPresetCur = value;
}

int ArmControlLeft::getLeftArmADCCurrent() const
{
    return leftArmADCCurrent;
}

void ArmControlLeft::setLeftArmADCCurrent(int value)
{
    leftArmADCCurrent = value;
}

int ArmControlLeft::getLeftArmADCPosition() const
{
    return leftArmADCPosition;
}

void ArmControlLeft::setLeftArmADCPosition(int value)
{
    leftArmADCPosition = value;
}

bool ArmControlLeft::getLeftArmMotorStop() const
{
    return leftArmMotorStop;
}

void ArmControlLeft::setLeftArmMotorStop(bool value)
{
    leftArmMotorStop = value;
}

bool ArmControlLeft::getLeftArmMotorActivated() const
{
    return leftArmMotorActivated;
}

void ArmControlLeft::setLeftArmMotorActivated(bool value)
{
    leftArmMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setLeftArmMotorStop(false);
    }
}

bool ArmControlLeft::getLeftArmMotorMovementDirection() const
{
    return leftArmMotorMovementDirection;
}

bool ArmControlLeft::setLeftArmMotorMovementDirection(bool value)
{
    if (getLeftArmMotorStop())
    {
        leftArmMotorMovementDirection = value;
        if (leftArmMotorMovementDirection) //down
        {
            sendCommand("ARMLIFTUPDOWN");
        }
        else
        {
            sendCommand("ARMLIFTUPUP");
        }
        return true;
    }
    else
    {
        if (getLeftArmMotorMovementDirection() == value)
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

bool ArmControlLeft::getLeftArmMotorDecelerating() const
{
    return leftArmMotorDecelerating;
}

void ArmControlLeft::setLeftArmMotorDecelerating(bool value)
{
    leftArmMotorDecelerating = value;
}

bool ArmControlLeft::getLeftArmMotorAccelerating() const
{
    return leftArmMotorAccelerating;
}

void ArmControlLeft::setLeftArmMotorAccelerating(bool value)
{
    leftArmMotorAccelerating = value;
}

int ArmControlLeft::getLeftArmMotorDeceleration() const
{
    return leftArmMotorDeceleration;
}

void ArmControlLeft::setLeftArmMotorDeceleration(int value)
{
    leftArmMotorDeceleration = value;
}

int ArmControlLeft::getLeftArmMotorAcceleration() const
{
    return leftArmMotorAcceleration;
}

void ArmControlLeft::setLeftArmMotorAcceleration(int value)
{
    leftArmMotorAcceleration = value;
}

int ArmControlLeft::getLeftArmMotorDutyMax() const
{
    return leftArmMotorDutyMax;
}

void ArmControlLeft::setLeftArmMotorDutyMax(int value)
{
    leftArmMotorDutyMax = value;
}

int ArmControlLeft::getLeftArmMotorDuty() const
{
    return leftArmMotorDuty;
}

void ArmControlLeft::setLeftArmMotorDuty(int value)
{
    leftArmMotorDuty = value;
    sendCommand(Valter::format_string("SETARMLIFTUPDUTY#%d", leftArmMotorDuty));
}

int ArmControlLeft::getLeftForearmMotorDutyPresetMax() const
{
    return leftForearmMotorDutyPresetMax;
}

void ArmControlLeft::setLeftForearmMotorDutyPresetMax(int value)
{
    leftForearmMotorDutyPresetMax = value;
}

int ArmControlLeft::getLeftForearmMotorDutyPresetMin() const
{
    return leftForearmMotorDutyPresetMin;
}

void ArmControlLeft::setLeftForearmMotorDutyPresetMin(int value)
{
    leftForearmMotorDutyPresetMin = value;
}

int ArmControlLeft::getLeftForearmMotorDutyPresetCur() const
{
    return leftForearmMotorDutyPresetCur;
}

void ArmControlLeft::setLeftForearmMotorDutyPresetCur(int value)
{
    leftForearmMotorDutyPresetCur = value;
}

int ArmControlLeft::getLeftForearmADCCurrent() const
{
    return leftForearmADCCurrent;
}

void ArmControlLeft::setLeftForearmADCCurrent(int value)
{
    leftForearmADCCurrent = value;
}

int ArmControlLeft::getLeftForearmADCPosition() const
{
    return leftForearmADCPosition;
}

void ArmControlLeft::setLeftForearmADCPosition(int value)
{
    leftForearmADCPosition = value;
}

bool ArmControlLeft::getLeftForearmMotorStop() const
{
    return leftForearmMotorStop;
}

void ArmControlLeft::setLeftForearmMotorStop(bool value)
{
    leftForearmMotorStop = value;
}

bool ArmControlLeft::getLeftForearmMotorActivated() const
{
    return leftForearmMotorActivated;
}

void ArmControlLeft::setLeftForearmMotorActivated(bool value)
{
    leftForearmMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setLeftForearmMotorStop(false);
    }
}

bool ArmControlLeft::getLeftForearmMotorMovementDirection() const
{
    return leftForearmMotorMovementDirection;
}

bool ArmControlLeft::setLeftForearmMotorMovementDirection(bool value)
{
    if (getLeftForearmMotorStop())
    {
        leftForearmMotorMovementDirection = value;
        if (leftForearmMotorMovementDirection) //down
        {
            sendCommand("FOREARMLIFTUPDOWN");
        }
        else
        {
            sendCommand("FOREARMLIFTUPUP");
        }
        return true;
    }
    else
    {
        if (getLeftForearmMotorMovementDirection() == value)
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

bool ArmControlLeft::getLeftForearmMotorDecelerating() const
{
    return leftForearmMotorDecelerating;
}

void ArmControlLeft::setLeftForearmMotorDecelerating(bool value)
{
    leftForearmMotorDecelerating = value;
}

bool ArmControlLeft::getLeftForearmMotorAccelerating() const
{
    return leftForearmMotorAccelerating;
}

void ArmControlLeft::setLeftForearmMotorAccelerating(bool value)
{
    leftForearmMotorAccelerating = value;
}

int ArmControlLeft::getLeftForearmMotorDeceleration() const
{
    return leftForearmMotorDeceleration;
}

void ArmControlLeft::setLeftForearmMotorDeceleration(int value)
{
    leftForearmMotorDeceleration = value;
}

int ArmControlLeft::getLeftForearmMotorAcceleration() const
{
    return leftForearmMotorAcceleration;
}

void ArmControlLeft::setLeftForearmMotorAcceleration(int value)
{
    leftForearmMotorAcceleration = value;
}

int ArmControlLeft::getLeftForearmMotorDutyMax() const
{
    return leftForearmMotorDutyMax;
}

void ArmControlLeft::setLeftForearmMotorDutyMax(int value)
{
    leftForearmMotorDutyMax = value;
}

int ArmControlLeft::getLeftForearmMotorDuty() const
{
    return leftForearmMotorDuty;
}

void ArmControlLeft::setLeftForearmMotorDuty(int value)
{
    leftForearmMotorDuty = value;
    sendCommand(Valter::format_string("SETFOREARMLIFTUPDUTY#%d", leftForearmMotorDuty));
}
