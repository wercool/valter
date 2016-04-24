#include "valter.h"
#include "armcontrolright.h"

#include "armcontrolright.h"

ArmControlRight *ArmControlRight::pArmControlRight = NULL;
bool ArmControlRight::instanceFlag = false;
const string ArmControlRight::controlDeviceId = "ARM-CONTROL-RIGHT";
const string ArmControlRight::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/arm-control-right-defaults";

ArmControlRight::ArmControlRight()
{
    Valter::log(ArmControlRight::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;

    resetToDefault();
    loadDefaults();

    new std::thread(&ArmControlRight::rightForearmWorker, this);
    new std::thread(&ArmControlRight::rightArmWorker, this);
    new std::thread(&ArmControlRight::rightLimbWorker, this);
    new std::thread(&ArmControlRight::rightForearmRollWorker, this);
    new std::thread(&ArmControlRight::rightArmSensorReadingsWorker, this);
}

ArmControlRight *ArmControlRight::getInstance()
{
    if(!instanceFlag)
    {
        pArmControlRight = new ArmControlRight();
        instanceFlag = true;
        return pArmControlRight;
    }
    else
    {
        return pArmControlRight;
    }
}

string ArmControlRight::getControlDeviceId()
{
    return controlDeviceId;
}

void ArmControlRight::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", ArmControlRight::controlDeviceId.c_str()));
    }
}

void ArmControlRight::setModuleInitialState()
{

}

void ArmControlRight::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&ArmControlRight::processMessagesQueueWorker, this));
}

void ArmControlRight::processMessagesQueueWorker()
{
    if (getControlDeviceIsSet())
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {
                string response = getControlDevice()->pullResponse();

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
}

void ArmControlRight::rightForearmWorker()
{
    while (!stopAllProcesses)
    {
        if (!getRightForearmMotorStop())
        {
            int curRightForearmMotorDuty = getRightForearmMotorDuty();
            bool acceleration, deceleration;
            if (getRightForearmMotorActivated())
            {
                acceleration = getRightForearmMotorAccelerating();
                if (curRightForearmMotorDuty + getRightForearmMotorAcceleration() < getRightForearmMotorDutyMax())
                {
                    curRightForearmMotorDuty += getRightForearmMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curRightForearmMotorDuty = getRightForearmMotorDutyMax();
                    acceleration = false;
                }
                if (getRightForearmMotorAccelerating())
                {
                    setRightForearmMotorDuty(curRightForearmMotorDuty);
                }
                setRightForearmMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getRightForearmMotorDecelerating();
                if (curRightForearmMotorDuty - getRightForearmMotorDeceleration() > 1)
                {
                    curRightForearmMotorDuty -= getRightForearmMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curRightForearmMotorDuty = 1;
                    deceleration = false;
                    setRightForearmMotorStop(true);
                }

                if (getRightForearmMotorDecelerating())
                {
                    setRightForearmMotorDuty(curRightForearmMotorDuty);
                }
                setRightForearmMotorDecelerating(deceleration);
                if (getRightForearmMotorStop())
                {
                    sendCommand("FOREARMLIFTUPSTOP");
                    setRightForearmMotorAccelerating(false);
                    setRightForearmMotorDecelerating(false);
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

void ArmControlRight::rightArmWorker()
{

}

void ArmControlRight::rightLimbWorker()
{

}

void ArmControlRight::rightForearmRollWorker()
{

}

void ArmControlRight::rightArmSensorReadingsWorker()
{

}

int ArmControlRight::getRightLimbMotorDutyPresetMax() const
{
    return rightLimbMotorDutyPresetMax;
}

void ArmControlRight::setRightLimbMotorDutyPresetMax(int value)
{
    rightLimbMotorDutyPresetMax = value;
}

bool ArmControlRight::prepareRightForearmMovement()
{
    if (getRightForearmMotorAccelerating() || getRightForearmMotorDecelerating())
    {
        return false;
    }
    else
    {
        setRightForearmMotorDuty(getRightForearmMotorDutyPresetMin());
        return true;
    }
}

bool ArmControlRight::prepareRightArmMovement()
{
    if (getRightArmMotorAccelerating() || getRightArmMotorDecelerating())
    {
        return false;
    }
    else
    {
        setRightArmMotorDuty(getRightArmMotorDutyPresetMin());
        return true;
    }
}

bool ArmControlRight::prepareRightLimbMovement()
{
    if (getRightLimbMotorAccelerating() || getRightLimbMotorDecelerating())
    {
        return false;
    }
    else
    {
        setRightLimbMotorDuty(getRightLimbMotorDutyPresetMin());
        return true;
    }
}

void ArmControlRight::handYaw(bool state)
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

void ArmControlRight::handPitch(bool state)
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

void ArmControlRight::setForearmRollMotorOnOff(bool state)
{
    if (state)//on
    {
        sendCommand("FOREARMROLLON");
    }
    else
    {
        sendCommand("FOREARMROLLOFF");
    }
}

void ArmControlRight::forearmYaw(bool state)
{
    if (state)
    {
        sendCommand("FOREARMYAWON");
    }
    else
    {
        sendCommand("FOREARMYAWOFF");
    }
}

void ArmControlRight::turnArmLedsOnOff()
{
    bool state = getArmLedsState();
    if (state)
    {
        sendCommand("ARMLEDSOFF");
    }
    else
    {
        sendCommand("ARMLEDSON");
    }
    setArmLedsState(!state);
}

int ArmControlRight::getRightLimbMotorDutyPresetMin() const
{
    return rightLimbMotorDutyPresetMin;
}

void ArmControlRight::setRightLimbMotorDutyPresetMin(int value)
{
    rightLimbMotorDutyPresetMin = value;
}

int ArmControlRight::getRightLimbMotorDutyPresetCur() const
{
    return rightLimbMotorDutyPresetCur;
}

void ArmControlRight::setRightLimbMotorDutyPresetCur(int value)
{
    rightLimbMotorDutyPresetCur = value;
}

int ArmControlRight::getRightLimbADCCurrent() const
{
    return rightLimbADCCurrent;
}

void ArmControlRight::setRightLimbADCCurrent(int value)
{
    rightLimbADCCurrent = value;
}

int ArmControlRight::getRightLimbADCPosition() const
{
    return rightLimbADCPosition;
}

void ArmControlRight::setRightLimbADCPosition(int value)
{
    rightLimbADCPosition = value;
}

bool ArmControlRight::getRightLimbMotorStop() const
{
    return rightLimbMotorStop;
}

void ArmControlRight::setRightLimbMotorStop(bool value)
{
    rightLimbMotorStop = value;
}

bool ArmControlRight::getRightLimbMotorActivated() const
{
    return rightLimbMotorActivated;
}

void ArmControlRight::setRightLimbMotorActivated(bool value)
{
    rightLimbMotorActivated = value;
}

bool ArmControlRight::getRightLimbMotorMovementDirection() const
{
    return rightLimbMotorMovementDirection;
}

bool ArmControlRight::setRightLimbMotorMovementDirection(bool value)
{
    if (getRightLimbMotorStop())
    {
        rightLimbMotorMovementDirection = value;
        if (rightLimbMotorMovementDirection) //down
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
        if (getRightLimbMotorMovementDirection() == value)
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

bool ArmControlRight::getRightLimbMotorDecelerating() const
{
    return rightLimbMotorDecelerating;
}

void ArmControlRight::setRightLimbMotorDecelerating(bool value)
{
    rightLimbMotorDecelerating = value;
}

bool ArmControlRight::getRightLimbMotorAccelerating() const
{
    return rightLimbMotorAccelerating;
}

void ArmControlRight::setRightLimbMotorAccelerating(bool value)
{
    rightLimbMotorAccelerating = value;
}

int ArmControlRight::getRightLimbMotorDeceleration() const
{
    return rightLimbMotorDeceleration;
}

void ArmControlRight::setRightLimbMotorDeceleration(int value)
{
    rightLimbMotorDeceleration = value;
}

int ArmControlRight::getRightLimbMotorAcceleration() const
{
    return rightLimbMotorAcceleration;
}

void ArmControlRight::setRightLimbMotorAcceleration(int value)
{
    rightLimbMotorAcceleration = value;
}

int ArmControlRight::getRightLimbMotorDutyMax() const
{
    return rightLimbMotorDutyMax;
}

void ArmControlRight::setRightLimbMotorDutyMax(int value)
{
    rightLimbMotorDutyMax = value;
}

int ArmControlRight::getRightLimbMotorDuty() const
{
    return rightLimbMotorDuty;
}

void ArmControlRight::setRightLimbMotorDuty(int value)
{
    rightLimbMotorDuty = value;
}

bool ArmControlRight::getArmLedsState() const
{
    return armLedsState;
}

void ArmControlRight::setArmLedsState(bool value)
{
    armLedsState = value;
}

int ArmControlRight::getHandPitchMotorADCCurrent() const
{
    return handPitchMotorADCCurrent;
}

void ArmControlRight::setHandPitchMotorADCCurrent(int value)
{
    handPitchMotorADCCurrent = value;
}

void ArmControlRight::setHandSensorsTrack(int idx, bool state)
{
    handSensorsTrack[idx] = state;
}

bool ArmControlRight::getHandSensorsTrack(int idx)
{
    return handSensorsTrack[idx];
}

void ArmControlRight::setHandSensorsADCForce(int idx, int value)
{
    handSensorsADCForce[idx] = value;
}

int ArmControlRight::getHandSensorsADCForce(int idx)
{
    return handSensorsADCForce[idx];
}

int ArmControlRight::getHandYawMotorADCCurrent() const
{
    return handYawMotorADCCurrent;
}

void ArmControlRight::setHandYawMotorADCCurrent(int value)
{
    handYawMotorADCCurrent = value;
}

int ArmControlRight::getForearmYawMotorADCCurrent() const
{
    return forearmYawMotorADCCurrent;
}

void ArmControlRight::setForearmYawMotorADCCurrent(int value)
{
    forearmYawMotorADCCurrent = value;
}

int ArmControlRight::getForearmYawADCPosition() const
{
    return forearmYawADCPosition;
}

void ArmControlRight::setForearmYawADCPosition(int value)
{
    forearmYawADCPosition = value;
}

int ArmControlRight::getHandPitchADCPosition() const
{
    return handPitchADCPosition;
}

void ArmControlRight::setHandPitchADCPosition(int value)
{
    handPitchADCPosition = value;
}

int ArmControlRight::getHandYawADCPosition() const
{
    return handYawADCPosition;
}

void ArmControlRight::setHandYawADCPosition(int value)
{
    handYawADCPosition = value;
}

int ArmControlRight::getLimbMotorADCCurrent() const
{
    return limbMotorADCCurrent;
}

void ArmControlRight::setLimbMotorADCCurrent(int value)
{
    limbMotorADCCurrent = value;
}

int ArmControlRight::getArmMotorADCCurrent() const
{
    return armMotorADCCurrent;
}

void ArmControlRight::setArmMotorADCCurrent(int value)
{
    armMotorADCCurrent = value;
}

int ArmControlRight::getForearmMotorADCCurrent() const
{
    return forearmMotorADCCurrent;
}

void ArmControlRight::setForearmMotorADCCurrent(int value)
{
    forearmMotorADCCurrent = value;
}

int ArmControlRight::getLimbADCPosition() const
{
    return limbADCPosition;
}

void ArmControlRight::setLimbADCPosition(int value)
{
    limbADCPosition = value;
}

int ArmControlRight::getArmADCPosition() const
{
    return armADCPosition;
}

void ArmControlRight::setArmADCPosition(int value)
{
    armADCPosition = value;
}

int ArmControlRight::getForearmADCPosition() const
{
    return forearmADCPosition;
}

void ArmControlRight::setForearmADCPosition(int value)
{
    forearmADCPosition = value;
}

bool ArmControlRight::getHandPitchMotorCurrentADC() const
{
    return handPitchMotorCurrentADC;
}

void ArmControlRight::setHandPitchMotorCurrentADC(bool value)
{
    handPitchMotorCurrentADC = value;
}

bool ArmControlRight::getHandPitchMotorCurrentTrack() const
{
    return handPitchMotorCurrentTrack;
}

void ArmControlRight::setHandPitchMotorCurrentTrack(bool value)
{
    handPitchMotorCurrentTrack = value;
}

bool ArmControlRight::getHandYawMotorCurrentADC() const
{
    return handYawMotorCurrentADC;
}

void ArmControlRight::setHandYawMotorCurrentADC(bool value)
{
    handYawMotorCurrentADC = value;
}

bool ArmControlRight::getHandYawMotorCurrentTrack() const
{
    return handYawMotorCurrentTrack;
}

void ArmControlRight::setHandYawMotorCurrentTrack(bool value)
{
    handYawMotorCurrentTrack = value;
}

bool ArmControlRight::getForearmYawMotorCurrentADC() const
{
    return forearmYawMotorCurrentADC;
}

void ArmControlRight::setForearmYawMotorCurrentADC(bool value)
{
    forearmYawMotorCurrentADC = value;
}

bool ArmControlRight::getForearmYawMotorCurrentTrack() const
{
    return forearmYawMotorCurrentTrack;
}

void ArmControlRight::setForearmYawMotorCurrentTrack(bool value)
{
    forearmYawMotorCurrentTrack = value;
}

bool ArmControlRight::getForearmYawPositionADC() const
{
    return forearmYawPositionADC;
}

void ArmControlRight::setForearmYawPositionADC(bool value)
{
    forearmYawPositionADC = value;
}

bool ArmControlRight::getForearmYawPositionTrack() const
{
    return forearmYawPositionTrack;
}

void ArmControlRight::setForearmYawPositionTrack(bool value)
{
    forearmYawPositionTrack = value;
}

bool ArmControlRight::getHandPitchPositionADC() const
{
    return handPitchPositionADC;
}

void ArmControlRight::setHandPitchPositionADC(bool value)
{
    handPitchPositionADC = value;
}

bool ArmControlRight::getHandPitchPositionTrack() const
{
    return handPitchPositionTrack;
}

void ArmControlRight::setHandPitchPositionTrack(bool value)
{
    handPitchPositionTrack = value;
}

bool ArmControlRight::getHandYawPositionADC() const
{
    return handYawPositionADC;
}

void ArmControlRight::setHandYawPositionADC(bool value)
{
    handYawPositionADC = value;
}

bool ArmControlRight::getHandYawPositionTrack() const
{
    return handYawPositionTrack;
}

void ArmControlRight::setHandYawPositionTrack(bool value)
{
    handYawPositionTrack = value;
}

bool ArmControlRight::getLimbMotorCurrentADC() const
{
    return limbMotorCurrentADC;
}

void ArmControlRight::setLimbMotorCurrentADC(bool value)
{
    limbMotorCurrentADC = value;
}

bool ArmControlRight::getLimbMotorCurrentTrack() const
{
    return limbMotorCurrentTrack;
}

void ArmControlRight::setLimbMotorCurrentTrack(bool value)
{
    limbMotorCurrentTrack = value;
}

bool ArmControlRight::getArmMotorCurrentADC() const
{
    return armMotorCurrentADC;
}

void ArmControlRight::setArmMotorCurrentADC(bool value)
{
    armMotorCurrentADC = value;
}

bool ArmControlRight::getArmMotorCurrentTrack() const
{
    return armMotorCurrentTrack;
}

void ArmControlRight::setArmMotorCurrentTrack(bool value)
{
    armMotorCurrentTrack = value;
}

bool ArmControlRight::getForearmMotorCurrentADC() const
{
    return forearmMotorCurrentADC;
}

void ArmControlRight::setForearmMotorCurrentADC(bool value)
{
    forearmMotorCurrentADC = value;
}

bool ArmControlRight::getForearmMotorCurrentTrack() const
{
    return forearmMotorCurrentTrack;
}

void ArmControlRight::setForearmMotorCurrentTrack(bool value)
{
    forearmMotorCurrentTrack = value;
}

bool ArmControlRight::getLimbPositionADC() const
{
    return limbPositionADC;
}

void ArmControlRight::setLimbPositionADC(bool value)
{
    limbPositionADC = value;
}

bool ArmControlRight::getLimbPositionTrack() const
{
    return limbPositionTrack;
}

void ArmControlRight::setLimbPositionTrack(bool value)
{
    limbPositionTrack = value;
}

bool ArmControlRight::getArmPositionADC() const
{
    return armPositionADC;
}

void ArmControlRight::setArmPositionADC(bool value)
{
    armPositionADC = value;
}

bool ArmControlRight::getArmPositionTrack() const
{
    return armPositionTrack;
}

void ArmControlRight::setArmPositionTrack(bool value)
{
    armPositionTrack = value;
}

bool ArmControlRight::getForearmPositionADC() const
{
    return forearmPositionADC;
}

void ArmControlRight::setForearmPositionADC(bool value)
{
    forearmPositionADC = value;
}

bool ArmControlRight::getForearmPositionTrack() const
{
    return forearmPositionTrack;
}

void ArmControlRight::setForearmPositionTrack(bool value)
{
    forearmPositionTrack = value;
}

bool ArmControlRight::getForearmYawDirection() const
{
    return forearmYawDirection;
}

void ArmControlRight::setForearmYawDirection(bool value)
{
    forearmYawDirection = value;
    if (value) //CW
    {
        sendCommand("FOREARMYAWCW");
    }
    else
    {
        sendCommand("FOREARMYAWCCW");
    }
}

bool ArmControlRight::getForearmRollResettingStepPosition() const
{
    return forearmRollResettingStepPosition;
}

void ArmControlRight::setForearmRollResettingStepPosition(bool value)
{
    forearmRollResettingStepPosition = value;
    setForearmRollDirection(true); //CW limit = 0 step position
}

bool ArmControlRight::getForearmRollCCWLimit() const
{
    return forearmRollCCWLimit;
}

void ArmControlRight::setForearmRollCCWLimit(bool value)
{
    forearmRollCCWLimit = value;
}

bool ArmControlRight::getForearmRollCWLimit() const
{
    return forearmRollCWLimit;
}

void ArmControlRight::setForearmRollCWLimit(bool value)
{
    forearmRollCWLimit = value;
}

int ArmControlRight::getForearmRollStepPosition() const
{
    return forearmRollStepPosition;
}

void ArmControlRight::setForearmRollStepPosition(int value)
{
    forearmRollStepPosition = value;
}

int ArmControlRight::getForearmRollStepSwitchDelay() const
{
    return forearmRollStepSwitchDelay;
}

void ArmControlRight::setForearmRollStepSwitchDelay(int value)
{
    forearmRollStepSwitchDelay = value;
}

int ArmControlRight::getForearmRollStepDelay() const
{
    return forearmRollStepDelay;
}

void ArmControlRight::setForearmRollStepDelay(int value)
{
    forearmRollStepDelay = value;
}

bool ArmControlRight::getForearmRollMotorActivated() const
{
    return forearmRollMotorActivated;
}

void ArmControlRight::setForearmRollMotorActivated(bool value)
{
    forearmRollMotorActivated = value;
}

bool ArmControlRight::getForearmRollMotorState() const
{
    return forearmRollMotorState;
}

void ArmControlRight::setForearmRollMotorState(bool value)
{
    forearmRollMotorState = value;
}

bool ArmControlRight::getForearmRollDirection() const
{
    return forearmRollDirection;
}

void ArmControlRight::setForearmRollDirection(bool value)
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

bool ArmControlRight::getHandPitchDirection() const
{
    return handPitchDirection;
}

void ArmControlRight::setHandPitchDirection(bool value)
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

bool ArmControlRight::getHandYawDirection() const
{
    return handYawDirection;
}

void ArmControlRight::setHandYawDirection(bool value)
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

int ArmControlRight::getRightArmMotorDutyPresetMax() const
{
    return rightArmMotorDutyPresetMax;
}

void ArmControlRight::setRightArmMotorDutyPresetMax(int value)
{
    rightArmMotorDutyPresetMax = value;
}

int ArmControlRight::getRightArmMotorDutyPresetMin() const
{
    return rightArmMotorDutyPresetMin;
}

void ArmControlRight::setRightArmMotorDutyPresetMin(int value)
{
    rightArmMotorDutyPresetMin = value;
}

int ArmControlRight::getRightArmMotorDutyPresetCur() const
{
    return rightArmMotorDutyPresetCur;
}

void ArmControlRight::setRightArmMotorDutyPresetCur(int value)
{
    rightArmMotorDutyPresetCur = value;
}

int ArmControlRight::getRightArmADCCurrent() const
{
    return rightArmADCCurrent;
}

void ArmControlRight::setRightArmADCCurrent(int value)
{
    rightArmADCCurrent = value;
}

int ArmControlRight::getRightArmADCPosition() const
{
    return rightArmADCPosition;
}

void ArmControlRight::setRightArmADCPosition(int value)
{
    rightArmADCPosition = value;
}

bool ArmControlRight::getRightArmMotorStop() const
{
    return rightArmMotorStop;
}

void ArmControlRight::setRightArmMotorStop(bool value)
{
    rightArmMotorStop = value;
}

bool ArmControlRight::getRightArmMotorActivated() const
{
    return rightArmMotorActivated;
}

void ArmControlRight::setRightArmMotorActivated(bool value)
{
    rightArmMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setRightArmMotorStop(false);
    }
}

bool ArmControlRight::getRightArmMotorMovementDirection() const
{
    return rightArmMotorMovementDirection;
}

bool ArmControlRight::setRightArmMotorMovementDirection(bool value)
{
    if (getRightArmMotorStop())
    {
        rightArmMotorMovementDirection = value;
        if (rightArmMotorMovementDirection) //down
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
        if (getRightArmMotorMovementDirection() == value)
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

bool ArmControlRight::getRightArmMotorDecelerating() const
{
    return rightArmMotorDecelerating;
}

void ArmControlRight::setRightArmMotorDecelerating(bool value)
{
    rightArmMotorDecelerating = value;
}

bool ArmControlRight::getRightArmMotorAccelerating() const
{
    return rightArmMotorAccelerating;
}

void ArmControlRight::setRightArmMotorAccelerating(bool value)
{
    rightArmMotorAccelerating = value;
}

int ArmControlRight::getRightArmMotorDeceleration() const
{
    return rightArmMotorDeceleration;
}

void ArmControlRight::setRightArmMotorDeceleration(int value)
{
    rightArmMotorDeceleration = value;
}

int ArmControlRight::getRightArmMotorAcceleration() const
{
    return rightArmMotorAcceleration;
}

void ArmControlRight::setRightArmMotorAcceleration(int value)
{
    rightArmMotorAcceleration = value;
}

int ArmControlRight::getRightArmMotorDutyMax() const
{
    return rightArmMotorDutyMax;
}

void ArmControlRight::setRightArmMotorDutyMax(int value)
{
    rightArmMotorDutyMax = value;
}

int ArmControlRight::getRightArmMotorDuty() const
{
    return rightArmMotorDuty;
}

void ArmControlRight::setRightArmMotorDuty(int value)
{
    rightArmMotorDuty = value;
}

int ArmControlRight::getRightForearmMotorDutyPresetMax() const
{
    return rightForearmMotorDutyPresetMax;
}

void ArmControlRight::setRightForearmMotorDutyPresetMax(int value)
{
    rightForearmMotorDutyPresetMax = value;
}

int ArmControlRight::getRightForearmMotorDutyPresetMin() const
{
    return rightForearmMotorDutyPresetMin;
}

void ArmControlRight::setRightForearmMotorDutyPresetMin(int value)
{
    rightForearmMotorDutyPresetMin = value;
}

int ArmControlRight::getRightForearmMotorDutyPresetCur() const
{
    return rightForearmMotorDutyPresetCur;
}

void ArmControlRight::setRightForearmMotorDutyPresetCur(int value)
{
    rightForearmMotorDutyPresetCur = value;
}

int ArmControlRight::getRightForearmADCCurrent() const
{
    return rightForearmADCCurrent;
}

void ArmControlRight::setRightForearmADCCurrent(int value)
{
    rightForearmADCCurrent = value;
}

int ArmControlRight::getRightForearmADCPosition() const
{
    return rightForearmADCPosition;
}

void ArmControlRight::setRightForearmADCPosition(int value)
{
    rightForearmADCPosition = value;
}

bool ArmControlRight::getRightForearmMotorStop() const
{
    return rightForearmMotorStop;
}

void ArmControlRight::setRightForearmMotorStop(bool value)
{
    rightForearmMotorStop = value;
}

bool ArmControlRight::getRightForearmMotorActivated() const
{
    return rightForearmMotorActivated;
}

void ArmControlRight::setRightForearmMotorActivated(bool value)
{
    rightForearmMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setRightForearmMotorStop(false);
    }
}

bool ArmControlRight::getRightForearmMotorMovementDirection() const
{
    return rightForearmMotorMovementDirection;
}

bool ArmControlRight::setRightForearmMotorMovementDirection(bool value)
{
    if (getRightForearmMotorStop())
    {
        rightForearmMotorMovementDirection = value;
        if (rightForearmMotorMovementDirection) //down
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
        if (getRightForearmMotorMovementDirection() == value)
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

bool ArmControlRight::getRightForearmMotorDecelerating() const
{
    return rightForearmMotorDecelerating;
}

void ArmControlRight::setRightForearmMotorDecelerating(bool value)
{
    rightForearmMotorDecelerating = value;
}

bool ArmControlRight::getRightForearmMotorAccelerating() const
{
    return rightForearmMotorAccelerating;
}

void ArmControlRight::setRightForearmMotorAccelerating(bool value)
{
    rightForearmMotorAccelerating = value;
}

int ArmControlRight::getRightForearmMotorDeceleration() const
{
    return rightForearmMotorDeceleration;
}

void ArmControlRight::setRightForearmMotorDeceleration(int value)
{
    rightForearmMotorDeceleration = value;
}

int ArmControlRight::getRightForearmMotorAcceleration() const
{
    return rightForearmMotorAcceleration;
}

void ArmControlRight::setRightForearmMotorAcceleration(int value)
{
    rightForearmMotorAcceleration = value;
}

int ArmControlRight::getRightForearmMotorDutyMax() const
{
    return rightForearmMotorDutyMax;
}

void ArmControlRight::setRightForearmMotorDutyMax(int value)
{
    rightForearmMotorDutyMax = value;
}

int ArmControlRight::getRightForearmMotorDuty() const
{
    return rightForearmMotorDuty;
}

void ArmControlRight::setRightForearmMotorDuty(int value)
{
    rightForearmMotorDuty = value;
    sendCommand(Valter::format_string("SETFOREARMLIFTUPDUTY#%d", rightForearmMotorDuty));
}
