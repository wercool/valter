#include "valter.h"
#include "armcontrolright.h"

#include "armcontrolright.h"

ArmControlRight *ArmControlRight::pArmControlRight = NULL;
bool ArmControlRight::instanceFlag = false;
const string ArmControlRight::controlDeviceId = "ARM-CONTROL-RIGHT";
const string ArmControlRight::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/arm-control-right-defaults";

ArmControlRight::ArmControlRight()
{
    Valter::log(ArmControlRight::controlDeviceId + " singleton initialized");
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

void ArmControlRight::initTcpInterface()
{
    tcpInterface = new TCPInterface();
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

                if (getForearmRollMotorState())
                {
                    if (response.compare("FA NO LIMIT") == 0)
                    {
                        setForearmRollCWLimit(false);
                        setForearmRollCCWLimit(false);
                        continue;
                    }
                    if (response.compare("FA CW LIMIT") == 0)
                    {
                        setForearmRollCWLimit(true);
                        continue;
                    }
                    if (response.compare("FA CCW LIMIT") == 0)
                    {
                        setForearmRollCCWLimit(true);
                        continue;
                    }
                }
                if (getForearmPositionTrack())
                {
                    if (response.find("FOREARM POS:") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        setForearmADCPosition(atoi(value_str.c_str()));
                        continue;
                    }
                }
                if (getArmPositionTrack())
                {
                    if (response.find("ARM POS:") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        setArmADCPosition(atoi(value_str.c_str()));
                        continue;
                    }
                }
                if (getLimbPositionTrack())
                {
                    if (response.find("LIMB POS:") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        setLimbADCPosition(atoi(value_str.c_str()));
                        continue;
                    }
                }
                if (getForearmMotorCurrentTrack())
                {
                    if (response.find("SENSOR:5") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setForearmMotorADCCurrent(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getArmMotorCurrentTrack())
                {
                    if (response.find("SENSOR:4") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setArmMotorADCCurrent(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getLimbMotorCurrentTrack())
                {
                    if (response.find("SENSOR:3") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setLimbMotorADCCurrent(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getHandYawPositionTrack())
                {
                    if (response.find("SENSOR:0") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setHandYawADCPosition(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getHandPitchPositionTrack())
                {
                    if (response.find("SENSOR:1") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setHandPitchADCPosition(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }                if (getForearmYawPositionTrack())
                {
                    if (response.find("SENSOR:2") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setForearmYawADCPosition(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getForearmYawMotorCurrentTrack())
                {
                    if (response.find("SENSOR:6") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setForearmYawMotorADCCurrent(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getHandYawMotorCurrentTrack())
                {
                    if (response.find("SENSOR:7") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setHandYawMotorADCCurrent(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }
                if (getHandPitchMotorCurrentTrack())
                {
                    if (response.find("SENSOR:8") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setHandPitchMotorADCCurrent(atoi(value_str_values[1].c_str()));
                        continue;
                    }
                }

                //track hand finger sensors
                bool trackHandFingerSensors = false;
                for (int idx = 0; idx < 13; idx++)
                {
                    if (getHandSensorsTrack(idx))
                    {
                        trackHandFingerSensors = true;
                    }
                }
                if (trackHandFingerSensors)
                {
                    if (response.find("HAND SENSOR:") != std::string::npos)
                    {
                        int value_str_pos = response.find_first_of(":") + 1;
                        string value_str = response.substr(value_str_pos);
                        vector<string>value_str_values = Valter::split(value_str, ',');
                        setHandSensorsADCForce(atoi(value_str_values[0].c_str()), atoi(value_str_values[1].c_str()));
                        continue;
                    }
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
    while (!stopAllProcesses)
    {
        if (!getRightArmMotorStop())
        {
            int curRightArmMotorDuty = getRightArmMotorDuty();
            bool acceleration, deceleration;
            if (getRightArmMotorActivated())
            {
                acceleration = getRightArmMotorAccelerating();
                if (curRightArmMotorDuty + getRightArmMotorAcceleration() < getRightArmMotorDutyMax())
                {
                    curRightArmMotorDuty += getRightArmMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curRightArmMotorDuty = getRightArmMotorDutyMax();
                    acceleration = false;
                }
                if (getRightArmMotorAccelerating())
                {
                    setRightArmMotorDuty(curRightArmMotorDuty);
                }
                setRightArmMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getRightArmMotorDecelerating();
                if (curRightArmMotorDuty - getRightArmMotorDeceleration() > 1)
                {
                    curRightArmMotorDuty -= getRightArmMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curRightArmMotorDuty = 1;
                    deceleration = false;
                    setRightArmMotorStop(true);
                }

                if (getRightArmMotorDecelerating())
                {
                    setRightArmMotorDuty(curRightArmMotorDuty);
                }
                setRightArmMotorDecelerating(deceleration);
                if (getRightArmMotorStop())
                {
                    sendCommand("ARMLIFTUPSTOP");
                    setRightArmMotorAccelerating(false);
                    setRightArmMotorDecelerating(false);
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

void ArmControlRight::rightLimbWorker()
{
    while (!stopAllProcesses)
    {
        if (!getRightLimbMotorStop())
        {
            int curRightLimbMotorDuty = getRightLimbMotorDuty();
            bool acceleration, deceleration;
            if (getRightLimbMotorActivated())
            {
                acceleration = getRightLimbMotorAccelerating();
                if (curRightLimbMotorDuty + getRightLimbMotorAcceleration() < getRightLimbMotorDutyMax())
                {
                    curRightLimbMotorDuty += getRightLimbMotorAcceleration();
                    acceleration = true;
                }
                else
                {
                    curRightLimbMotorDuty = getRightLimbMotorDutyMax();
                    acceleration = false;
                }
                if (getRightLimbMotorAccelerating())
                {
                    setRightLimbMotorDuty(curRightLimbMotorDuty);
                }
                setRightLimbMotorAccelerating(acceleration);
            }
            else
            {
                deceleration = getRightLimbMotorDecelerating();
                if (curRightLimbMotorDuty - getRightLimbMotorDeceleration() > 1)
                {
                    curRightLimbMotorDuty -= getRightLimbMotorDeceleration();
                    deceleration = true;
                }
                else
                {
                    curRightLimbMotorDuty = 1;
                    deceleration = false;
                    setRightLimbMotorStop(true);
                }

                if (getRightLimbMotorDecelerating())
                {
                    setRightLimbMotorDuty(curRightLimbMotorDuty);
                }
                setRightLimbMotorDecelerating(deceleration);
                if (getRightLimbMotorStop())
                {
                    sendCommand("LIMBLIFTUPSTOP");
                    setRightLimbMotorAccelerating(false);
                    setRightLimbMotorDecelerating(false);
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

void ArmControlRight::rightForearmRollWorker()
{
    while (!stopAllProcesses)
    {
        if (getForearmRollResettingStepPosition())
        {
            if (getForearmRollMotorState())
            {
                if (!getForearmRollCCWLimit())
                {
                    sendCommand("FOREARMROLL#250");
                    this_thread::sleep_for(std::chrono::microseconds(10000));
                }
                else
                {
                    setForearmRollStepPosition(0);
                    setForearmRollResettingStepPosition(false);
                }
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else
        {
            if (getForearmRollMotorActivated())
            {
                int forearmRollStepPosition = getForearmRollStepPosition();
                if (getForearmRollDirection())
                {
                    if (!getForearmRollCWLimit())
                    {
                        setForearmRollStepPosition(++forearmRollStepPosition);
                        sendCommand(Valter::format_string("FOREARMROLL#%d", getForearmRollStepSwitchDelay()));
                    }
                }
                else
                {
                    if (!getForearmRollCCWLimit())
                    {
                        setForearmRollStepPosition(--forearmRollStepPosition);
                        sendCommand(Valter::format_string("FOREARMROLL#%d", getForearmRollStepSwitchDelay()));
                    }
                }
                this_thread::sleep_for(std::chrono::microseconds(getForearmRollStepDelay()));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
}

void ArmControlRight::rightArmSensorReadingsWorker()
{
    while (!stopAllProcesses)
    {
        bool rightArmReadingActive = false;

        if (getForearmPositionTrack())
        {
            sendCommand("GETFOREARMPOS");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getArmPositionTrack())
        {
            sendCommand("GETARMPOS");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getLimbPositionTrack())
        {
            sendCommand("GETLIMBPOS");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getForearmMotorCurrentTrack())
        {
            sendCommand("SENSORCH5");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getArmMotorCurrentTrack())
        {
            sendCommand("SENSORCH4");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getLimbMotorCurrentTrack())
        {
            sendCommand("SENSORCH3");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getHandYawPositionTrack())
        {
            sendCommand("SENSORCH0");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getHandPitchPositionTrack())
        {
            sendCommand("SENSORCH1");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getForearmYawPositionTrack())
        {
            sendCommand("SENSORCH2");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getForearmYawMotorCurrentTrack())
        {
            sendCommand("SENSORCH6");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getHandYawMotorCurrentTrack())
        {
            sendCommand("SENSORCH7");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }
        if (getHandPitchMotorCurrentTrack())
        {
            sendCommand("SENSORCH8");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            rightArmReadingActive = true;
        }

        for (int idx = 0; idx < 13; idx++)
        {
            if (getHandSensorsTrack(idx))
            {
                sendCommand(Valter::format_string("ARMCH%d", idx));
                this_thread::sleep_for(std::chrono::milliseconds(10));
                sendCommand("GETHANDSENSOR");
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        if (rightArmReadingActive)
        {
            this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
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

void ArmControlRight::setRightForearmMotorDuty(int value)
{
    rightForearmMotorDuty = value;
    sendCommand(Valter::format_string("SETFOREARMLIFTUPDUTY#%d", rightForearmMotorDuty));
}
