#include "valter.h"
#include "armcontrolleft.h"

#include "armcontrolleft.utils.cpp"
#include "tcphandlers/armcontrolleft.tcphandler.cpp"

ArmControlLeft *ArmControlLeft::pArmControlLeft = NULL;
bool ArmControlLeft::instanceFlag = false;
const string ArmControlLeft::controlDeviceId = "ARM-CONTROL-LEFT";
const string ArmControlLeft::defaultsFilePath = "settings/arm-control-left-defaults";

ArmControlLeft::ArmControlLeft()
{
    Valter::log(ArmControlLeft::controlDeviceId + " singleton initialized");
    this->controlDeviceIsSet = false;

    initTcpInterface();

    resetToDefault();
    loadDefaults();

    new std::thread(&ArmControlLeft::leftForearmWorker, this);
    new std::thread(&ArmControlLeft::leftArmWorker, this);
    new std::thread(&ArmControlLeft::leftLimbWorker, this);
    new std::thread(&ArmControlLeft::leftForearmRollWorker, this);
    new std::thread(&ArmControlLeft::leftArmSensorReadingsWorker, this);
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

void ArmControlLeft::initTcpInterface()
{
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(33339);
        setTcpInterface(tcpInterface);
    }
}

void ArmControlLeft::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new ArmControlLeftTCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
}

void ArmControlLeft::processControlDeviceResponse(string response)
{

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
                }
                if (getForearmYawPositionTrack())
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

void ArmControlLeft::leftForearmRollWorker()
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

void ArmControlLeft::leftArmSensorReadingsWorker()
{
    while (!stopAllProcesses)
    {
        bool leftArmReadingActive = false;

        if (getForearmPositionTrack())
        {
            sendCommand("GETFOREARMPOS");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getArmPositionTrack())
        {
            sendCommand("GETARMPOS");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getLimbPositionTrack())
        {
            sendCommand("GETLIMBPOS");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getForearmMotorCurrentTrack())
        {
            sendCommand("SENSORCH5");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getArmMotorCurrentTrack())
        {
            sendCommand("SENSORCH4");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getLimbMotorCurrentTrack())
        {
            sendCommand("SENSORCH3");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getHandYawPositionTrack())
        {
            sendCommand("SENSORCH0");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getHandPitchPositionTrack())
        {
            sendCommand("SENSORCH1");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getForearmYawPositionTrack())
        {
            sendCommand("SENSORCH2");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getForearmYawMotorCurrentTrack())
        {
            sendCommand("SENSORCH6");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getHandYawMotorCurrentTrack())
        {
            sendCommand("SENSORCH7");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
        }
        if (getHandPitchMotorCurrentTrack())
        {
            sendCommand("SENSORCH8");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            sendCommand("GETSENSOR");
            this_thread::sleep_for(std::chrono::milliseconds(10));
            leftArmReadingActive = true;
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

        if (leftArmReadingActive)
        {
            this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool ArmControlLeft::getForearmYawDirection() const
{
    return forearmYawDirection;
}

void ArmControlLeft::setForearmYawDirection(bool value)
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

void ArmControlLeft::forearmYaw(bool state)
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

void ArmControlLeft::stopAllWatchers()
{
    sendCommand("STOPWATCHERS");
}

void ArmControlLeft::startAllWatchers()
{
    sendCommand("STARTWATCHERS");
}

void ArmControlLeft::setForearmRollResettingStepPosition(bool value)
{
    forearmRollResettingStepPosition = value;
    setForearmRollDirection(false); //CCW limit = 0 step position
}

void ArmControlLeft::turnArmLedsOnOff()
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

void ArmControlLeft::setLeftLimbMotorDuty(int value)
{
    leftLimbMotorDuty = value;
    sendCommand(Valter::format_string("SETLIMBLIFTUPDUTY#%d", leftLimbMotorDuty));
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

void ArmControlLeft::setLeftArmMotorDuty(int value)
{
    leftArmMotorDuty = value;
    sendCommand(Valter::format_string("SETARMLIFTUPDUTY#%d", leftArmMotorDuty));
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

void ArmControlLeft::setLeftForearmMotorDuty(int value)
{
    leftForearmMotorDuty = value;
    sendCommand(Valter::format_string("SETFOREARMLIFTUPDUTY#%d", leftForearmMotorDuty));
}