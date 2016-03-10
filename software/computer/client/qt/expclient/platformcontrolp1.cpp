#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformcontrolp1.h"

PlatformControlP1 *PlatformControlP1::pPlatformControlP1 = NULL;
bool PlatformControlP1::instanceFlag = false;
const string PlatformControlP1::controlDeviceId = "PLATFORM-CONTROL-P1";
const string PlatformControlP1::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/platform-control-p1-defaults";

PlatformControlP1::PlatformControlP1()
{
    Valter::log(PlatformControlP1::controlDeviceId + " singleton started");
    loadDefaults();
    controlDeviceIsSet = false;
    resetValuesToDefault();
    chargerButtonPressStep = 0;
    new std::thread(&PlatformControlP1::platformMovementWorker, this);
    new std::thread(&PlatformControlP1::turretRotationWorker, this);
    new std::thread(&PlatformControlP1::scanFor220VACAvailable, this);
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
        getControlDevice()->setAutoReActivation(true);
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
    //platform motors
    leftMotorDirection                  = true;
    rightMotorDirection                 = true;
    leftMotorStop                       = true;
    rightMotorStop                      = true;
    platformEmergencyStop               = false;
    leftMotorActivated                  = false;
    rightMotorActivated                 = false;
    leftMotorAccelerating               = false;
    rightMotorAccelerating              = false;
    leftMotorDecelerating               = false;
    rightMotorDecelerating              = false;
    //turret motor
    turretEmergencyStop                 = false;
    turretMotorDirection                = true;
    turretMotorStop                     = true;
    turretMotorActivated                = false;
    turretMotorAccelerating             = false;
    turretMotorDecelerating             = false;

    curChannel1Input                    = 0;
    curChannel2Input                    = 0;
    //platform motors
    leftMotorDutyMax                    = 1;
    rightMotorDutyMax                   = 1;
    leftMotorDuty                       = 1;
    rightMotorDuty                      = 1;
    platformDeceleration                = 1;
    platformAcceleration                = 1;
    //turret motors
    turretDeceleration                  = 1;
    turretAcceleration                  = 1;
    turretMotorDutyMax                  = 1;
    turretMotorDuty                     = 1;

    chargerVoltageADC                   = 0;
    chargerVoltageVolts                 = 0.0;

    if (chargerButtonPressStep != 0)
    {
        setChargerMode(false);
    }
}

void PlatformControlP1::processMessagesQueueWorker()
{
    if (getControlDeviceIsSet())
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {
                string response = getControlDevice()->pullResponse();

                //execute identification of responses with priority
                if (getLeftMotorStop() && getRightMotorStop() && getTurretMotorStop())
                {
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

bool PlatformControlP1::preparePlatformMovement()
{
    bool canMove = true;

    canMove &= !getPower220VACAvailable();
    canMove &= !getMainAccumulatorRelayOnState();

    setScan220ACAvailable(false);

    return canMove;
}

void PlatformControlP1::platformMovementWorker()
{
    while (!stopAllProcesses)
    {
        if (!getPlatformEmergencyStop())
        {
            platformMovementDynamics();
        }
        else
        {
            int curDeceleration = getPlatformDeceleration();
            setPlatformDeceleration(10);
            setLeftMotorActivated(false);
            setRightMotorActivated(false);
            while (!getLeftMotorStop() || !getRightMotorStop())
            {
                platformMovementDynamics();
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            setPlatformEmergencyStop(false);
            setPlatformDeceleration(curDeceleration);
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PlatformControlP1::platformMovementDynamics()
{
    //Left Motor
    if (!getLeftMotorStop())
    {
        int curLeftMotorDuty = getLeftMotorDuty();
        bool acceleration, decceleration;
        if (getLeftMotorActivated())
        {
            acceleration = getLeftMotorAccelerating();
            if (curLeftMotorDuty + getPlatformAcceleration() < getLeftMotorDutyMax())
            {
                curLeftMotorDuty += getPlatformAcceleration();
                acceleration = true;
            }
            else
            {
                curLeftMotorDuty = getLeftMotorDutyMax();
                acceleration = false;
            }
            if (getLeftMotorAccelerating())
            {
                setLeftMotorDuty(curLeftMotorDuty);
                sendCommand(Valter::format_string("SETLEFTMOTORPWMDUTY#%d", curLeftMotorDuty));
            }
            setLeftMotorAccelerating(acceleration);
        }
        else
        {
            decceleration = getLeftMotorDecelerating();
            if (curLeftMotorDuty - getPlatformDeceleration() > 1)
            {
                curLeftMotorDuty -= getPlatformDeceleration();
                decceleration = true;
            }
            else
            {
                curLeftMotorDuty = 1;
                decceleration = false;
                setLeftMotorStop(true);
            }

            if (getLeftMotorDecelerating())
            {
                setLeftMotorDuty(curLeftMotorDuty);
                sendCommand(Valter::format_string("SETLEFTMOTORPWMDUTY#%d", curLeftMotorDuty));
            }
            setLeftMotorDecelerating(decceleration);
            if (getLeftMotorStop())
            {
                sendCommand("LEFTMOTORSTOP");
            }
        }
    }
    //Right Motor
    if (!getRightMotorStop())
    {
        int curRightMotorDuty = getRightMotorDuty();
        bool acceleration, decceleration;
        if (getRightMotorActivated())
        {
            acceleration = getRightMotorAccelerating();
            if (curRightMotorDuty + getPlatformAcceleration() < getRightMotorDutyMax())
            {
                curRightMotorDuty += getPlatformAcceleration();
                acceleration = true;
            }
            else
            {
                curRightMotorDuty = getRightMotorDutyMax();
                acceleration = false;
            }
            if (getRightMotorAccelerating())
            {
                setRightMotorDuty(curRightMotorDuty);
                sendCommand(Valter::format_string("SETRIGHTMOTORPWMDUTY#%d", curRightMotorDuty));
            }
            setRightMotorAccelerating(acceleration);
        }
        else
        {
            decceleration = getRightMotorDecelerating();
            if (curRightMotorDuty - getPlatformDeceleration() > 1)
            {
                curRightMotorDuty -= getPlatformDeceleration();
                decceleration = true;
            }
            else
            {
                curRightMotorDuty = 1;
                decceleration = false;
                setRightMotorStop(true);
            }

            if (getRightMotorDecelerating())
            {
                setRightMotorDuty(curRightMotorDuty);
                sendCommand(Valter::format_string("SETRIGHTMOTORPWMDUTY#%d", curRightMotorDuty));
            }
            setRightMotorDecelerating(decceleration);
            if (getRightMotorStop())
            {
                sendCommand("RIGHTMOTORSTOP");
            }
        }
    }
}

void PlatformControlP1::turretRotationWorker()
{
    while (!stopAllProcesses)
    {
        if (!getTurretEmergencyStop())
        {
            turretRotationDynamics();
        }
        else
        {
            int curDeceleration = getTurretAcceleration();
            setTurretAcceleration(10);
            setTurretMotorActivated(false);
            while (!getTurretMotorStop())
            {
                turretRotationDynamics();
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            setTurretEmergencyStop(false);
            setTurretAcceleration(curDeceleration);
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PlatformControlP1::turretRotationDynamics()
{
    if (!getTurretMotorStop())
    {
        int curTurretMotorDuty = getTurretMotorDuty();
        bool acceleration, decceleration;
        if (getTurretMotorActivated())
        {
            acceleration = getTurretMotorAccelerating();
            if (curTurretMotorDuty + getTurretAcceleration() < getTurretMotorDutyMax())
            {
                curTurretMotorDuty += getTurretAcceleration();
                acceleration = true;
            }
            else
            {
                curTurretMotorDuty = getTurretMotorDutyMax();
                acceleration = false;
            }
            if (getTurretMotorAccelerating())
            {
                setTurretMotorDuty(curTurretMotorDuty);
                sendCommand(Valter::format_string("SETTURRETMOTORPWMDUTY#%d", curTurretMotorDuty));
            }
            setTurretMotorAccelerating(acceleration);
        }
        else
        {
            decceleration = getTurretMotorDecelerating();
            if (curTurretMotorDuty - getTurretDeceleration() > 1)
            {
                curTurretMotorDuty -= getTurretDeceleration();
                decceleration = true;
            }
            else
            {
                curTurretMotorDuty = 1;
                decceleration = false;
                setTurretMotorStop(true);
            }

            if (getTurretMotorDecelerating())
            {
                setTurretMotorDuty(curTurretMotorDuty);
                sendCommand(Valter::format_string("SETTURRETMOTORPWMDUTY#%d", curTurretMotorDuty));
            }
            setTurretMotorDecelerating(decceleration);
            if (getTurretMotorStop())
            {
                sendCommand("TURRETMOTORSTOP");
            }
        }
    }
}
int PlatformControlP1::getTurretAccelerationPresetCur() const
{
    return turretAccelerationPresetCur;
}

void PlatformControlP1::setTurretAccelerationPresetCur(int value)
{
    turretAccelerationPresetCur = value;
}

int PlatformControlP1::getTurretAccelerationPresetMax() const
{
    return turretAccelerationPresetMax;
}

void PlatformControlP1::setTurretAccelerationPresetMax(int value)
{
    turretAccelerationPresetMax = value;
}

int PlatformControlP1::getTurretAccelerationPresetMin() const
{
    return turretAccelerationPresetMin;
}

void PlatformControlP1::setTurretAccelerationPresetMin(int value)
{
    turretAccelerationPresetMin = value;
}

int PlatformControlP1::getTurretDecelerationPresetCur() const
{
    return turretDecelerationPresetCur;
}

void PlatformControlP1::setTurretDecelerationPresetCur(int value)
{
    turretDecelerationPresetCur = value;
}

int PlatformControlP1::getTurretDecelerationPresetMax() const
{
    return turretDecelerationPresetMax;
}

void PlatformControlP1::setTurretDecelerationPresetMax(int value)
{
    turretDecelerationPresetMax = value;
}

int PlatformControlP1::getTurretDecelerationPresetMin() const
{
    return turretDecelerationPresetMin;
}

void PlatformControlP1::setTurretDecelerationPresetMin(int value)
{
    turretDecelerationPresetMin = value;
}

int PlatformControlP1::getTurretMotorDutyPresetCur() const
{
    return turretMotorDutyPresetCur;
}

void PlatformControlP1::setTurretMotorDutyPresetCur(int value)
{
    turretMotorDutyPresetCur = value;
}

int PlatformControlP1::getTurretMotorDutyPresetMax() const
{
    return turretMotorDutyPresetMax;
}

void PlatformControlP1::setTurretMotorDutyPresetMax(int value)
{
    turretMotorDutyPresetMax = value;
}

int PlatformControlP1::getTurretMotorDutyPresetMin() const
{
    return turretMotorDutyPresetMin;
}

void PlatformControlP1::setTurretMotorDutyPresetMin(int value)
{
    turretMotorDutyPresetMin = value;
}

int PlatformControlP1::getPlatformAccelerationPresetCur() const
{
    return platformAccelerationPresetCur;
}

void PlatformControlP1::setPlatformAccelerationPresetCur(int value)
{
    platformAccelerationPresetCur = value;
}

int PlatformControlP1::getPlatformAccelerationPresetMax() const
{
    return platformAccelerationPresetMax;
}

void PlatformControlP1::setPlatformAccelerationPresetMax(int value)
{
    platformAccelerationPresetMax = value;
}

int PlatformControlP1::getPlatformAccelerationPresetMin() const
{
    return platformAccelerationPresetMin;
}

void PlatformControlP1::setPlatformAccelerationPresetMin(int value)
{
    platformAccelerationPresetMin = value;
}

int PlatformControlP1::getPlatformDecelerationPresetCur() const
{
    return platformDecelerationPresetCur;
}

void PlatformControlP1::setPlatformDecelerationPresetCur(int value)
{
    platformDecelerationPresetCur = value;
}

int PlatformControlP1::getPlatformDecelerationPresetMax() const
{
    return platformDecelerationPresetMax;
}

void PlatformControlP1::setPlatformDecelerationPresetMax(int value)
{
    platformDecelerationPresetMax = value;
}

int PlatformControlP1::getPlatformDecelerationPresetMin() const
{
    return platformDecelerationPresetMin;
}

void PlatformControlP1::setPlatformDecelerationPresetMin(int value)
{
    platformDecelerationPresetMin = value;
}

int PlatformControlP1::getRightMotorDutyPresetCur() const
{
    return rightMotorDutyPresetCur;
}

void PlatformControlP1::setRightMotorDutyPresetCur(int value)
{
    rightMotorDutyPresetCur = value;
}

int PlatformControlP1::getRightMotorDutyPresetMax() const
{
    return rightMotorDutyPresetMax;
}

void PlatformControlP1::setRightMotorDutyPresetMax(int value)
{
    rightMotorDutyPresetMax = value;
}

int PlatformControlP1::getRightMotorDutyPresetMin() const
{
    return rightMotorDutyPresetMin;
}

void PlatformControlP1::setRightMotorDutyPresetMin(int value)
{
    rightMotorDutyPresetMin = value;
}

int PlatformControlP1::getLeftMotorDutyPresetCur() const
{
    return leftMotorDutyPresetCur;
}

void PlatformControlP1::setLeftMotorDutyPresetCur(int value)
{
    leftMotorDutyPresetCur = value;
}

int PlatformControlP1::getLeftMotorDutyPresetMax() const
{
    return leftMotorDutyPresetMax;
}

void PlatformControlP1::setLeftMotorDutyPresetMax(int value)
{
    leftMotorDutyPresetMax = value;
}

int PlatformControlP1::getLeftMotorDutyPresetMin() const
{
    return leftMotorDutyPresetMin;
}

void PlatformControlP1::setLeftMotorDutyPresetMin(int value)
{
    leftMotorDutyPresetMin = value;
}

bool PlatformControlP1::getTurretMotorDirection() const
{
    return turretMotorDirection;
}

bool PlatformControlP1::setTurretMotorDirection(bool value)
{
    if (getTurretMotorStop())
    {
        turretMotorDirection = value;
        if (turretMotorDirection) //right
        {
            sendCommand("TURRETMOTORCW");
        }
        else
        {
            sendCommand("TURRETMOTORCCW");
        }
        return true;
    }
    else
    {
        if (getTurretMotorDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool PlatformControlP1::getTurretEmergencyStop() const
{
    return turretEmergencyStop;
}

void PlatformControlP1::setTurretEmergencyStop(bool value)
{
    turretEmergencyStop = value;
}

bool PlatformControlP1::getTurretMotorDecelerating() const
{
    return turretMotorDecelerating;
}

void PlatformControlP1::setTurretMotorDecelerating(bool value)
{
    turretMotorDecelerating = value;
}

bool PlatformControlP1::getTurretMotorAccelerating() const
{
    return turretMotorAccelerating;
}

void PlatformControlP1::setTurretMotorAccelerating(bool value)
{
    turretMotorAccelerating = value;
}

int PlatformControlP1::getTurretAcceleration() const
{
    return turretAcceleration;
}

void PlatformControlP1::setTurretAcceleration(int value)
{
    turretAcceleration = value;
}

int PlatformControlP1::getTurretDeceleration() const
{
    return turretDeceleration;
}

void PlatformControlP1::setTurretDeceleration(int value)
{
    turretDeceleration = value;
}

bool PlatformControlP1::getTurretMotorActivated() const
{
    return turretMotorActivated;
}

void PlatformControlP1::setTurretMotorActivated(bool value)
{
    turretMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setTurretMotorStop(false);
    }
}

bool PlatformControlP1::getTurretMotorStop() const
{
    return turretMotorStop;
}

void PlatformControlP1::setTurretMotorStop(bool value)
{
    turretMotorStop = value;
}

int PlatformControlP1::getTurretMotorDuty() const
{
    return turretMotorDuty;
}

void PlatformControlP1::setTurretMotorDuty(int value)
{
    turretMotorDuty = value;
}

int PlatformControlP1::getTurretMotorDutyMax() const
{
    return turretMotorDutyMax;
}

void PlatformControlP1::setTurretMotorDutyMax(int value)
{
    turretMotorDutyMax = value;
}

bool PlatformControlP1::getRightMotorDecelerating() const
{
    return rightMotorDecelerating;
}

void PlatformControlP1::setRightMotorDecelerating(bool value)
{
    rightMotorDecelerating = value;
}

bool PlatformControlP1::getLeftMotorDecelerating() const
{
    return leftMotorDecelerating;
}

void PlatformControlP1::setLeftMotorDecelerating(bool value)
{
    leftMotorDecelerating = value;
}

bool PlatformControlP1::getRightMotorAccelerating() const
{
    return rightMotorAccelerating;
}

void PlatformControlP1::setRightMotorAccelerating(bool value)
{
    rightMotorAccelerating = value;
}

bool PlatformControlP1::getLeftMotorAccelerating() const
{
    return leftMotorAccelerating;
}

void PlatformControlP1::setLeftMotorAccelerating(bool value)
{
    leftMotorAccelerating = value;
}

int PlatformControlP1::getPlatformAcceleration() const
{
    return platformAcceleration;
}

void PlatformControlP1::setPlatformAcceleration(int value)
{
    platformAcceleration = value;
}

int PlatformControlP1::getPlatformDeceleration() const
{
    return platformDeceleration;
}

void PlatformControlP1::setPlatformDeceleration(int value)
{
    platformDeceleration = value;
}

bool PlatformControlP1::getRightMotorActivated() const
{
    return rightMotorActivated;
}

void PlatformControlP1::setRightMotorActivated(bool value)
{
    rightMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setRightMotorStop(false);
    }
}

bool PlatformControlP1::getLeftMotorActivated() const
{
    return leftMotorActivated;
}

void PlatformControlP1::setLeftMotorActivated(bool value)
{
    leftMotorActivated = value;
    if (value)
    {
        setLeftMotorStop(false);
    }
}


bool PlatformControlP1::getPlatformEmergencyStop() const
{
    return platformEmergencyStop;
}

void PlatformControlP1::setPlatformEmergencyStop(bool value)
{
    platformEmergencyStop = value;
}

bool PlatformControlP1::getRightMotorStop() const
{
    return rightMotorStop;
}

void PlatformControlP1::setRightMotorStop(bool value)
{
    rightMotorStop = value;
}

bool PlatformControlP1::getLeftMotorStop() const
{
    return leftMotorStop;
}

void PlatformControlP1::setLeftMotorStop(bool value)
{
    leftMotorStop = value;
}

bool PlatformControlP1::getRightMotorDirection() const
{
    return rightMotorDirection;
}

bool PlatformControlP1::setRightMotorDirection(bool value)
{
    if (getRightMotorStop())
    {
        rightMotorDirection = value;
        if (rightMotorDirection) //forward
        {
            sendCommand("RIGHTMOTORCCW");
        }
        else
        {
            sendCommand("RIGHTMOTORCW");
        }
        return true;
    }
    else
    {
        if (getRightMotorDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool PlatformControlP1::getLeftMotorDirection() const
{
    return leftMotorDirection;
}

bool PlatformControlP1::setLeftMotorDirection(bool value)
{
    if (getLeftMotorStop())
    {
        leftMotorDirection = value;
        if (leftMotorDirection) //forward
        {
            sendCommand("LEFTMOTORCW");
        }
        else
        {
            sendCommand("LEFTMOTORCCW");
        }
        return true;
    }
    else
    {
        if (getLeftMotorDirection() == value)
        {
            return true;
        }
        else
        {
            return false;
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

void PlatformControlP1::loadDefaults()
{
    ifstream defaultsFile(PlatformControlP1::defaultsFilePath);
    string line;
    while (getline(defaultsFile, line, '\n'))
    {
        char *lineStrPtr = Valter::stringToCharPtr(line);
        string defaultValueName(strtok(lineStrPtr, ":" ));
        string defaultValue(strtok(NULL, ":" ));
        addDefault(defaultValueName, defaultValue);
    }
    defaultsFile.close();

    string deFaultvalue;
    char *deFaultvaluePtr;
    int curValue;
    int minValue;
    int maxValue;

    //leftMotorDuty
    deFaultvalue = getDefault("leftMotorDuty");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftMotorDutyPresetMin(minValue);
    setLeftMotorDutyPresetMax(maxValue);
    setLeftMotorDutyPresetCur(curValue);
    setLeftMotorDutyMax(getLeftMotorDutyPresetCur());

    //rightMotorDuty
    deFaultvalue = getDefault("rightMotorDuty");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightMotorDutyPresetMin(minValue);
    setRightMotorDutyPresetMax(maxValue);
    setRightMotorDutyPresetCur(curValue);
    setRightMotorDutyMax(getRightMotorDutyPresetCur());

    //motorsDeceleration
    deFaultvalue = getDefault("motorsDeceleration");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setPlatformDecelerationPresetMin(minValue);
    setPlatformDecelerationPresetMax(maxValue);
    setPlatformDecelerationPresetCur(curValue);
    setPlatformDeceleration(getPlatformDecelerationPresetCur());

    //motorsAcceleration
    deFaultvalue = getDefault("motorsAcceleration");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setPlatformAccelerationPresetMin(minValue);
    setPlatformAccelerationPresetMax(maxValue);
    setPlatformAccelerationPresetCur(curValue);
    setPlatformAcceleration(getPlatformAccelerationPresetCur());

    //turretMotorDuty
    deFaultvalue = getDefault("turretMotorDuty");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setTurretMotorDutyPresetMin(minValue);
    setTurretMotorDutyPresetMax(maxValue);
    setTurretMotorDutyPresetCur(curValue);
    setTurretMotorDutyMax(getTurretMotorDutyPresetCur());

    //turretMotorDeceleration
    deFaultvalue = getDefault("turretMotorDeceleration");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setTurretDecelerationPresetMin(minValue);
    setTurretDecelerationPresetMax(maxValue);
    setTurretDecelerationPresetCur(curValue);
    setTurretDeceleration(getTurretDecelerationPresetCur());

    //turretMotorAcceleration
    deFaultvalue = getDefault("turretMotorAcceleration");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setTurretAccelerationPresetMin(minValue);
    setTurretAccelerationPresetMax(maxValue);
    setTurretAccelerationPresetCur(curValue);
    setTurretAcceleration(getTurretAccelerationPresetCur());
}
