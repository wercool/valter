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
    new std::thread(&PlatformControlP1::additionalReadingsScanWorker, this);
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

    leftMotorCurrentRead                = false;
    rightMotorCurrentRead               = false;
    turretMotorCurrentRead              = false;

    leftWheelEncoderRead                = false;
    rightWheelEncoderRead               = false;
    leftWheelEncoderAutoreset           = false;
    rightWheelEncoderAutoreset          = false;
    leftWheelEncoderGetOnce              = false;
    rightWheelEncoderGetOnce             = false;

    turretPositionRead                  = false;
    turretPositionGetOnce               = false;

    mainAccumulatorVoltageRead          = false;
    leftAccumulatorVoltageRead          = false;
    rightAccumulatorVoltageRead         = false;
    mainAccumulatorAmperageTotalRead    = false;
    platformAmperageRead                = false;
    bodyAmperageRead                    = false;
    leftAccumulatorAmperageRead         = false;
    rightAccumulatorAmperageRead        = false;
    chargerVoltageRead                  = false;

    mainAccumulatorVoltageReadADCPreset         = false;
    leftAccumulatorVoltageReadADCPreset         = false;
    rightAccumulatorVoltageReadADCPreset        = false;
    mainAccumulatorAmperageTotalReadADCPreset   = false;
    platformAmperageReadADCPreset               = false;
    bodyAmperageReadADCPreset                   = false;
    leftAccumulatorAmperageReadADCPreset        = false;
    rightAccumulatorAmperageReadADCPreset       = false;
    chargerVoltageReadADCPreset                 = false;

    curChannel1Input                    = 0;
    curChannel2Input                    = 0;
    //platform motors
    leftMotorDutyMax                    = 1;
    rightMotorDutyMax                   = 1;
    leftMotorDuty                       = 1;
    rightMotorDuty                      = 1;
    platformDeceleration                = 1;
    platformAcceleration                = 1;
    //turret motor
    turretDeceleration                  = 1;
    turretAcceleration                  = 1;
    turretMotorDutyMax                  = 1;
    turretMotorDuty                     = 1;

    motorsPWMFrequncy                   = 8000;
    leftMotorCurrentADC                 = 0;
    rightMotorCurrentADC                = 0;
    turretMotorCurrentADC               = 0;
    leftMotorCurrentAmps                = 0.0;
    rightMotorCurrentAmps               = 0.0;
    turretMotorCurrentAmps              = 0.0;

    leftWheelEncoder                    = 0;
    rightWheelEncoder                   = 0;

    turretPositionADC                   = 0;
    turretPositionDeg                   = 0.0;
    turretPositionRad                   = 0.0;

    chargerVoltageADC                   = 0;
    chargerVoltageVolts                 = 0.0;

    mainAccumulatorVoltageADC           = 0;
    leftAccumulatorVoltageADC           = 0;
    rightAccumulatorVoltageADC          = 0;
    mainAccumulatorAmperageTotalADC     = 0;
    platformAmperageADC                 = 0;
    bodyAmperageADC                     = 0;
    leftAccumulatorAmperageADC          = 0;
    rightAccumulatorAmperageADC         = 0;

    mainAccumulatorVoltageVolts         = 0.0;
    leftAccumulatorVoltageVolts         = 0.0;
    rightAccumulatorVoltageVolts        = 0.0;
    mainAccumulatorAmperageTotalAmps    = 0.0;
    platformAmperageAmps                = 0.0;
    bodyAmperageAmps                    = 0.0;
    leftAccumulatorAmperageAmps         = 0.0;
    rightAccumulatorAmperageAmps        = 0.0;

    additionalReadingsDelayPresetMin    = 5;
    additionalReadingsDelayPresetMax    = 1000;
    additionalReadingsDelayCur          = 250;

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
                else
                {
                    //response template
                    //ALLCURREADINGS:{IN1 CH#,READING;IN2 CH#,READING;TURRET POSITION;LEFT MOTOR CURRENT;RIGHT MOTOR CURRENT;TURRET MOTOR CURRENT;LEFT MOTOR COUNTER;RIGHT MOTOR COUNTER}
                    if (response.find("ALLCURREADINGS:") !=std::string::npos) //composed readings
                    {
                        int substr_pos = response.find(":");
                        string value_str = response.substr(substr_pos);
                        value_str.replace(0,1,"");
                        value_str.replace(value_str.length(),1,"");
                        vector<string>value_str_values = Valter::split(value_str, ';');

                        int turretPosition = atoi(Valter::stringToCharPtr(value_str_values[3]));

                        int leftMotorCurrent = atoi(Valter::stringToCharPtr(value_str_values[3]));
                        int rightMotorCurrent = atoi(Valter::stringToCharPtr(value_str_values[4]));
                        int turretMotorCurrent = atoi(Valter::stringToCharPtr(value_str_values[5]));

                        int leftWheelEncoder = atoi(Valter::stringToCharPtr(value_str_values[6]));
                        int rightWheelEncoder = atoi(Valter::stringToCharPtr(value_str_values[7]));

                        if (!getLeftMotorStop())
                        {
                            setLeftMotorCurrentADC(leftMotorCurrent);
                        }
                        if (!getRightMotorStop())
                        {
                            setRightMotorCurrentADC(rightMotorCurrent);
                        }
                        if (!getTurretMotorStop())
                        {
                            setTurretMotorCurrentADC(turretMotorCurrent);
                        }
                        if (getLeftWheelEncoderRead())
                        {
                            setLeftWheelEncoder(leftWheelEncoder);
                        }
                        if (getRightWheelEncoderRead())
                        {
                            setRightWheelEncoder(rightWheelEncoder);
                        }

                        if (getTurretPositionRead())
                        {
                            setTurretPositionADC(turretPosition);
                        }
                        continue;
                    }
                }
                //get when requested inspite motors is not stopped
                if (getLeftWheelEncoderGetOnce())
                {
                    if (response.find("LEFT MOTOR COUNTER: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setLeftWheelEncoder(value);
                        setLeftWheelEncoderGetOnce(false);
                        continue;
                    }
                }
                if (getRightWheelEncoderGetOnce())
                {
                    if (response.find("RIGHT MOTOR COUNTER: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setRightWheelEncoder(value);
                        setRightWheelEncoderGetOnce(false);
                        continue;
                    }
                }
                if (getTurretPositionGetOnce())
                {
                    if (response.find("TURRET: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setTurretPositionADC(value);
                        setTurretPositionGetOnce(false);
                        continue;
                    }
                }
                if (getMainAccumulatorVoltageRead())
                {
                    //main accumulator voltage
                    if (response.find("INPUT1 CHANNEL [0]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setMainAccumulatorVoltageADC(value);
                        continue;
                    }
                }
                if (getLeftAccumulatorVoltageRead())
                {
                    //left accumulator voltage
                    if (response.find("INPUT1 CHANNEL [1]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setLeftAccumulatorVoltageADC(value);
                        continue;
                    }
                }
                if (getRightAccumulatorVoltageRead())
                {
                    //right accumulator voltage
                    if (response.find("INPUT1 CHANNEL [2]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setRightAccumulatorVoltageADC(value);
                        continue;
                    }
                }
                if (getMainAccumulatorAmperageTotalRead())
                {
                    //main accumulator amperage total
                    if (response.find("INPUT1 CHANNEL [3]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setMainAccumulatorAmperageTotalADC(value);
                        continue;
                    }
                }
                if (getPlatformAmperageRead())
                {
                    //main accumulator amperage bottom (platform)
                    if (response.find("INPUT1 CHANNEL [4]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setPlatformAmperageADC(value);
                        continue;
                    }
                }
                if (getBodyAmperageRead())
                {
                    //main accumulator amperage top (body)
                    if (response.find("INPUT1 CHANNEL [5]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setBodyAmperageADC(value);
                        continue;
                    }
                }
                if (getLeftAccumulatorAmperageRead())
                {
                    //left accumulator amperage
                    if (response.find("INPUT1 CHANNEL [6]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setLeftAccumulatorAmperageADC(value);
                        continue;
                    }
                }
                if (getRightAccumulatorAmperageRead())
                {
                    //right accumulator amperage
                    if (response.find("INPUT1 CHANNEL [7]: ") !=std::string::npos)
                    {
                        int substr_pos = response.find(":") + 1;
                        string value_str = response.substr(substr_pos);
                        int value = atoi(value_str.c_str());
                        setRightAccumulatorAmperageADC(value);
                        continue;
                    }
                }
                if (getChargerVoltageRead())
                {
                    //charger connected (charger voltage)
                    if (response.find("INPUT1 CHANNEL [8]: ") !=std::string::npos)
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

    if (getLeftWheelEncoderAutoreset())
    {
        resetLeftWheelEncoder();
    }
    if (getRightWheelEncoderAutoreset())
    {
        resetRightWheelEncoder();
    }

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
            setPlatformDeceleration(getPlatformDecelerationPresetMax());
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
    if (!getLeftMotorStop() || !getRightMotorStop())
    {
        if (getLeftMotorCurrentRead() || getRightMotorCurrentRead())
        {
            sendCommand("GETALLCURREADINGS");
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
            setTurretDeceleration(getTurretDecelerationPresetMax());
            setTurretMotorActivated(false);
            while (!getTurretMotorStop())
            {
                turretRotationDynamics();
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            setTurretEmergencyStop(false);
            setTurretDeceleration(curDeceleration);
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
        if (getTurretMotorCurrentRead())
        {
            sendCommand("GETALLCURREADINGS");
        }
    }
}

void PlatformControlP1::additionalReadingsScanWorker()
{
    while (!stopAllProcesses)
    {
        if (getMainAccumulatorVoltageRead())
        {
            //main accumulator voltage
            sendCommand("SETINPUT1CHANNEL0");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getLeftAccumulatorVoltageRead())
        {
            //left accumulator voltage
            sendCommand("SETINPUT1CHANNEL1");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getRightAccumulatorVoltageRead())
        {
            //right accumulator voltage
            sendCommand("SETINPUT1CHANNEL2");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getMainAccumulatorAmperageTotalRead())
        {
            //main accumulator amperage total
            sendCommand("SETINPUT1CHANNEL3");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getPlatformAmperageRead())
        {
            //main accumulator amperage bottom
            sendCommand("SETINPUT1CHANNEL4");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getBodyAmperageRead())
        {
            //main accumulator amperage top
            sendCommand("SETINPUT1CHANNEL5");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getLeftAccumulatorAmperageRead())
        {
            //left accumulator amperage
            sendCommand("SETINPUT1CHANNEL6");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getRightAccumulatorAmperageRead())
        {
            //right accumulator amperage
            sendCommand("SETINPUT1CHANNEL7");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        if (getChargerVoltageRead())
        {
            //charger connected (charger voltage)
            sendCommand("SETINPUT1CHANNEL8");
            sendCommand("GETINPUT1");
            this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
        }
        this_thread::sleep_for(std::chrono::milliseconds(getAdditionalReadingsDelayCur()));
    }
}

int PlatformControlP1::getAdditionalReadingsDelayCur() const
{
    return additionalReadingsDelayCur;
}

void PlatformControlP1::setAdditionalReadingsDelayCur(int value)
{
    additionalReadingsDelayCur = value;
}

int PlatformControlP1::getAdditionalReadingsDelayPresetMax() const
{
    return additionalReadingsDelayPresetMax;
}

void PlatformControlP1::setAdditionalReadingsDelayPresetMax(int value)
{
    additionalReadingsDelayPresetMax = value;
}

int PlatformControlP1::getAdditionalReadingsDelayPresetMin() const
{
    return additionalReadingsDelayPresetMin;
}

void PlatformControlP1::setAdditionalReadingsDelayPresetMin(int value)
{
    additionalReadingsDelayPresetMin = value;
}
bool PlatformControlP1::getChargerVoltageReadADCPreset() const
{
    return chargerVoltageReadADCPreset;
}

void PlatformControlP1::setChargerVoltageReadADCPreset(bool value)
{
    chargerVoltageReadADCPreset = value;
}

bool PlatformControlP1::getRightAccumulatorAmperageReadADCPreset() const
{
    return rightAccumulatorAmperageReadADCPreset;
}

void PlatformControlP1::setRightAccumulatorAmperageReadADCPreset(bool value)
{
    rightAccumulatorAmperageReadADCPreset = value;
}

bool PlatformControlP1::getLeftAccumulatorAmperageReadADCPreset() const
{
    return leftAccumulatorAmperageReadADCPreset;
}

void PlatformControlP1::setLeftAccumulatorAmperageReadADCPreset(bool value)
{
    leftAccumulatorAmperageReadADCPreset = value;
}

bool PlatformControlP1::getBodyAmperageReadADCPreset() const
{
    return bodyAmperageReadADCPreset;
}

void PlatformControlP1::setBodyAmperageReadADCPreset(bool value)
{
    bodyAmperageReadADCPreset = value;
}

bool PlatformControlP1::getPlatformAmperageReadADCPreset() const
{
    return platformAmperageReadADCPreset;
}

void PlatformControlP1::setPlatformAmperageReadADCPreset(bool value)
{
    platformAmperageReadADCPreset = value;
}

bool PlatformControlP1::getMainAccumulatorAmperageTotalReadADCPreset() const
{
    return mainAccumulatorAmperageTotalReadADCPreset;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalReadADCPreset(bool value)
{
    mainAccumulatorAmperageTotalReadADCPreset = value;
}

bool PlatformControlP1::getRightAccumulatorVoltageReadADCPreset() const
{
    return rightAccumulatorVoltageReadADCPreset;
}

void PlatformControlP1::setRightAccumulatorVoltageReadADCPreset(bool value)
{
    rightAccumulatorVoltageReadADCPreset = value;
}

bool PlatformControlP1::getLeftAccumulatorVoltageReadADCPreset() const
{
    return leftAccumulatorVoltageReadADCPreset;
}

void PlatformControlP1::setLeftAccumulatorVoltageReadADCPreset(bool value)
{
    leftAccumulatorVoltageReadADCPreset = value;
}

bool PlatformControlP1::getMainAccumulatorVoltageReadADCPreset() const
{
    return mainAccumulatorVoltageReadADCPreset;
}

void PlatformControlP1::setMainAccumulatorVoltageReadADCPreset(bool value)
{
    mainAccumulatorVoltageReadADCPreset = value;
}

bool PlatformControlP1::getChargerVoltageRead() const
{
    return chargerVoltageRead;
}

void PlatformControlP1::setChargerVoltageRead(bool value)
{
    chargerVoltageRead = value;
}

bool PlatformControlP1::getRightAccumulatorAmperageRead() const
{
    return rightAccumulatorAmperageRead;
}

void PlatformControlP1::setRightAccumulatorAmperageRead(bool value)
{
    rightAccumulatorAmperageRead = value;
}

bool PlatformControlP1::getLeftAccumulatorAmperageRead() const
{
    return leftAccumulatorAmperageRead;
}

void PlatformControlP1::setLeftAccumulatorAmperageRead(bool value)
{
    leftAccumulatorAmperageRead = value;
}

bool PlatformControlP1::getBodyAmperageRead() const
{
    return bodyAmperageRead;
}

void PlatformControlP1::setBodyAmperageRead(bool value)
{
    bodyAmperageRead = value;
}

bool PlatformControlP1::getPlatformAmperageRead() const
{
    return platformAmperageRead;
}

void PlatformControlP1::setPlatformAmperageRead(bool value)
{
    platformAmperageRead = value;
}

bool PlatformControlP1::getMainAccumulatorAmperageTotalRead() const
{
    return mainAccumulatorAmperageTotalRead;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalRead(bool value)
{
    mainAccumulatorAmperageTotalRead = value;
}

bool PlatformControlP1::getRightAccumulatorVoltageRead() const
{
    return rightAccumulatorVoltageRead;
}

void PlatformControlP1::setRightAccumulatorVoltageRead(bool value)
{
    rightAccumulatorVoltageRead = value;
}

bool PlatformControlP1::getLeftAccumulatorVoltageRead() const
{
    return leftAccumulatorVoltageRead;
}

void PlatformControlP1::setLeftAccumulatorVoltageRead(bool value)
{
    leftAccumulatorVoltageRead = value;
}

bool PlatformControlP1::getMainAccumulatorVoltageRead() const
{
    return mainAccumulatorVoltageRead;
}

void PlatformControlP1::setMainAccumulatorVoltageRead(bool value)
{
    mainAccumulatorVoltageRead = value;
}

float PlatformControlP1::getRightAccumulatorAmperageAmps() const
{
    return rightAccumulatorAmperageAmps;
}

void PlatformControlP1::setRightAccumulatorAmperageAmps(float value)
{
    rightAccumulatorAmperageAmps = value;
}

float PlatformControlP1::getLeftAccumulatorAmperageAmps() const
{
    return leftAccumulatorAmperageAmps;
}

void PlatformControlP1::setLeftAccumulatorAmperageAmps(float value)
{
    leftAccumulatorAmperageAmps = value;
}

float PlatformControlP1::getBodyAmperageAmps() const
{
    return bodyAmperageAmps;
}

void PlatformControlP1::setBodyAmperageAmps(float value)
{
    bodyAmperageAmps = value;
}

float PlatformControlP1::getPlatformAmperageAmps() const
{
    return platformAmperageAmps;
}

void PlatformControlP1::setPlatformAmperageAmps(float value)
{
    platformAmperageAmps = value;
}

float PlatformControlP1::getMainAccumulatorAmperageTotalAmps() const
{
    return mainAccumulatorAmperageTotalAmps;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalAmps(float value)
{
    mainAccumulatorAmperageTotalAmps = value;
}

float PlatformControlP1::getRightAccumulatorVoltageVolts() const
{
    return rightAccumulatorVoltageVolts;
}

void PlatformControlP1::setRightAccumulatorVoltageVolts(float value)
{
    rightAccumulatorVoltageVolts = value;
}

float PlatformControlP1::getLeftAccumulatorVoltageVolts() const
{
    return leftAccumulatorVoltageVolts;
}

void PlatformControlP1::setLeftAccumulatorVoltageVolts(float value)
{
    leftAccumulatorVoltageVolts = value;
}

float PlatformControlP1::getMainAccumulatorVoltageVolts() const
{
    return mainAccumulatorVoltageVolts;
}

void PlatformControlP1::setMainAccumulatorVoltageVolts(float value)
{
    mainAccumulatorVoltageVolts = value;
}

int PlatformControlP1::getRightAccumulatorAmperageADC() const
{
    return rightAccumulatorAmperageADC;
}

void PlatformControlP1::setRightAccumulatorAmperageADC(int value)
{
    rightAccumulatorAmperageADC = value;
    setRightAccumulatorAmperageAmps((float)value / (float)10);
}

int PlatformControlP1::getLeftAccumulatorAmperageADC() const
{
    return leftAccumulatorAmperageADC;
}

void PlatformControlP1::setLeftAccumulatorAmperageADC(int value)
{
    leftAccumulatorAmperageADC = value;
    setLeftAccumulatorAmperageAmps((float)value / (float)10);
}

int PlatformControlP1::getBodyAmperageADC() const
{
    return bodyAmperageADC;
}

void PlatformControlP1::setBodyAmperageADC(int value)
{
    bodyAmperageADC = value;
    setBodyAmperageAmps((float)value / (float)10);
}

int PlatformControlP1::getPlatformAmperageADC() const
{
    return platformAmperageADC;
}

void PlatformControlP1::setPlatformAmperageADC(int value)
{
    platformAmperageADC = value;
    setPlatformAmperageAmps((float)value / (float)10);
}

int PlatformControlP1::getMainAccumulatorAmperageTotalADC() const
{
    return mainAccumulatorAmperageTotalADC;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalADC(int value)
{
    mainAccumulatorAmperageTotalADC = value;
    setMainAccumulatorAmperageTotalAmps((float)value / (float)10);
}

int PlatformControlP1::getRightAccumulatorVoltageADC() const
{
    return rightAccumulatorVoltageADC;
}

void PlatformControlP1::setRightAccumulatorVoltageADC(int value)
{
    rightAccumulatorVoltageADC = value;
    setRightAccumulatorVoltageVolts((float)value / (float)10);
}

int PlatformControlP1::getLeftAccumulatorVoltageADC() const
{
    return leftAccumulatorVoltageADC;
}

void PlatformControlP1::setLeftAccumulatorVoltageADC(int value)
{
    leftAccumulatorVoltageADC = value;
    setLeftAccumulatorVoltageVolts((float)value / (float)10);
}

int PlatformControlP1::getMainAccumulatorVoltageADC() const
{
    return mainAccumulatorVoltageADC;
}

void PlatformControlP1::setMainAccumulatorVoltageADC(int value)
{
    mainAccumulatorVoltageADC = value;
    setMainAccumulatorVoltageVolts((float)value / (float)10);
}

bool PlatformControlP1::getTurretPositionGetOnce() const
{
    return turretPositionGetOnce;
}

void PlatformControlP1::setTurretPositionGetOnce(bool value)
{
    turretPositionGetOnce = value;
}

bool PlatformControlP1::getRightWheelEncoderGetOnce() const
{
    return rightWheelEncoderGetOnce;
}

void PlatformControlP1::setRightWheelEncoderGetOnce(bool value)
{
    rightWheelEncoderGetOnce = value;
}

bool PlatformControlP1::getLeftWheelEncoderGetOnce() const
{
    return leftWheelEncoderGetOnce;
}

void PlatformControlP1::setLeftWheelEncoderGetOnce(bool value)
{
    leftWheelEncoderGetOnce = value;
}

float PlatformControlP1::getTurretPositionRad() const
{
    return turretPositionRad;
}

void PlatformControlP1::setTurretPositionRad(float value)
{
    turretPositionRad = value;
}

float PlatformControlP1::getTurretPositionDeg() const
{
    return turretPositionDeg;
}

void PlatformControlP1::setTurretPositionDeg(float value)
{
    turretPositionDeg = value;
}

int PlatformControlP1::getTurretPositionADC() const
{
    return turretPositionADC;
}

void PlatformControlP1::setTurretPositionADC(int value)
{
    turretPositionADC = value;
    setTurretPositionDeg((float)value / (float)10);
    setTurretPositionRad((float)value / (float)10);
}

bool PlatformControlP1::getTurretPositionRead() const
{
    return turretPositionRead;
}

void PlatformControlP1::setTurretPositionRead(bool value)
{
    turretPositionRead = value;
}


bool PlatformControlP1::getRightWheelEncoderAutoreset() const
{
    return rightWheelEncoderAutoreset;
}

void PlatformControlP1::setRightWheelEncoderAutoreset(bool value)
{
    rightWheelEncoderAutoreset = value;
}

void PlatformControlP1::resetLeftWheelEncoder()
{
    sendCommand("RESETLEFTMOTORCOUNTER");
}

void PlatformControlP1::resetRightWheelEncoder()
{
    sendCommand("RESETRIGHTMOTORCOUNTER");
}

bool PlatformControlP1::getLeftWheelEncoderAutoreset() const
{
    return leftWheelEncoderAutoreset;
}

void PlatformControlP1::setLeftWheelEncoderAutoreset(bool value)
{
    leftWheelEncoderAutoreset = value;
}

bool PlatformControlP1::getRightWheelEncoderRead() const
{
    return rightWheelEncoderRead;
}

void PlatformControlP1::setRightWheelEncoderRead(bool value)
{
    rightWheelEncoderRead = value;
}

bool PlatformControlP1::getLeftWheelEncoderRead() const
{
    return leftWheelEncoderRead;
}

void PlatformControlP1::setLeftWheelEncoderRead(bool value)
{
    leftWheelEncoderRead = value;
}

int PlatformControlP1::getRightWheelEncoder() const
{
    return rightWheelEncoder;
}

void PlatformControlP1::setRightWheelEncoder(int value)
{
    rightWheelEncoder = value;
}

int PlatformControlP1::getLeftWheelEncoder() const
{
    return leftWheelEncoder;
}

void PlatformControlP1::setLeftWheelEncoder(int value)
{
    leftWheelEncoder = value;
}
bool PlatformControlP1::getTurretMotorCurrentRead() const
{
    return turretMotorCurrentRead;
}

void PlatformControlP1::setTurretMotorCurrentRead(bool value)
{
    turretMotorCurrentRead = value;
}

bool PlatformControlP1::getRightMotorCurrentRead() const
{
    return rightMotorCurrentRead;
}

void PlatformControlP1::setRightMotorCurrentRead(bool value)
{
    rightMotorCurrentRead = value;
}

bool PlatformControlP1::getLeftMotorCurrentRead() const
{
    return leftMotorCurrentRead;
}

void PlatformControlP1::setLeftMotorCurrentRead(bool value)
{
    leftMotorCurrentRead = value;
}

float PlatformControlP1::getTurretMotorCurrentAmps() const
{
    return turretMotorCurrentAmps;
}

void PlatformControlP1::setTurretMotorCurrentAmps(float value)
{
    turretMotorCurrentAmps = value;
}

float PlatformControlP1::getRightMotorCurrentAmps() const
{
    return rightMotorCurrentAmps;
}

void PlatformControlP1::setRightMotorCurrentAmps(float value)
{
    rightMotorCurrentAmps = value;
}

float PlatformControlP1::getLeftMotorCurrentAmps() const
{
    return leftMotorCurrentAmps;
}

void PlatformControlP1::setLeftMotorCurrentAmps(float value)
{
    leftMotorCurrentAmps = value;
}

int PlatformControlP1::getRightMotorCurrentADC() const
{
    return rightMotorCurrentADC;
}

void PlatformControlP1::setRightMotorCurrentADC(int value)
{
    rightMotorCurrentADC = value;
    setRightMotorCurrentAmps((float)value / (float)10);
}

int PlatformControlP1::getTurretMotorCurrentADC() const
{
    return turretMotorCurrentADC;
}

void PlatformControlP1::setTurretMotorCurrentADC(int value)
{
    turretMotorCurrentADC = value;
    setTurretMotorCurrentAmps((float)value / (float)10);
}

int PlatformControlP1::getLeftMotorCurrentADC() const
{
    return leftMotorCurrentADC;
}

void PlatformControlP1::setLeftMotorCurrentADC(int value)
{
    leftMotorCurrentADC = value;
    setLeftMotorCurrentAmps((float)value / (float)10);
}


int PlatformControlP1::getMotorsPWMFrequncy() const
{
    return motorsPWMFrequncy;
}

void PlatformControlP1::setMotorsPWMFrequncy(int value)
{
    motorsPWMFrequncy = value;
    sendCommand(Valter::format_string("SETPWMFREQUENCY#%d", motorsPWMFrequncy));
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
    if (turretMotorStop)
    {
        setTurretMotorCurrentADC(0);
    }
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
    if (rightMotorStop)
    {
        setRightMotorCurrentADC(0);
    }
}

bool PlatformControlP1::getLeftMotorStop() const
{
    return leftMotorStop;
}

void PlatformControlP1::setLeftMotorStop(bool value)
{
    leftMotorStop = value;
    if (leftMotorStop)
    {
        setLeftMotorCurrentADC(0);
    }
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
    setChargerVoltageVolts((float)value / (float)10);
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
        if (line.substr(0, 2).compare("//") != 0)
        {
            char *lineStrPtr = Valter::stringToCharPtr(line);
            string defaultValueName(strtok(lineStrPtr, ":" ));
            string defaultValue(strtok(NULL, ":" ));
            addDefault(defaultValueName, defaultValue);
        }
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

    //trackLeftMotorCurrent
    deFaultvalue = getDefault("trackLeftMotorCurrent");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setLeftMotorCurrentRead((atoi(deFaultvaluePtr)) ? true : false);

    //trackRightMotorCurrent
    deFaultvalue = getDefault("trackRightMotorCurrent");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setRightMotorCurrentRead((atoi(deFaultvaluePtr)) ? true : false);

    //trackTurretMotorCurrent
    deFaultvalue = getDefault("trackTurretMotorCurrent");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setTurretMotorCurrentRead((atoi(deFaultvaluePtr)) ? true : false);

    //trackLeftWheelEncoder
    deFaultvalue = getDefault("trackLeftWheelEncoder");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setLeftWheelEncoderRead((atoi(deFaultvaluePtr)) ? true : false);

    //trackRightWheelEncoder
    deFaultvalue = getDefault("trackRightWheelEncoder");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setRightWheelEncoderRead((atoi(deFaultvaluePtr)) ? true : false);

    //leftWheelEncoderAutoreset
    deFaultvalue = getDefault("leftWheelEncoderAutoreset");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setLeftWheelEncoderAutoreset((atoi(deFaultvaluePtr)) ? true : false);

    //rightWheelEncoderAutoreset
    deFaultvalue = getDefault("rightWheelEncoderAutoreset");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setRightWheelEncoderAutoreset((atoi(deFaultvaluePtr)) ? true : false);

    //trackTurretPosition
    deFaultvalue = getDefault("trackTurretPosition");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    setTurretPositionRead((atoi(deFaultvaluePtr)) ? true : false);

    //platformControlP1ReadingsTable
    deFaultvalue = getDefault("platformControlP1ReadingsTable");
    vector<string>deFaultvalue_str_values = Valter::split(deFaultvalue, ',');
    vector<string>::iterator iter = deFaultvalue_str_values.begin();

    int idx = 1;
    while( iter != deFaultvalue_str_values.end() )
    {
        string val = *iter++;
        deFaultvaluePtr = Valter::stringToCharPtr(val);
        switch (idx)
        {
            case 1:
                setMainAccumulatorVoltageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 2:
                setLeftAccumulatorVoltageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 3:
                setRightAccumulatorVoltageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 4:
                setMainAccumulatorAmperageTotalRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 5:
                setPlatformAmperageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 6:
                setBodyAmperageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 7:
                setLeftAccumulatorAmperageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 8:
                setRightAccumulatorAmperageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 9:
                setChargerVoltageRead((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 10:
                setMainAccumulatorVoltageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 11:
                setLeftAccumulatorVoltageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 12:
                setRightAccumulatorVoltageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 13:
                setMainAccumulatorAmperageTotalReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 14:
                setPlatformAmperageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 15:
                setBodyAmperageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 16:
                setLeftAccumulatorAmperageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 17:
                setRightAccumulatorAmperageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
            case 18:
                setChargerVoltageReadADCPreset((atoi(deFaultvaluePtr)) ? true : false);
            break;
        }

        idx++;
    }
    //additionalReadingsScanDelay
    deFaultvalue = getDefault("additionalReadingsScanDelay");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    minValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setAdditionalReadingsDelayPresetMin(minValue);
    setAdditionalReadingsDelayPresetMax(maxValue);
    setAdditionalReadingsDelayCur(curValue);
}
