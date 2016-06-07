#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformcontrolp1.h"

#include "platformcontrolp1.utils.cpp"
#include "tcphandlers/platformcontrolp1.tcphandler.cpp"


PlatformControlP1 *PlatformControlP1::pPlatformControlP1 = NULL;
bool PlatformControlP1::instanceFlag = false;
const string PlatformControlP1::controlDeviceId = "PLATFORM-CONTROL-P1";
const string PlatformControlP1::defaultsFilePath = "settings/platform-control-p1-defaults";

PlatformControlP1::PlatformControlP1()
{
    Valter::log(PlatformControlP1::controlDeviceId + " singleton initialized");

    controlDeviceIsSet = false;

    initTcpInterface();

    resetToDefault();
    loadDefaults();

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
        setModuleInitialState();
    }
}

void PlatformControlP1::setModuleInitialState()
{
    setPlatformEmergencyStop(true);
    setTurretEmergencyStop(true);
}

void PlatformControlP1::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP1::processMessagesQueueWorker, this));
    getControlDevice()->addMsgToDataExchangeLog("PlatformControlP1 Module process messages queue worker started...");
}

void PlatformControlP1::initTcpInterface()
{
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(33333);
        setTcpInterface(tcpInterface);
        initTcpCommandAcceptorInterface();
    }
}

void PlatformControlP1::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new PlatformControlP1TCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
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

                processControlDeviceResponse(response);

                bool successfullySent = getTcpInterface()->sendCDRToCentralCommandHost(Valter::format_string("CDR~%s", response.c_str()));
                if (!successfullySent)
                {
                    stopAll();
                    getTcpInterface()->setConnected(false);
                }

                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        getControlDevice()->addMsgToDataExchangeLog("PlatformControlP1 Module process messages queue worker stopped!");
    }
}

void PlatformControlP1::processControlDeviceResponse(string response)
{
    //execute identification of responses with priority
    if (getLeftMotorStop() && getRightMotorStop() && getTurretMotorStop())
    {
        if (response.compare("DC/DC 5V ENABLED") == 0)
        {
            setPower5VOnState(true);
            return;
        }
        if (response.compare("DC/DC 5V DISABLED") == 0)
        {
            setPower5VOnState(false);
            return;
        }
        if (response.compare("LEFT ACCUMULATOR CONNECTED") == 0) //to charger
        {
            setLeftAccumulatorConnected(true);
            return;
        }
        if (response.compare("LEFT ACCUMULATOR DISCONNECTED") == 0) //from charger
        {
            setLeftAccumulatorConnected(false);
            return;
        }
        if (response.compare("RIGHT ACCUMULATOR CONNECTED") == 0) //to charger
        {
            setRightAccumulatorConnected(true);
            return;
        }
        if (response.compare("RIGHT ACCUMULATOR DISCONNECTED") == 0) //from charger
        {
            setRightAccumulatorConnected(false);
            return;
        }
        if (response.find("INPUT1 CHANNEL [8]: ") != std::string::npos) //charger voltage
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setChargerVoltageADC(value);
            setPower220VACAvailable(true);
            return;
        }
        if (response.compare("MAIN ACCUMULATOR RELAY SET ON") == 0)
        {
            setMainAccumulatorRelayOnState(true);
            return;
        }
        if (response.compare("MAIN ACCUMULATOR RELAY SET OFF") == 0)
        {
            setMainAccumulatorRelayOnState(false);
            return;
        }
        if (response.compare("LEFT ACCUMULATOR RELAY SET ON") == 0)
        {
            setLeftAccumulatorRelayOnState(true);
            return;
        }
        if (response.compare("LEFT ACCUMULATOR RELAY SET OFF") == 0)
        {
            setLeftAccumulatorRelayOnState(false);
            return;
        }
        if (response.compare("RIGHT ACCUMULATOR RELAY SET ON") == 0)
        {
            setRightAccumulatorRelayOnState(true);
            return;
        }
        if (response.compare("RIGHT ACCUMULATOR RELAY SET OFF") == 0)
        {
            setRightAccumulatorRelayOnState(false);
            return;
        }
        if (response.find("INPUT2 CHANNEL [8]: ") != std::string::npos) //charger connected
        {
            int substr_pos = response.find(":") + 2;
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
            setPower220VACAvailable(true);
            return;
        }
        if (response.find("INPUT2 CHANNEL [9]: ") != std::string::npos) //14.4V / 0.8A 1.2-35Ah
        {
            int substr_pos = response.find(":") + 2;
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
            return;
        }
        if (response.find("INPUT2 CHANNEL [10]: ") != std::string::npos) //14.4V / 3.6A 35-120Ah
        {
            int substr_pos = response.find(":") + 2;
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
            return;
        }
        if (response.find("INPUT2 CHANNEL [6]: ") != std::string::npos) //Charging in progress
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            if (value > 500)
            {
                setChargingInProgress(true);
            }
            else
            {
                setChargingInProgress(false);
            }
            return;
        }
        if (response.find("INPUT2 CHANNEL [7]: ") != std::string::npos) //Charging complete
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            if (value > 500)
            {
                setChargingComplete(true);
            }
            else
            {
                setChargingComplete(false);
            }
            return;
        }
    }
    else
    {
        //response template
        //ALLCURREADINGS:IN1 CH#,READING;IN2 CH#,READING;TURRET POSITION;LEFT MOTOR CURRENT;RIGHT MOTOR CURRENT;TURRET MOTOR CURRENT;LEFT MOTOR COUNTER;RIGHT MOTOR COUNTER
        if (response.find("ALLCURREADINGS:") != std::string::npos) //composed readings
        {
            int substr_pos = response.find(":") + 1;
            string value_str = response.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, ';');

            int turretPosition = atoi(Valter::stringToCharPtr(value_str_values[2]));

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

            if (getTurretPositionRead() && !getTurretMotorStop())
            {
                setTurretPositionADC(turretPosition);
            }
            return;
        }
    }
    //get when requested inspite motors is not stopped
    if (getLeftWheelEncoderGetOnce())
    {
        if (response.find("LEFT MOTOR COUNTER: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setLeftWheelEncoder(value);
            setLeftWheelEncoderGetOnce(false);
            return;
        }
    }
    if (getRightWheelEncoderGetOnce())
    {
        if (response.find("RIGHT MOTOR COUNTER: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setRightWheelEncoder(value);
            setRightWheelEncoderGetOnce(false);
            return;
        }
    }
    if (getTurretPositionGetOnce())
    {
        if (response.find("TURRET: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setTurretPositionADC(value);
            setTurretPositionGetOnce(false);
            return;
        }
    }
    if (getMainAccumulatorVoltageRead())
    {
        //main accumulator voltage
        if (response.find("INPUT1 CHANNEL [0]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setMainAccumulatorVoltageADC(value);
            return;
        }
    }
    if (getLeftAccumulatorVoltageRead())
    {
        //left accumulator voltage
        if (response.find("INPUT1 CHANNEL [1]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setLeftAccumulatorVoltageADC(value);
            return;
        }
    }
    if (getRightAccumulatorVoltageRead())
    {
        //right accumulator voltage
        if (response.find("INPUT1 CHANNEL [2]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setRightAccumulatorVoltageADC(value);
            return;
        }
    }
    if (getMainAccumulatorAmperageTotalRead())
    {
        //main accumulator amperage total
        if (response.find("INPUT1 CHANNEL [3]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setMainAccumulatorAmperageTotalADC(value);
            return;
        }
    }
    if (getPlatformAmperageRead())
    {
        //main accumulator amperage bottom (platform)
        if (response.find("INPUT1 CHANNEL [4]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setPlatformAmperageADC(value);
            return;
        }
    }
    if (getBodyAmperageRead())
    {
        //main accumulator amperage top (body)
        if (response.find("INPUT1 CHANNEL [5]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setBodyAmperageADC(value);
            return;
        }
    }
    if (getLeftAccumulatorAmperageRead())
    {
        //left accumulator amperage
        if (response.find("INPUT1 CHANNEL [6]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setLeftAccumulatorAmperageADC(value);
            return;
        }
    }
    if (getRightAccumulatorAmperageRead())
    {
        //right accumulator amperage
        if (response.find("INPUT1 CHANNEL [7]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setRightAccumulatorAmperageADC(value);
            return;
        }
    }
    if (getChargerVoltageRead())
    {
        //charger connected (charger voltage)
        if (response.find("INPUT1 CHANNEL [8]: ") != std::string::npos)
        {
            int substr_pos = response.find(":") + 2;
            string value_str = response.substr(substr_pos);
            int value = atoi(value_str.c_str());
            setChargerVoltageADC(value);
            setPower220VACAvailable(true);
            return;
        }
    }

    if (response.find("REMOTECD") != std::string::npos)
    {
        vector<string>value_str_values = Valter::split(response, ':');
        ControlDevice *controlDevice = new ControlDevice();
        controlDevice->setControlDeviceId(value_str_values[1]);
        controlDevice->setRemote(true);
        controlDevice->setRemoteIPAddress(value_str_values[2]);
        controlDevice->setRemotePort(atoi(((string)value_str_values[3]).c_str()));
        controlDevice->setRemoteStatus(value_str_values[4]);
        Valter::getInstance()->addControlDeviceToRemoteControlDevicesMap(controlDevice);
        return;
    }
}

unsigned int PlatformControlP1::executeTask(string taskScriptLine)
{
    return 0;
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
    qDebug("STOPPED: PlatformControlP1::scanFor220VACAvailable");
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

    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (platformControlP2->getLeftEncoderAutoreset())
    {
        platformControlP2->resetLeftEncoder();
    }
    if (platformControlP2->getRightEncoderAutoreset())
    {
        platformControlP2->resetRightEncoder();
    }

    return canMove;
}

void PlatformControlP1::toggle5VSource(bool state)
{
    if (state)
    {
        sendCommand("DCDC5VENABLEON");
    }
    else
    {
        sendCommand("DCDC5VENABLEOFF");
    }
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
    qDebug("STOPPED: PlatformControlP1::platformMovementWorker");
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
    qDebug("STOPPED: PlatformControlP1::turretRotationWorker");
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
    qDebug("STOPPED: PlatformControlP1::additionalReadingsScanWorker");
}



void PlatformControlP1::setRightAccumulatorAmperageADC(int value)
{
    rightAccumulatorAmperageADC = value;
    setRightAccumulatorAmperageAmps((float)value / (float)10);
}

void PlatformControlP1::setLeftAccumulatorAmperageADC(int value)
{
    leftAccumulatorAmperageADC = value;
    setLeftAccumulatorAmperageAmps((float)value / (float)10);
}

void PlatformControlP1::setBodyAmperageADC(int value)
{
    bodyAmperageADC = value;
    setBodyAmperageAmps((float)value / (float)10);
}

void PlatformControlP1::setPlatformAmperageADC(int value)
{
    platformAmperageADC = value;
    setPlatformAmperageAmps((float)value / (float)10);
}

void PlatformControlP1::setMainAccumulatorAmperageTotalADC(int value)
{
    mainAccumulatorAmperageTotalADC = value;
    setMainAccumulatorAmperageTotalAmps((float)value / (float)10);
}

void PlatformControlP1::setRightAccumulatorVoltageADC(int value)
{
    rightAccumulatorVoltageADC = value;
    setRightAccumulatorVoltageVolts((float)value / (float)10);
}

void PlatformControlP1::setLeftAccumulatorVoltageADC(int value)
{
    leftAccumulatorVoltageADC = value;
    setLeftAccumulatorVoltageVolts((float)value / (float)10);
}

void PlatformControlP1::setMainAccumulatorVoltageADC(int value)
{
    mainAccumulatorVoltageADC = value;
    setMainAccumulatorVoltageVolts((float)value / (float)10);
}

void PlatformControlP1::setTurretPositionGetOnce(bool value)
{
    turretPositionGetOnce = value;
    if (turretPositionGetOnce)
    {
        sendCommand("GETTURRETPOSITION");
    }
}

void PlatformControlP1::setRightWheelEncoderGetOnce(bool value)
{
    rightWheelEncoderGetOnce = value;
    if (rightWheelEncoderGetOnce)
    {
        sendCommand("GETRIGHTMOTORCOUNTER");
    }
}

void PlatformControlP1::setLeftWheelEncoderGetOnce(bool value)
{
    leftWheelEncoderGetOnce = value;
    if (leftWheelEncoderGetOnce)
    {
        sendCommand("GETLEFTMOTORCOUNTER");
    }
}

void PlatformControlP1::setTurretPositionADC(int value)
{
    turretPositionADC = value;
    setTurretPositionDeg((float)value / (float)10);
    setTurretPositionRad((float)value / (float)10);
}

void PlatformControlP1::resetLeftWheelEncoder()
{
    sendCommand("RESETLEFTMOTORCOUNTER");
    setLeftWheelEncoder(0);
}

void PlatformControlP1::resetRightWheelEncoder()
{
    sendCommand("RESETRIGHTMOTORCOUNTER");
    setRightWheelEncoder(0);
}

void PlatformControlP1::setRightMotorCurrentADC(int value)
{
    rightMotorCurrentADC = value;
    setRightMotorCurrentAmps((float)value / (float)10);
}

void PlatformControlP1::setTurretMotorCurrentADC(int value)
{
    turretMotorCurrentADC = value;
    setTurretMotorCurrentAmps((float)value / (float)10);
}

void PlatformControlP1::setLeftMotorCurrentADC(int value)
{
    leftMotorCurrentADC = value;
    setLeftMotorCurrentAmps((float)value / (float)10);
}

void PlatformControlP1::setMotorsPWMFrequncy(int value)
{
    motorsPWMFrequncy = value;
    sendCommand(Valter::format_string("SETPWMFREQUENCY#%d", motorsPWMFrequncy));
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

void PlatformControlP1::setTurretMotorActivated(bool value)
{
    turretMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setTurretMotorStop(false);
    }
}

void PlatformControlP1::setTurretMotorStop(bool value)
{
    turretMotorStop = value;
    if (turretMotorStop)
    {
        setTurretMotorCurrentADC(0);
    }
}

void PlatformControlP1::setRightMotorActivated(bool value)
{
    rightMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setRightMotorStop(false);
    }
}

void PlatformControlP1::setLeftMotorActivated(bool value)
{
    leftMotorActivated = value;
    if (value)
    {
        setLeftMotorStop(false);
    }
}

void PlatformControlP1::setRightMotorStop(bool value)
{
    rightMotorStop = value;
    if (rightMotorStop)
    {
        setRightMotorCurrentADC(0);
    }
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

void PlatformControlP1::setChargerMode(bool value)
{
    chargerMode = value;
    new std::thread(&PlatformControlP1::chargerModeSetting, this);
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
        toggleMainAccumulatorRelayState(true);
        return true;
    }
    else
    {
        return false;
    }
}

void PlatformControlP1::setChargerVoltageADC(int value)
{
    chargerVoltageADC = value;
    setChargerVoltageVolts((float)value / (float)10);
}



void PlatformControlP1::toggleRightAccumulator(bool state)
{
    if (state)
    {
        sendCommand("RIGHTACCUMULATORCONNECT");
    }
    else
    {
        sendCommand("RIGHTACCUMULATORDISCONNECT");
    }
}

void PlatformControlP1::toggleRightAccumulatorRelay(bool state)
{
    if (state)
    {
        sendCommand("RIGHTACCUMULATORRELAYON");
    }
    else
    {
        sendCommand("RIGHTACCUMULATORRELAYOFF");
    }
}

void PlatformControlP1::toggleMainAccumulatorRelayState(bool state)
{
    if (state)
    {
        sendCommand("MAINACCUMULATORRELAYON");
    }
    else
    {
        sendCommand("MAINACCUMULATORRELAYOFF");
    }
}

void PlatformControlP1::toggleLeftAccumulatorRelay(bool state)
{
    if (state)
    {
        sendCommand("LEFTACCUMULATORRELAYON");
    }
    else
    {
        sendCommand("LEFTACCUMULATORRELAYOFF");
    }
}

void PlatformControlP1::toggleLeftAccumulator(bool state)
{
    if (state)
    {
        sendCommand("LEFTACCUMULATORCONNECT");
    }
    else
    {
        sendCommand("LEFTACCUMULATORDISCONNECT");
    }
}
