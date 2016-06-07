#include "valter.h"
#include "armcontrolright.h"

#include "armcontrolright.h"
#include "tcphandlers/armcontrolright.tcphandler.cpp"

ArmControlRight *ArmControlRight::pArmControlRight = NULL;
bool ArmControlRight::instanceFlag = false;
const string ArmControlRight::controlDeviceId = "ARM-CONTROL-RIGHT";
const string ArmControlRight::defaultsFilePath = "settings/arm-control-right-defaults";

ArmControlRight::ArmControlRight()
{
    Valter::log(ArmControlRight::controlDeviceId + " singleton initialized");
    this->controlDeviceIsSet = false;

    initTcpInterface();

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
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(33338);
        setTcpInterface(tcpInterface);
        initTcpCommandAcceptorInterface();
    }
}

void ArmControlRight::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new ArmControlRightTCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
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
    }
}

void ArmControlRight::processControlDeviceResponse(string response)
{
    if (response.compare("FOREARM ROLL MOTOR ON") == 0)
    {
        setForearmRollMotorState(true);
        return;
    }
    if (response.compare("FOREARM ROLL MOTOR OFF") == 0)
    {
        setForearmRollMotorState(false);
        return;
    }

    if (getForearmRollMotorState())
    {
        if (response.compare("FA NO LIMIT") == 0)
        {
            setForearmRollCWLimit(false);
            setForearmRollCCWLimit(false);
            return;
        }
        if (response.compare("FA CW LIMIT") == 0)
        {
            setForearmRollCWLimit(true);
            return;
        }
        if (response.compare("FA CCW LIMIT") == 0)
        {
            setForearmRollCCWLimit(true);
            return;
        }
    }
    if (getForearmPositionTrack())
    {
        if (response.find("FOREARM POS:") != std::string::npos)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setForearmADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getArmPositionTrack())
    {
        if (response.find("ARM POS:") != std::string::npos)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setArmADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getLimbPositionTrack())
    {
        if (response.find("LIMB POS:") != std::string::npos)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            setLimbADCPosition(atoi(value_str.c_str()));
            return;
        }
    }
    if (getForearmMotorCurrentTrack())
    {
        if (response.find("SENSOR:5") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setForearmMotorADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getArmMotorCurrentTrack())
    {
        if (response.find("SENSOR:4") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setArmMotorADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getLimbMotorCurrentTrack())
    {
        if (response.find("SENSOR:3") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setLimbMotorADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getHandYawPositionTrack())
    {
        if (response.find("SENSOR:0") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setHandYawADCPosition(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getHandPitchPositionTrack())
    {
        if (response.find("SENSOR:1") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setHandPitchADCPosition(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getForearmYawPositionTrack())
    {
        if (response.find("SENSOR:2") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setForearmYawADCPosition(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getForearmYawMotorCurrentTrack())
    {
        if (response.find("SENSOR:6") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setForearmYawMotorADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getHandYawMotorCurrentTrack())
    {
        if (response.find("SENSOR:7") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setHandYawMotorADCCurrent(atoi(value_str_values[1].c_str()));
            return;
        }
    }
    if (getHandPitchMotorCurrentTrack())
    {
        if (response.find("SENSOR:8") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setHandPitchMotorADCCurrent(atoi(value_str_values[1].c_str()));
            return;
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
        if (response.find("HAND SENSOR:") == 0)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            vector<string>value_str_values = Valter::split(value_str, ',');
            setHandSensorsADCForce(atoi(value_str_values[0].c_str()), atoi(value_str_values[1].c_str()));
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

unsigned int ArmControlRight::executeTask(string taskScriptLine)
{
    return 0;
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
    qDebug("STOPPED: ArmControlRight::rightForearmWorker");
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
    qDebug("STOPPED: ArmControlRight::rightArmWorker");
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
    qDebug("STOPPED: ArmControlRight::rightLimbWorker");
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
    qDebug("STOPPED: ArmControlRight::rightForearmRollWorker");
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
    qDebug("STOPPED: ArmControlRight::rightArmSensorReadingsWorker");
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

void ArmControlRight::stopAllWatchers()
{
    sendCommand("STOPWATCHERS");
}

void ArmControlRight::startAllWatchers()
{
    sendCommand("STARTWATCHERS");
}

void ArmControlRight::releaseFinger(unsigned int idx)
{
    Valter::getInstance()->executeUscCmdMaestroLinux(Valter::format_string("--servo %d,0", idx));
}

void ArmControlRight::releaseAllFingers()
{
//    releaseFinger(0);
//    releaseFinger(1);
//    releaseFinger(2);
//    releaseFinger(3);
//    releaseFinger(4);
//    releaseFinger(5);
    Valter::getInstance()->executeUscCmdMaestroLinux("--sub 010"); //RIGHT_ARM_INACTIVE
}

void ArmControlRight::fingersToInitialPositions()
{
//    for (int i = 0; i < 6; i++)
//    {
//        setFingerPosition(i, getFingerInitialPosition(i));
//    }
    Valter::getInstance()->executeUscCmdMaestroLinux("--sub 011"); //RIGHT_ARM_INITIAL
}

void ArmControlRight::setFingerPosition(unsigned int idx, unsigned int position)
{
    Valter::getInstance()->executeUscCmdMaestroLinux(Valter::format_string("--servo %d,%d", idx, position * 4));
}

void ArmControlRight::fingersGrasp()
{
//    for (int i = 0; i < 6; i++)
//    {
//        setFingerPosition(i, getFingerGraspedPosition(i));
//    }
    Valter::getInstance()->executeUscCmdMaestroLinux("--sub 012"); //RIGHT_ARM_GRASPING
}

void ArmControlRight::fingersSqueeze()
{

}

unsigned int ArmControlRight::getFingerInitialPosition(unsigned int idx)
{
    return fingerInitialPositions[idx];
}

unsigned int ArmControlRight::getFingerGraspedPosition(unsigned int idx)
{
    return fingerGraspedPositions[idx];
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

void ArmControlRight::setHandSensorsADCForce(int idx, int value)
{
    handSensorsADCForce[idx] = value;
    switch (idx)
    {
        case 0: //rightPalmJamb
            setPalmJambReading(value);
        break;
        case 1: //rightFinger5Tip
            setFinger5TipReading(value);
        break;
        case 2: //rightFinger5Phalanx
            setFinger5PhalanxReading(value);
        break;
        case 3: //rightFinger4Tip
            setFinger4TipReading(value);
        break;
        case 4: //rightFinger4Phalanx
            setFinger4PhalanxReading(value);
        break;
        case 5: //rightFinger3Phalanx
            setFinger3PhalanxReading(value);
        break;
        case 6: //rightFinger2Phalanx
            setFinger2PhalanxReading(value);
        break;
        case 7: //rightFinger1Tip
            setFinger1TipReading(value);
        break;
        case 8: //rightFinger1Phalanx
            setFinger1PhalanxReading(value);
        break;
        case 9: //rightFinger0Phalanx
            setFinger0PhalanxReading(value);
        break;
        case 10: //rightFinger0Tip
            setFinger0TipReading(value);
        break;
        case 11: //rightPalmUpper
            setPalmUpperReading(value);
        break;
        case 12: //rightPalmLower
            setPalmLowerReading(value);
        break;
    }
}
