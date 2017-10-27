#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformcontrolp2.h"
#include "tcphandlers/platformcontrolp2.tcphandler.cpp"

PlatformControlP2 *PlatformControlP2::pPlatformControlP2 = NULL;
bool PlatformControlP2::instanceFlag = false;
const string PlatformControlP2::controlDeviceId = "PLATFORM-CONTROL-P2";
const string PlatformControlP2::defaultsFilePath = "settings/platform-control-p2-defaults";

PlatformControlP2::PlatformControlP2()
{
    Valter::log(PlatformControlP2::controlDeviceId + " singleton initialized");
    this->controlDeviceIsSet = false;

    initTcpInterface();

    resetToDefault();
    loadDefaults();
}

void PlatformControlP2::processMessagesQueueWorker()
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


void PlatformControlP2::processControlDeviceResponse(string response)
{
    if (leftEncoderGetOnce)
    {
        if (response.find("LWEN:") !=std::string::npos)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            int  value = atoi(value_str.c_str());
            setLeftEncoder(value);
            return;
        }
    }
    if (rightEncoderGetOnce)
    {
        if (response.find("RWEN:") !=std::string::npos)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            int  value = atoi(value_str.c_str());
            setRightEncoder(value);
            return;
        }
    }

    if (getEncodersWorkerActivated())
    {
        if (getReadLeftEncoder())
        {
            if (response.find("LWEN:") !=std::string::npos)
            {
                int value_str_pos = response.find_first_of(":") + 1;
                string value_str = response.substr(value_str_pos);
                int  value = atoi(value_str.c_str());
                setLeftEncoder(value);
                return;
            }
        }
        if (getReadRightEncoder())
        {
            if (response.find("RWEN:") !=std::string::npos)
            {
                int value_str_pos = response.find_first_of(":") + 1;
                string value_str = response.substr(value_str_pos);
                int  value = atoi(value_str.c_str());
                setRightEncoder(value);
                return;
            }
        }
    }

    if (getIrScanningWorkerActivated())
    {
        if (response.find("IRSCAN:") !=std::string::npos)
        {
            int value_str_pos = response.find_first_of(":") + 1;
            string value_str = response.substr(value_str_pos);
            int  value = atoi(value_str.c_str());
            if (getIntetntionalIRScannerReadingReuqest() == 1)
            {
                setIrScannerReadingADC(value);
                setIntetntionalIRScannerReadingReuqest(2);
            }
            else
            {
                addIRScannerScan(getIRScannerAngle(), value);
            }
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

unsigned int PlatformControlP2::executeTask(string taskScriptLine)
{
    return 0;
}

bool PlatformControlP2::getChargerMotorRotateWorkerActivated() const
{
    return chargerMotorRotateWorkerActivated;
}

void PlatformControlP2::setChargerMotorRotateWorkerActivated(bool value)
{
    if (value)
    {
        spawnChargerMotorRotateWorker();
    }
    else
    {
        stopChargerMotor();
    }
    chargerMotorRotateWorkerActivated = value;
}

void PlatformControlP2::stopChargerMotor()
{
    setChargerMotorDuty(1);
    sendCommand("CHARGERDRIVESTOP");
}

void PlatformControlP2::alarmBeep()
{
    sendCommand(Valter::format_string("ALARMBEEP#%d", getBeepDuration()));
}

void PlatformControlP2::ALARMOnOff(bool state)
{
    ALARM = state;

    if (ALARM)
    {
        sendCommand("ALARMON");
    }
    else
    {
        sendCommand("ALARMOFF");
    }
}

void PlatformControlP2::disableIRScannerServo()
{
    sendCommand("DISABLESONARSERVO");
}

void PlatformControlP2::resetIRScannerServo()
{
    sendCommand("SETSONARSERVODUTY#18");
    disableIRScannerServo();
}

void PlatformControlP2::requestIrScannerReadingADC()
{
    intetntionalIRScannerReadingReuqest = 1;
    sendCommand("GETIRSCAN");
}

bool PlatformControlP2::getChargerMotorDirection() const
{
    return chargerMotorDirection;
}

void PlatformControlP2::setChargerMotorDirection(bool value)
{
    chargerMotorDirection = value;
    sendCommand(Valter::format_string("CHARGERDRIVEDIRECTION#%d", (chargerMotorDirection ? 1 : 2)));
}

void PlatformControlP2::spawnChargerMotorRotateWorker()
{
    if (!getChargerMotorRotateWorkerActivated())
    {
        new std::thread(&PlatformControlP2::chargerMotorRotateWorker, this);
        Valter::log("chargerMotorRotateWorker spawned...");
    }
    else
    {
        Valter::log("chargerMotorRotateWorker ALREADY activated!");
    }
}


int PlatformControlP2::getChargerMotorDuty() const
{
    return chargerMotorDuty;
}

void PlatformControlP2::setChargerMotorDuty(int value)
{
    chargerMotorDuty = value;
    sendCommand(Valter::format_string("SETCHARGERDRIVEDUTY#%d", getChargerMotorDuty()));
}

void PlatformControlP2::encodersWorker()
{
    while (!stopAllProcesses)
    {
        if (getEncodersWorkerActivated())
        {
            if (getReadLeftEncoder())
            {
                sendCommand("LWENGET");
                this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            if (getReadRightEncoder())
            {
                sendCommand("RWENGET");
            }
            this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        else
        {
            break;
        }
    }
    qDebug("STOPPED: PlatformControlP2::encodersWorker");
}

void PlatformControlP2::irScanningWorker()
{
    while (!stopAllProcesses)
    {
        if (getIrScanningWorkerActivated())
        {
            if (!getIRScannerIntentionalAngleSet())
            {
                signed int curAngle = getIRScannerAngle();
                if (getIRScannerDirection())
                {
                    if (curAngle > iRScannerMinAngle)
                    {
                        curAngle -= 4;
                        setIRScannerAngle(curAngle);
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                        sendCommand("GETIRSCAN");
                    }
                    else
                    {
                        setIRScannerDirection(false);
                    }
                }
                else
                {
                    if (curAngle < iRScannerMaxAngle)
                    {
                        curAngle += 4;
                        setIRScannerAngle(curAngle);
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                        sendCommand("GETIRSCAN");
                    }
                    else
                    {
                        setIRScannerDirection(true);
                    }
                }
            }
        }
        else
        {
            break;
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    qDebug("STOPPED: PlatformControlP2::irScanningWorker");
}

void PlatformControlP2::chargerMotorRotateWorker()
{
    this_thread::sleep_for(std::chrono::milliseconds(5));
    while (getChargerMotorDuty() < getChargerMotorDutyPresetCur())
    {
        if (stopAllProcesses || !getChargerMotorRotateWorkerActivated())
        {
            stopChargerMotor();
            break;
        }
        if (getChargerMotorDuty() + 5 > 100)
        {
            setChargerMotorDuty(100);
        }
        else
        {
            setChargerMotorDuty(getChargerMotorDuty() + 5);
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    //setChargerMotorRotateWorkerActivated(false);
    if (getControlDeviceIsSet())
    {
        getControlDevice()->addMsgToDataExchangeLog("chargerMotorRotateWorker finished...");
    }
}

int PlatformControlP2::getIntetntionalIRScannerReadingReuqest() const
{
    return intetntionalIRScannerReadingReuqest;
}

void PlatformControlP2::setIntetntionalIRScannerReadingReuqest(int value)
{
    intetntionalIRScannerReadingReuqest = value;
}

int PlatformControlP2::getIrScannerReadingADC() const
{
    return irScannerReadingADC;
}

void PlatformControlP2::setIrScannerReadingADC(int value)
{
    irScannerReadingADC = value;
}
bool PlatformControlP2::getIRScannerIntentionalAngleSet() const
{
    return iRScannerIntentionalAngleSet;
}

void PlatformControlP2::setIRScannerIntentionalAngleSet(bool value)
{
    iRScannerIntentionalAngleSet = value;
}

void PlatformControlP2::pushChargerMotor()
{
    sendCommand(Valter::format_string("CHARGERDRIVEPUSH#%d#%d#%d", (getChargerMotorDirection() ? 1 : 2), getChargerMotorDutyPresetCur(), getChargerMotorPushDuration()));
}

int PlatformControlP2::getIRScannerMaxAngle() const
{
    return iRScannerMaxAngle;
}

void PlatformControlP2::setIRScannerMaxAngle(int value)
{
    iRScannerMaxAngle = value;
}

int PlatformControlP2::getIRScannerMinAngle() const
{
    return iRScannerMinAngle;
}

void PlatformControlP2::setIRScannerMinAngle(int value)
{
    iRScannerMinAngle = value;
}

bool PlatformControlP2::getIRScannerDirection() const
{
    return iRScannerDirection;
}

void PlatformControlP2::setIRScannerDirection(bool value)
{
    iRScannerDirection = value;
}


bool PlatformControlP2::getBottomRearLeds() const
{
    return bottomRearLeds;
}

void PlatformControlP2::setBottomRearLeds(bool value)
{
    bottomRearLeds = value;
    if (bottomRearLeds)
    {
        sendCommand("PLATFORMREARLEDSON");
    }
    else
    {
        sendCommand("PLATFORMREARLEDSOFF");
    }
}

void PlatformControlP2::resetLeftEncoder()
{
    sendCommand("LWENRESET");
}

void PlatformControlP2::resetRightEncoder()
{
    sendCommand("RWENRESET");
}

void PlatformControlP2::getLeftEncoderOnce(bool value)
{
    if (value)
    {
        sendCommand("LWENGET");
    }
    leftEncoderGetOnce = value;
}

void PlatformControlP2::getRightEncoderOnce(bool value)
{
    if (value)
    {
        sendCommand("RWENGET");
    }
    rightEncoderGetOnce = value;
}

void PlatformControlP2::addIRScannerScan(signed int angle, float distance)
{
    std::lock_guard<std::mutex> guard(IRScannerScans_mutex);
    distance = distance;
    IRScannerScans[angle] = distance;
}

float PlatformControlP2::getIRScannerScan(signed int angle)
{
    return IRScannerScans[angle];
}

void PlatformControlP2::clearIRScannerScans()
{
    std::lock_guard<std::mutex> guard(IRScannerScans_mutex);
    IRScannerScans.clear();
}

signed int PlatformControlP2::getIRScannerAngle() const
{
    return iRScannerAngle;
}

void PlatformControlP2::setIRScannerAngle(signed int value)
{
    iRScannerAngle = value;
    int dutyValue = 18;
    if (iRScannerAngle != 0)
    {
        dutyValue += double (value) * 0.125;
    }
    sendCommand(Valter::format_string("SETSONARSERVODUTY#%d", dutyValue));
}

bool PlatformControlP2::getBottomFrontLeds() const
{
    return bottomFrontLeds;
}

void PlatformControlP2::setBottomFrontLeds(bool value)
{
    bottomFrontLeds = value;
    if (bottomFrontLeds)
    {
        sendCommand("PLATFORMFRONTLEDSON");
    }
    else
    {
        sendCommand("PLATFORMFRONTLEDSOFF");
    }
}

bool PlatformControlP2::getChargerLeds() const
{
    return chargerLeds;
}

void PlatformControlP2::setChargerLeds(bool value)
{
    chargerLeds = value;
    if (chargerLeds)
    {
        sendCommand("CHARGERLEDSON");
    }
    else
    {
        sendCommand("CHARGERLEDSOFF");
    }
}

int PlatformControlP2::getBeepDurationPresetMax() const
{
    return beepDurationPresetMax;
}

void PlatformControlP2::setBeepDurationPresetMax(int value)
{
    beepDurationPresetMax = value;
}

int PlatformControlP2::getBeepDurationPresetMin() const
{
    return beepDurationPresetMin;
}

void PlatformControlP2::setBeepDurationPresetMin(int value)
{
    beepDurationPresetMin = value;
}

int PlatformControlP2::getBeepDuration() const
{
    return beepDuration;
}

void PlatformControlP2::setBeepDuration(int value)
{
    beepDuration = value;
}

bool PlatformControlP2::getIrScanningWorkerActivated() const
{
    return irScanningWorkerActivated;
}

void PlatformControlP2::setIrScanningWorkerActivated(bool value)
{
    irScanningWorkerActivated = value;
    if (irScanningWorkerActivated)
    {
        spawnIRScanningWorker();
    }
    else
    {
        disableIRScannerServo();
    }
}

void PlatformControlP2::spawnEncodersWorker()
{
    new std::thread(&PlatformControlP2::encodersWorker, this);
    Valter::log("encodersWorker spawned...");
}

void PlatformControlP2::spawnIRScanningWorker()
{
    clearIRScannerScans();
    new std::thread(&PlatformControlP2::irScanningWorker, this);
    Valter::log("irScanningWorker spawned...");
}

bool PlatformControlP2::getEncodersWorkerActivated() const
{
    return encodersWorkerActivated;
}

void PlatformControlP2::setEncodersWorkerActivated(bool value)
{
    bool curValue = encodersWorkerActivated;
    if (value)
    {
        encodersWorkerActivated = value;
        if (!curValue)
        {
            spawnEncodersWorker();
        }
    }
    else
    {
        if (!getReadLeftEncoder() && !getReadRightEncoder())
        {
            encodersWorkerActivated = value;
        }
    }
}

int PlatformControlP2::getChargerMotorPushDuration() const
{
    return chargerMotorPushDuration;
}

void PlatformControlP2::setChargerMotorPushDuration(int value)
{
    chargerMotorPushDuration = value;
}

int PlatformControlP2::getChargerMotorPushDurationPresetMax() const
{
    return chargerMotorPushDurationPresetMax;
}

void PlatformControlP2::setChargerMotorPushDurationPresetMax(int value)
{
    chargerMotorPushDurationPresetMax = value;
}

int PlatformControlP2::getChargerMotorPushDurationPresetCur() const
{
    return chargerMotorPushDuration;
}

void PlatformControlP2::setChargerMotorPushDurationPresetCur(int value)
{
    chargerMotorPushDuration = value;
}

int PlatformControlP2::getChargerMotorPushDurationPresetMin() const
{
    return chargerMotorPushDurationPresetMin;
}

void PlatformControlP2::setChargerMotorPushDurationPresetMin(int value)
{
    chargerMotorPushDurationPresetMin = value;
}

int PlatformControlP2::getChargerMotorDutyPresetCur() const
{
    return chargerMotorDutyPresetCur;
}

void PlatformControlP2::setChargerMotorDutyPresetCur(int value)
{
    chargerMotorDutyPresetCur = value;
}

int PlatformControlP2::getChargerMotorDutyPresetMax() const
{
    return chargerMotorDutyPresetMax;
}

void PlatformControlP2::setChargerMotorDutyPresetMax(int value)
{
    chargerMotorDutyPresetMax = value;
}

int PlatformControlP2::getChargerMotorDutyPresetMin() const
{
    return chargerMotorDutyPresetMin;
}

void PlatformControlP2::setChargerMotorDutyPresetMin(int value)
{
    chargerMotorDutyPresetMin = value;
}

bool PlatformControlP2::getRightEncoderAutoreset() const
{
    return rightEncoderAutoreset;
}

void PlatformControlP2::setRightEncoderAutoreset(bool value)
{
    rightEncoderAutoreset = value;
}

bool PlatformControlP2::getLeftEncoderAutoreset() const
{
    return leftEncoderAutoreset;
}

void PlatformControlP2::setLeftEncoderAutoreset(bool value)
{
    leftEncoderAutoreset = value;
}

int PlatformControlP2::getRightEncoder() const
{
    return rightEncoder;
}

void PlatformControlP2::setRightEncoder(int value)
{
    rightEncoder = value;
}

int PlatformControlP2::getLeftEncoder() const
{
    return leftEncoder;
}

void PlatformControlP2::setLeftEncoder(int value)
{
    leftEncoder = value;
}

bool PlatformControlP2::getReadRightEncoder() const
{
    return readRightEncoder;
}

void PlatformControlP2::setReadRightEncoder(bool value)
{
    readRightEncoder = value;
    if (readRightEncoder)
    {
        sendCommand("RWENSTART");
    }
    else
    {
        sendCommand("RWENSTOP");
    }
}

bool PlatformControlP2::getReadLeftEncoder() const
{
    return readLeftEncoder;
}

void PlatformControlP2::setReadLeftEncoder(bool value)
{
    readLeftEncoder = value;
    if (readLeftEncoder)
    {
        sendCommand("LWENSTART");
    }
    else
    {
        sendCommand("LWENSTOP");
    }
}

string PlatformControlP2::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformControlP2 *PlatformControlP2::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformControlP2 = new PlatformControlP2();
        instanceFlag = true;
        return pPlatformControlP2;
    }
    else
    {
        return pPlatformControlP2;
    }
}

void PlatformControlP2::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformControlP2::controlDeviceId.c_str()));
    }
}

void PlatformControlP2::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    readLeftEncoder         = false;
    readRightEncoder        = false;
    leftEncoder             = 0;
    rightEncoder            = 0;
    leftEncoderAutoreset    = false;
    rightEncoderAutoreset   = false;

    chargerMotorDuty            = 1;
    chargerMotorDutyPresetMin   = 1;
    chargerMotorDutyPresetMax   = 100;
    chargerMotorDutyPresetCur   = 50;

    chargerMotorPushDuration            = 250;
    chargerMotorPushDurationPresetMin   = 50;
    chargerMotorPushDurationPresetMax   = 500;

    encodersWorkerActivated     = false;
    irScanningWorkerActivated   = false;


    beepDuration                = 5;
    beepDurationPresetMin       = 1;
    beepDurationPresetMax       = 200;

    chargerLeds         = false;
    bottomFrontLeds     = false;
    bottomRearLeds      = false;

    leftEncoderGetOnce  = false;
    rightEncoderGetOnce = false;

    iRScannerAngle  = 0;

    iRScannerDirection = true;

    iRScannerMinAngle   = -90;
    iRScannerMaxAngle   = 30;

    iRScannerIntentionalAngleSet = false;

    chargerMotorDirection = false; //ccw

    chargerMotorRotateWorkerActivated = false;

    ALARM = false;

    irScannerReadingADC = 0;
    intetntionalIRScannerReadingReuqest = 1;
}

void PlatformControlP2::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP2::processMessagesQueueWorker, this));
}

void PlatformControlP2::initTcpInterface()
{
    if (!getTcpInterface())
    {
        TCPInterface *tcpInterface = new TCPInterface(33334);
        setTcpInterface(tcpInterface);
        initTcpCommandAcceptorInterface();
    }
}

void PlatformControlP2::initTcpCommandAcceptorInterface()
{
    getTcpInterface()->setConnectionHandler((Thread*)new PlatformControlP2TCPConnectionHandler(getTcpInterface()->queue));
    getTcpInterface()->startListening();
}

void PlatformControlP2::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + PlatformControlP2::defaultsFilePath);
    string line;
    while (getline(defaultsFile, line, '\n'))
    {
        if (line.substr(0, 2).compare("//") != 0 && line.length() > 0)
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

    //trackLeftWheelEncoder
    defaultValue = getDefault("trackLeftWheelEncoder");
    setReadLeftEncoder((defaultValue.compare("1") == 0) ? true : false);

    //trackRightWheelEncoder
    defaultValue = getDefault("trackRightWheelEncoder");
    setReadRightEncoder((defaultValue.compare("1") == 0) ? true : false);

    //autoresetLeftWheelEncoder
    defaultValue = getDefault("autoresetLeftWheelEncoder");
    setLeftEncoderAutoreset((defaultValue.compare("1") == 0) ? true : false);

    //autoresetRightWheelEncoder
    defaultValue = getDefault("autoresetRightWheelEncoder");
    setRightEncoderAutoreset((defaultValue.compare("1") == 0) ? true : false);

    //chargerMotorDuty
    defaultValue = getDefault("chargerMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setChargerMotorDutyPresetMin(minValue);
    setChargerMotorDutyPresetMax(maxValue);
    setChargerMotorDutyPresetCur(curValue);

    //chargerMotorPushDuration
    defaultValue = getDefault("chargerMotorPushDuration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setChargerMotorPushDurationPresetMin(minValue);
    setChargerMotorPushDurationPresetMax(maxValue);
    setChargerMotorPushDuration(curValue);

    //irScanning
    defaultValue = getDefault("irScanning");
    setIrScanningWorkerActivated((defaultValue.compare("1") == 0) ? true : false);

    //beepDuration
    defaultValue = getDefault("beepDuration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setBeepDurationPresetMin(minValue);
    setBeepDurationPresetMax(maxValue);
    setBeepDuration(curValue);

    //chargerLeds
    defaultValue = getDefault("chargerLeds");
    setChargerLeds((defaultValue.compare("1") == 0) ? true : false);

    //bottomFrontLeds
    defaultValue = getDefault("bottomFrontLeds");
    setBottomFrontLeds((defaultValue.compare("1") == 0) ? true : false);

    //bottomRearLeds
    defaultValue = getDefault("bottomRearLeds");
    setBottomRearLeds((defaultValue.compare("1") == 0) ? true : false);

    //iRScannerMinAngle
    defaultValue = getDefault("iRScannerMinAngle");
    iRScannerMinAngle = atoi(Valter::stringToCharPtr(defaultValue));

    //iRScannerMaxAngle
    defaultValue = getDefault("iRScannerMaxAngle");
    iRScannerMaxAngle = atoi(Valter::stringToCharPtr(defaultValue));
}

void PlatformControlP2::setModuleInitialState()
{
    qDebug("PlatformControlP2::setModuleInitialState");
    for (char i = 0; i < 100; i++)
    {
        setIRScannerAngle(0);
    }
    resetIRScannerServo();
}
