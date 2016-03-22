#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformlocationp1.h"

PlatformLocationP1 *PlatformLocationP1::pPlatformLocationP1= NULL;
bool PlatformLocationP1::instanceFlag = false;
const string PlatformLocationP1::controlDeviceId = "PLATFORM-LOCATION-P1";
const string PlatformLocationP1::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/platform-location-p1-defaults";

PlatformLocationP1::PlatformLocationP1()
{
    Valter::log(PlatformLocationP1::controlDeviceId + " singleton started");
    loadDefaults();
    controlDeviceIsSet = false;

    new std::thread(&PlatformLocationP1::readSensorsWorker, this);
}

string PlatformLocationP1::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformLocationP1 *PlatformLocationP1::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformLocationP1 = new PlatformLocationP1();
        instanceFlag = true;
        return pPlatformLocationP1;
    }
    else
    {
        return pPlatformLocationP1;
    }
}

void PlatformLocationP1::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformLocationP1::controlDeviceId.c_str()));
    }
}

void PlatformLocationP1::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    for (int i = 0; i < 12; i++)
    {
        redLedArray[i] = false;
        greenLedArray[i] = false;
        readIRSensor[i] = false;
        readIRSensorADCPreset[i] = false;
        readUSSensor[i] = false;
        readUSSensorADCPreset[i] = false;
        irSensorADC[i] = 0;
        usSensorADC[i] = 0;
    }

    ledStatesSet = false;

    irSensorMetersThreshold = 0.1;
    usSensorMetersThreshold = 0.1;

    relativeUSSensorVoltage = 0;
    usSignalDuty            = 127;
    usSignalBurst           = 250;
    usSignalDelay           = 2500;

    leftSonarActivated      = false;
    rightSonarActivated     = false;

    leftSonarAngle          = 0;
    rightSonarAngle         = 0;

    leftSonarDirection      = true;  //true - CCW, false - CW
    rightSonarDirection     = true; //true - CW, false - CCW

    leftSonarMinAngle           = -90;
    leftSonarMaxAngle           = 30;
    rightSonarMinAngle          = -90;
    rightSonarMaxAngle          = 30;
    leftSonarMinRotationDuty    = 5;
    leftSonarMaxRotationDuty    = 40;
    rightSonarMinRotationDuty   = 5;
    rightSonarMaxRotationDuty   = 40;

    leftSonarIntentionalAngleSet    = false;
    rightSonarIntentionalAngleSet   = false;
}

void PlatformLocationP1::setModuleInitialState()
{
    for (int i = 0; i < 12; i++)
    {
        redLedArray[i] = false;
        greenLedArray[i] = false;
    }
    setRedLedState(0, false);
    setGreenLedState(0, false);
}

void PlatformLocationP1::spawnProcessMessagesQueueWorkerThread()
{
    setProcessMessagesQueueWorkerThread(new std::thread(&PlatformLocationP1::processMessagesQueueWorker, this));
}

void PlatformLocationP1::loadDefaults()
{
    ifstream defaultsFile(PlatformLocationP1::defaultsFilePath);
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
    vector<string>defaultValue_str_values;
    vector<string>::iterator iter;
    int idx;

    //IRSensorsReadTable
    defaultValue = getDefault("IRSensorsReadTable");
    defaultValue_str_values = Valter::split(defaultValue, ',');
    iter = defaultValue_str_values.begin();
    idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setReadIRSensor(idx, ((atoi(defaultValuePtr)) ? true : false));
        idx++;
    }
    //IRSensorsADCReadingTable
    defaultValue = getDefault("IRSensorsADCReadingTable");
    defaultValue_str_values = Valter::split(defaultValue, ',');
    iter = defaultValue_str_values.begin();
    idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setReadIRSensorADCPreset(idx, ((atoi(defaultValuePtr)) ? true : false));
        idx++;
    }

    //USSensorsReadTable
    defaultValue = getDefault("USSensorsReadTable");
    defaultValue_str_values = Valter::split(defaultValue, ',');
    iter = defaultValue_str_values.begin();
    idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setReadUSSensor(idx, ((atoi(defaultValuePtr)) ? true : false));
        idx++;
    }
    //USSensorsTicksReadingTable
    defaultValue = getDefault("USSensorsTicksReadingTable");
    defaultValue_str_values = Valter::split(defaultValue, ',');
    iter = defaultValue_str_values.begin();
    idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setReadUSSensorTicksPreset(idx, ((atoi(defaultValuePtr)) ? true : false));
        idx++;
    }

    //irSensorMetersThreshold
    defaultValue = getDefault("irSensorMetersThreshold");
    irSensorMetersThreshold = stof(Valter::stringToCharPtr(defaultValue));

    //usSensorMetersThreshold
    defaultValue = getDefault("usSensorMetersThreshold");
    usSensorMetersThreshold = stof(Valter::stringToCharPtr(defaultValue));

    //relativeUSSensorVoltage
    defaultValue = getDefault("relativeUSSensorVoltage");
    relativeUSSensorVoltage = atoi(Valter::stringToCharPtr(defaultValue));

    //usSignalDuty
    defaultValue = getDefault("usSignalDuty");
    usSignalDuty = atoi(Valter::stringToCharPtr(defaultValue));

    //usSignalBurst
    defaultValue = getDefault("usSignalBurst");
    usSignalBurst = atoi(Valter::stringToCharPtr(defaultValue));

    //usSignalDelay
    defaultValue = getDefault("usSignalDelay");
    usSignalDelay = atoi(Valter::stringToCharPtr(defaultValue));

    //leftSonarMinAngle
    defaultValue = getDefault("leftSonarMinAngle");
    leftSonarMinAngle = atoi(Valter::stringToCharPtr(defaultValue));

    //leftSonarMaxAngle
    defaultValue = getDefault("leftSonarMaxAngle");
    leftSonarMaxAngle = atoi(Valter::stringToCharPtr(defaultValue));

    //rightSonarMinAngle
    defaultValue = getDefault("rightSonarMinAngle");
    rightSonarMinAngle = atoi(Valter::stringToCharPtr(defaultValue));

    //rightSonarMaxAngle
    defaultValue = getDefault("rightSonarMaxAngle");
    rightSonarMaxAngle = atoi(Valter::stringToCharPtr(defaultValue));

    //leftSonarMinRotationDuty
    defaultValue = getDefault("leftSonarMinRotationDuty");
    leftSonarMinRotationDuty = atoi(Valter::stringToCharPtr(defaultValue));

    //leftSonarMaxRotationDuty
    defaultValue = getDefault("leftSonarMaxRotationDuty");
    leftSonarMaxRotationDuty = atoi(Valter::stringToCharPtr(defaultValue));

    //rightSonarMinRotationDuty
    defaultValue = getDefault("rightSonarMinRotationDuty");
    rightSonarMinRotationDuty = atoi(Valter::stringToCharPtr(defaultValue));

    //rightSonarMaxRotationDuty
    defaultValue = getDefault("rightSonarMaxRotationDuty");
    rightSonarMaxRotationDuty = atoi(Valter::stringToCharPtr(defaultValue));
}

bool PlatformLocationP1::getRedLedState(int index)
{
    return redLedArray[index];
}

void PlatformLocationP1::setRedLedState(int index, bool value)
{
    redLedArray[index] = value;
    setRedLedsState();
}

bool PlatformLocationP1::getGreenLedState(int index)
{
    return greenLedArray[index];
}

void PlatformLocationP1::setGreenLedState(int index, bool value)
{
    greenLedArray[index] = value;
    setRedLedsState();
}

void PlatformLocationP1::setRedLedsState()
{
    string ledStates = "SETLEDS#";
    for (int i = 0; i < 12; i++)
    {
        ledStates.append(getGreenLedState(i) ? "1" : "0");
        ledStates.append(getRedLedState(i) ? "1" : "0");
    }
    sendCommand(ledStates);
}

void PlatformLocationP1::processMessagesQueueWorker()
{
    if (getControlDeviceIsSet())
    {
        while (getControlDevice()->getStatus() == ControlDevice::StatusActive)
        {
            if (getControlDevice()->responsesAvailable())
            {
                string response = getControlDevice()->pullResponse();

                if (response.find("IR[") !=std::string::npos) //infrared SHARP sensor readings
                {
                    int channel_str_pos1 = response.find_first_of("[") + 1;
                    int channel_str_pos2 = response.find_first_of("]");
                    string channel_str = response.substr(channel_str_pos1, channel_str_pos2 - channel_str_pos1);
                    int channel = atoi(channel_str.c_str());
                    int value_str_pos = response.find_first_of(":") + 1;
                    string value_str = response.substr(value_str_pos);
                    int  value = atoi(value_str.c_str());

                    setIRSensorADC(channel, value);

                    continue;
                }
                if (response.find("US[") !=std::string::npos) //ultrasound distance meter sensor readings
                {
                    int channel_str_pos1 = response.find_first_of("[") + 1;
                    int channel_str_pos2 = response.find_first_of("]");
                    string channel_str = response.substr(channel_str_pos1, channel_str_pos2 - channel_str_pos1);
                    int channel = atoi(channel_str.c_str());
                    int value_str_pos = response.find_first_of(":") + 1;
                    string value_str = response.substr(value_str_pos);
                    int  value = atoi(value_str.c_str());

                    setUSSensorTicks(channel, value);

                    continue;
                }
                if (response.find("LUSS") !=std::string::npos) //left ultrasound sonar distance meter sensor readings
                {
                    int value_str_pos = response.find_first_of(":") + 1;
                    string value_str = response.substr(value_str_pos);
                    int  value = atoi(value_str.c_str());
                    addLeftSonarScan(getLeftSonarAngle(), value);
                }
                if (response.find("RUSS") !=std::string::npos) //right ultrasound sonar distance meter sensor readings
                {
                    int value_str_pos = response.find_first_of(":") + 1;
                    string value_str = response.substr(value_str_pos);
                    int  value = atoi(value_str.c_str());
                    addRightSonarScan(getRightSonarAngle(), value);
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

void PlatformLocationP1::LEDStatesWorker()
{
    while (!stopAllProcesses)
    {
        if (getLedStatesSet())
        {
            for (int ch = 0; ch < 12; ch++)
            {
                if (getReadIRSensor(ch) || getReadUSSensor(ch))
                {
                    if ((getReadIRSensor(ch) && getIRSensorMeters(ch) < irSensorMetersThreshold)
                         ||
                        (getReadUSSensor(ch) && getUSSensorMeters(ch) < usSensorMetersThreshold))
                    {
                        redLedArray[ch] = true;
                        greenLedArray[ch] = false;
                    }
                    else
                    {
                        redLedArray[ch] = false;
                        greenLedArray[ch] = true;
                    }
                }
                else
                {
                    redLedArray[ch] = false;
                    greenLedArray[ch] = false;
                }
            }
            setRedLedsState();
            this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        else
        {
            break;
        }
    }
    if (getControlDeviceIsSet())
    {
        getControlDevice()->addMsgToDataExchangeLog("LEDStatesWorker finished...");
    }
}

void PlatformLocationP1::leftSonarWorker()
{
    while (!stopAllProcesses)
    {
        if (getLeftSonarActivated())
        {
            if (!getLeftSonarIntentionalAngleSet())
            {
                signed int curAngle = getLeftSonarAngle();
                if (getLeftSonarDirection())
                {
                    if (curAngle > leftSonarMinAngle)
                    {
                        curAngle -= 2;
                        setLeftSonarAngle(curAngle);
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                        sendCommand("GETLEFTSONAR");
                        this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    else
                    {
                        setLeftSonarDirection(false);
                    }
                }
                else
                {
                    if (curAngle < leftSonarMaxAngle)
                    {
                        curAngle += 2;
                        setLeftSonarAngle(curAngle);
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                        sendCommand("GETLEFTSONAR");
                        this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    else
                    {
                        setLeftSonarDirection(true);
                    }
                }
            }
        }
        else
        {
            break;
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (getControlDeviceIsSet())
    {
        getControlDevice()->addMsgToDataExchangeLog("leftSonarWorker finished...");
    }
}

bool PlatformLocationP1::getLeftSonarActivated() const
{
    return leftSonarActivated;
}

void PlatformLocationP1::setLeftSonarActivated(bool value)
{
    leftSonarActivated = value;
    if (leftSonarActivated)
    {
        this_thread::sleep_for(std::chrono::milliseconds(150));
        spawnLeftSonarWorker();
    }
}

void PlatformLocationP1::rightSonarWorker()
{
    while (!stopAllProcesses)
    {
        if (getRightSonarActivated())
        {
            if (!getRightSonarIntentionalAngleSet())
            {
                signed int curAngle = getRightSonarAngle();
                if (getRightSonarDirection())
                {
                    if (curAngle < rightSonarMaxAngle)
                    {
                        curAngle += 2;
                        setRightSonarAngle(curAngle);
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                        sendCommand("GETRIGHTSONAR");
                        this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    else
                    {
                        setRightSonarDirection(false);
                    }
                }
                else
                {
                    if (curAngle > rightSonarMinAngle)
                    {
                        curAngle -= 2;
                        setRightSonarAngle(curAngle);
                        this_thread::sleep_for(std::chrono::milliseconds(100));
                        sendCommand("GETRIGHTSONAR");
                        this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    else
                    {
                        setRightSonarDirection(true);
                    }
                }
            }
        }
        else
        {
            break;
        }
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (getControlDeviceIsSet())
    {
        getControlDevice()->addMsgToDataExchangeLog("rightSonarWorker finished...");
    }
}

bool PlatformLocationP1::getRightSonarIntentionalAngleSet() const
{
    return rightSonarIntentionalAngleSet;
}

void PlatformLocationP1::setRightSonarIntentionalAngleSet(bool value)
{
    rightSonarIntentionalAngleSet = value;
}

void PlatformLocationP1::addLeftSonarScan(signed int angle, float distance)
{
    std::lock_guard<std::mutex> guard(leftSonarScans_mutex);
    distance = distance / (200 - (rand() % 20));
    leftSonarScans[angle] = distance;
}

void PlatformLocationP1::addRightSonarScan(signed int angle, float distance)
{
    std::lock_guard<std::mutex> guard(rightSonarScans_mutex);
    distance = distance / (200 - (rand() % 20));
    rightSonarScans[angle] = distance;
}

float PlatformLocationP1::getLeftSonarScan(signed int angle)
{
    return leftSonarScans[angle];
}

float PlatformLocationP1::getRightSonarScan(signed int angle)
{
    return rightSonarScans[angle];
}

void PlatformLocationP1::clearLeftSonarScans()
{
    std::lock_guard<std::mutex> guard(leftSonarScans_mutex);
    leftSonarScans.clear();
}

void PlatformLocationP1::clearRightSonarScans()
{
    std::lock_guard<std::mutex> guard(rightSonarScans_mutex);
    rightSonarScans.clear();
}

bool PlatformLocationP1::getLeftSonarIntentionalAngleSet() const
{
    return leftSonarIntentionalAngleSet;
}

void PlatformLocationP1::setLeftSonarIntentionalAngleSet(bool value)
{
    leftSonarIntentionalAngleSet = value;
}

int PlatformLocationP1::getRightSonarMaxRotationDuty() const
{
    return rightSonarMaxRotationDuty;
}

void PlatformLocationP1::setRightSonarMaxRotationDuty(int value)
{
    rightSonarMaxRotationDuty = value;
}

int PlatformLocationP1::getRightSonarMinRotationDuty() const
{
    return rightSonarMinRotationDuty;
}

void PlatformLocationP1::setRightSonarMinRotationDuty(int value)
{
    rightSonarMinRotationDuty = value;
}

int PlatformLocationP1::getLeftSonarMaxRotationDuty() const
{
    return leftSonarMaxRotationDuty;
}

void PlatformLocationP1::setLeftSonarMaxRotationDuty(int value)
{
    leftSonarMaxRotationDuty = value;
}

int PlatformLocationP1::getLeftSonarMinRotationDuty() const
{
    return leftSonarMinRotationDuty;
}

void PlatformLocationP1::setLeftSonarMinRotationDuty(int value)
{
    leftSonarMinRotationDuty = value;
}

int PlatformLocationP1::getRightSonarMaxAngle() const
{
    return rightSonarMaxAngle;
}

void PlatformLocationP1::setRightSonarMaxAngle(int value)
{
    rightSonarMaxAngle = value;
}

int PlatformLocationP1::getRightSonarMinAngle() const
{
    return rightSonarMinAngle;
}

void PlatformLocationP1::setRightSonarMinAngle(int value)
{
    rightSonarMinAngle = value;
}

int PlatformLocationP1::getLeftSonarMaxAngle() const
{
    return leftSonarMaxAngle;
}

void PlatformLocationP1::setLeftSonarMaxAngle(int value)
{
    leftSonarMaxAngle = value;
}

int PlatformLocationP1::getLeftSonarMinAngle() const
{
    return leftSonarMinAngle;
}

void PlatformLocationP1::setLeftSonarMinAngle(int value)
{
    leftSonarMinAngle = value;
}

bool PlatformLocationP1::getRightSonarDirection() const
{
    return rightSonarDirection;
}

void PlatformLocationP1::setRightSonarDirection(bool value)
{
    rightSonarDirection = value;
}

bool PlatformLocationP1::getLeftSonarDirection() const
{
    return leftSonarDirection;
}

void PlatformLocationP1::setLeftSonarDirection(bool value)
{
    leftSonarDirection = value;
}

signed int PlatformLocationP1::getRightSonarAngle() const
{
    return rightSonarAngle;
}

void PlatformLocationP1::setRightSonarAngle(signed int value)
{
    rightSonarAngle = value;
    int dutyValue = round(27);
    sendCommand(Valter::format_string("SETRIGHTUSSONARSERVODUTY#%d", dutyValue));
}

signed int PlatformLocationP1::getLeftSonarAngle() const
{
    return leftSonarAngle;
}

void PlatformLocationP1::setLeftSonarAngle(signed int value)
{
    leftSonarAngle = value;
    int dutyValue = round(14);
    sendCommand(Valter::format_string("SETLEFTUSSONARSERVODUTY#%d", dutyValue));
}

bool PlatformLocationP1::getRightSonarActivated() const
{
    return rightSonarActivated;
}

void PlatformLocationP1::setRightSonarActivated(bool value)
{
    rightSonarActivated = value;
    if (rightSonarActivated)
    {
        this_thread::sleep_for(std::chrono::milliseconds(150));
        spawnRightSonarWorker();
    }
}
int PlatformLocationP1::getRelativeUSSensorVoltage() const
{
    return relativeUSSensorVoltage;
}

void PlatformLocationP1::setRelativeUSSensorVoltageUp()
{
    relativeUSSensorVoltage += 25;
    sendCommand("USSENSORSVOLTAGEUP#25");
}

void PlatformLocationP1::setRelativeUSSensorVoltageDown()
{
    relativeUSSensorVoltage -= 25;
    sendCommand("USSENSORSVOLTAGEDOWN#25");
}

void PlatformLocationP1::spawnLeftSonarWorker()
{
    clearLeftSonarScans();
    new std::thread(&PlatformLocationP1::leftSonarWorker, this);
    Valter::log("leftSonarWorker spawned...");
}

void PlatformLocationP1::spawnRightSonarWorker()
{
    clearRightSonarScans();
    new std::thread(&PlatformLocationP1::rightSonarWorker, this);
    Valter::log("rightSonarWorker spawned...");
}

int PlatformLocationP1::getUsSignalDelay() const
{
    return usSignalDelay;
}

void PlatformLocationP1::setUsSignalDelay(int value)
{
    usSignalDelay = value;
    sendCommand(Valter::format_string("USSENSORSDELAY#%d", usSignalDelay));
}

int PlatformLocationP1::getUsSignalBurst() const
{
    return usSignalBurst;
}

void PlatformLocationP1::setUsSignalBurst(int value)
{
    usSignalBurst = value;
    sendCommand(Valter::format_string("USSENSORSBURST#%d", usSignalBurst));
}

int PlatformLocationP1::getUsSignalDuty() const
{
    return usSignalDuty;
}

void PlatformLocationP1::setUsSignalDuty(int value)
{
    usSignalDuty = value;
    sendCommand(Valter::format_string("USSENSORSDUTY#%d", usSignalDuty));
}

void PlatformLocationP1::spawnLEDStatesWorker()
{
    new std::thread(&PlatformLocationP1::LEDStatesWorker, this);
    Valter::log("LEDStatesWorker spawned...");
}

bool PlatformLocationP1::getLedStatesSet() const
{
    return ledStatesSet;
}

void PlatformLocationP1::setLedStatesSet(bool value)
{
    ledStatesSet = value;
    if (ledStatesSet)
    {
        this_thread::sleep_for(std::chrono::milliseconds(150));
        spawnLEDStatesWorker();
    }
}

void PlatformLocationP1::setReadIRSensor(int index, bool value)
{
    readIRSensor[index] = value;
}

bool PlatformLocationP1::getReadIRSensor(int index)
{
    return readIRSensor[index];
}

void PlatformLocationP1::setReadIRSensorADCPreset(int index, bool value)
{
    readIRSensorADCPreset[index] = value;
}

bool PlatformLocationP1::getReadIRSensorADCPreset(int index)
{
    return readIRSensorADCPreset[index];
}

void PlatformLocationP1::setReadUSSensor(int index, bool value)
{
    readUSSensor[index] = value;
}

bool PlatformLocationP1::getReadUSSensor(int index)
{
    return readUSSensor[index];
}

void PlatformLocationP1::setReadUSSensorTicksPreset(int index, bool value)
{
    readUSSensorADCPreset[index] = value;
}

bool PlatformLocationP1::getReadUSSensorTicksPreset(int index)
{
    return readUSSensorADCPreset[index];
}

void PlatformLocationP1::setIRSensorADC(int channel, int value)
{
    irSensorADC[channel] = value;
}

void PlatformLocationP1::setUSSensorTicks(int channel, int value)
{
    usSensorADC[channel] = value;
}

int PlatformLocationP1::getIRSensorADC(int channel)
{
    return irSensorADC[channel];
}

int PlatformLocationP1::getUSSensorTicks(int channel)
{
    return usSensorADC[channel];
}

float PlatformLocationP1::getIRSensorMeters(int channel)
{
    return ((float)irSensorADC[channel] / (float)10);
}

float PlatformLocationP1::getUSSensorMeters(int channel)
{
    return ((float)usSensorADC[channel] / (float)10);
}

void PlatformLocationP1::readSensorsWorker()
{
    bool readSensors = false;
    while (!stopAllProcesses)
    {
        readSensors = false;
        for (int ch = 0; ch < 12; ch++)
        {
            readSensors |= getReadIRSensor(ch);
            readSensors |= getReadUSSensor(ch);
        }
        if (readSensors)
        {
            for (int ch = 0; ch < 12; ch++)
            {
                if (getReadIRSensor(ch) || getReadUSSensor(ch))
                {
                    sendCommand(Valter::format_string("SETSENSORSCHANNEL%d", ch));
                }
                this_thread::sleep_for(std::chrono::milliseconds(50));
                if (getReadIRSensor(ch))
                {
                    sendCommand("GETCURIRSENSOR");
                    this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                if (getReadUSSensor(ch))
                {
                    sendCommand("GETCURUSSENSOR");
                    this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
