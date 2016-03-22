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
    Valter::log("LEDStatesWorker finished...");
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
