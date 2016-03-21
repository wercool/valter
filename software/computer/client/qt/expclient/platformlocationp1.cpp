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
    }
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

    string deFaultvalue;
    char *deFaultvaluePtr;

    //IRSensorsReadTable
    deFaultvalue = getDefault("IRSensorsReadTable");
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
                qDebug("!!!!!!");
            break;
        }
    }
}

bool PlatformLocationP1::getRedLedState(int index)
{
    return redLedArray[index];
}

void PlatformLocationP1::setRedLedState(int index, bool value)
{
    redLedArray[index] = value;
    string ledStates = "SETLEDS#";
    for (int i = 0; i < 12; i++)
    {
        ledStates.append(getGreenLedState(i) ? "1" : "0");
        ledStates.append(getRedLedState(i) ? "1" : "0");
    }
    sendCommand(ledStates);
}

bool PlatformLocationP1::getGreenLedState(int index)
{
    return greenLedArray[index];
}

void PlatformLocationP1::setGreenLedState(int index, bool value)
{
    greenLedArray[index] = value;
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
                this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else
            {
                this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
}
