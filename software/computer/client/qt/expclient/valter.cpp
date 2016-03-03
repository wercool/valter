//Qt specific
#include <QtDebug>

#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "controldevice.h"

Valter* Valter::pValter = NULL;
bool Valter::instanceFlag = false;
const string Valter::cmdFilesPath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/commands/";

Valter::Valter()
{
    logControlDeviceMessages = true;

    controlDeviceIds = {"PLATFORM-CONTROL-P1",
                        "PLATFORM-CONTROL-P2",
                        "PLATFORM-LOCATION-P1",
                        "PLATFORM-MANIPULATOR-AND-IR-BUMPER",
                        "BODY-CONTROL-P1",
                        "ARM-CONTROL-RIGHT",
                        "ARM-CONTROL-LEFT"};

    readControlDevicesCommandsFromFiles();

    ControlDevice::listDevices();
}


Valter* Valter::getInstance()
{
    if(!instanceFlag)
    {
        pValter = new Valter();
        instanceFlag = true;
        return pValter;
    }
    else
    {
        return pValter;
    }
}

string Valter::getVersion()
{
    return "0.0.1";
}

bool Valter::getLogControlDeviceMessages() const
{
    return logControlDeviceMessages;
}

void Valter::setLogControlDeviceMessages(bool value)
{
    logControlDeviceMessages = value;
}

void Valter::log(string msg)
{
    if (logToGUI)
    {
        MainWindow::getInstance()->addMsgToLog(msg);
    }
}

void Valter::readControlDevicesCommandsFromFiles(bool printCommands)
{
    vector<string> commands;
    for(vector<string>::size_type i = 0; i != controlDeviceIds.size(); i++)
    {
        commands.clear();
        ifstream cmdfile(cmdFilesPath + (string)controlDeviceIds[i]);
        string line;
        while (getline(cmdfile, line, '\n'))
        {
            commands.push_back(line);
        }
        controlDevicesCommands[(string)controlDeviceIds[i]] = commands;
    }
    typedef map<string, vector<string>>::iterator it_type;

    if (printCommands)
    {
        for(it_type iterator = controlDevicesCommands.begin(); iterator != controlDevicesCommands.end(); iterator++)
        {
            vector<string> commands = controlDevicesCommands[iterator->first];
            Valter::log(format_string("%s commands: ", ((string)iterator->first).c_str()));
            for(vector<string>::size_type i = 0; i != commands.size(); i++)
            {
                Valter::log(format_string("%s", ((string)commands[i]).c_str()));
            }
            Valter::log("\n");
        }
    }
}

map<string, ControlDevice *> Valter::getControlDevicesMap() const
{
    return controlDevicesMap;
}

void Valter::setControlDevicesMap(const map<string, ControlDevice *> &value)
{
    controlDevicesMap = value;
}

void Valter::scanControlDevices()
{
    ControlDevice::scanControlDevices();
}

map<string, vector<string> > Valter::getControlDevicesCommands()
{
    return controlDevicesCommands;
}

void Valter::addControlDevice(string controlDeviceId, string port)
{
    serial::Serial *controlDevicePort = new serial::Serial(port, ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));
    ControlDevice *controlDevice = new ControlDevice();
    controlDevice->setControlDeviceId(controlDeviceId);
    controlDevice->setControlDevicePort(controlDevicePort);
    controlDevice->getControlDevicePort()->close();
    controlDevice->setStatus(ControlDevice::StatusReady);
    addControlDeviceToControlDevicesMap(controlDevice);
}

void Valter::addControlDeviceToControlDevicesMap(ControlDevice *controlDevice)
{
    controlDevicesMap[controlDevice->getControlDeviceId()] = controlDevice;
}

ControlDevice *Valter::getControlDeviceById(string controlDeviceId)
{
    return controlDevicesMap[controlDeviceId];
}

void Valter::closeAllControlDevicePorts()
{

    map<string, ControlDevice*> controlDevicesMap = getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        if (controlDevice->getControlDevicePort()->isOpen())
        {
            controlDevice->setStatus(ControlDevice::StatusReady);
            this_thread::sleep_for(std::chrono::milliseconds(10));
            controlDevice->getControlDevicePort()->close();
        }
    }
}
