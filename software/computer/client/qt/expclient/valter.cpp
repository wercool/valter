//Qt specific
#include <QtDebug>

#include <string>
#include <iostream>
#include <cstdio>
#include <sys/time.h>

#include <algorithm>

#include "valter.h"
#include "controldevice.h"

Valter* Valter::pValter = NULL;
bool Valter::instanceFlag = false;
const string Valter::filePathPrefix = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/";
const string Valter::cmdFilesPath = "commands/";
const string Valter::maestoServoControllerUscCmdPathPrefix = "/home/maska/Desktop/maestro_linux/UscCmd";

Valter::Valter()
{
    logControlDeviceMessages = true;

    controlDeviceIds = {"PLATFORM-CONTROL-P1",                  //idx 1
                        "PLATFORM-CONTROL-P2",                  //idx 2
                        "PLATFORM-LOCATION-P1",                 //idx 3
                        "PLATFORM-MANIPULATOR-AND-IR-BUMPER",   //idx 4
                        "BODY-CONTROL-P1",                      //idx 5
                        "ARM-CONTROL-RIGHT",                    //idx 6
                        "ARM-CONTROL-LEFT"};                    //idx 7

    readControlDevicesCommandsFromFiles();

    ControlDevice::listDevices();

    addUpdateValterModule(PlatformControlP1::getInstance()->getControlDeviceId(), PlatformControlP1::getInstance());
    addUpdateValterModule(PlatformControlP2::getInstance()->getControlDeviceId(), PlatformControlP2::getInstance());
    addUpdateValterModule(PlatformLocationP1::getInstance()->getControlDeviceId(), PlatformLocationP1::getInstance());
    addUpdateValterModule(PlatformManipulatorAndIRBumper::getInstance()->getControlDeviceId(), PlatformManipulatorAndIRBumper::getInstance());
    addUpdateValterModule(BodyControlP1::getInstance()->getControlDeviceId(), BodyControlP1::getInstance());
    addUpdateValterModule(ArmControlRight::getInstance()->getControlDeviceId(), ArmControlRight::getInstance());
    addUpdateValterModule(ArmControlLeft::getInstance()->getControlDeviceId(), ArmControlLeft::getInstance());

    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("PCP1", PlatformControlP1::getInstance()));
    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("PCP2", PlatformControlP2::getInstance()));
    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("PLP1", PlatformLocationP1::getInstance()));
    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("PMIB", PlatformManipulatorAndIRBumper::getInstance()));
    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("BCP1", BodyControlP1::getInstance()));
    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("ACR", ArmControlRight::getInstance()));
    valterModuleShortNameMap.insert(pair<std::string, IValterModule*>("ACL", ArmControlLeft::getInstance()));

    TaskManager::getInstance();

    remoteControlDeviceTCPInterfacesIpAddressesVector = {};
}

vector<string> Valter::getRemoteControlDeviceTCPInterfacesIpAddressesVector() const
{
    return remoteControlDeviceTCPInterfacesIpAddressesVector;
}

map<string, IValterModule *> Valter::getValterModulesMap() const
{
    return valterModulesMap;
}

void Valter::setValterModulesMap(const map<string, IValterModule *> &value)
{
    valterModulesMap = value;
}

void Valter::addUpdateValterModule(string controlDeviceId, IValterModule *valterModule)
{
    valterModulesMap[controlDeviceId] = valterModule;
}

IValterModule *Valter::getValterModule(string controlDeviceId)
{
    return valterModulesMap[controlDeviceId];
}

void Valter::executeUscCmdMaestroLinux(string cmdArgs)
{
    string cmd = maestoServoControllerUscCmdPathPrefix + " " + cmdArgs;
    qDebug("SERVO CONTROLLER ← [%s]", cmd.c_str());
    string result = Valter::exec_shell(cmd);
    qDebug("SERVO CONTROLLER → %s", result.c_str());
}

void Valter::stopAllModules()
{
    typedef map<string, IValterModule*>::iterator it_type;
    for(it_type iterator = valterModulesMap.begin(); iterator != valterModulesMap.end(); iterator++)
    {
        IValterModule *valterModule = valterModulesMap[iterator->first];
        valterModule->stopAll();
    }
}


void Valter::setAllModulesInitialState()
{
    typedef map<string, IValterModule*>::iterator it_type;
    for(it_type iterator = valterModulesMap.begin(); iterator != valterModulesMap.end(); iterator++)
    {
        IValterModule *valterModule = valterModulesMap[iterator->first];
        valterModule->setModuleInitialState();
    }
}

IValterModule *Valter::getValterModulePtrByShortName(string shortValterModuleName)
{
    return valterModuleShortNameMap[shortValterModuleName];
}

void Valter::addIpAddressToRemoteControlDeviceTCPInterfacesIpAddressesVector(string ipAddress)
{
    if (ipAddress.compare("127.0.0.1") == 0)
    {
        return;
    }
    bool addIpAddress = true;
    for(std::vector<string>::size_type i = 0; i != remoteControlDeviceTCPInterfacesIpAddressesVector.size(); i++)
    {
        if (((string)remoteControlDeviceTCPInterfacesIpAddressesVector[i]).compare(ipAddress) == 0)
        {
            addIpAddress = false;
        }
    }
    if (addIpAddress)
    {
        remoteControlDeviceTCPInterfacesIpAddressesVector.push_back(ipAddress);
    }
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
    return "0.1.2";
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
    if (msg.length() > 0)
    {
        if (logToGUI)
        {
            MainWindow::getInstance()->addMsgToLog(msg);
        }
        if (logToConsole)
        {
            timeval curTime;
            gettimeofday(&curTime, NULL);
            int milli = curTime.tv_usec / 1000;

            char buffer [80];
            strftime(buffer, 80, "%H:%M:%S", localtime(&curTime.tv_sec));

            char currentTime[84] = "";
            sprintf(currentTime, "%s:%03d", buffer, milli);

            qDebug("%s: %s", currentTime, msg.c_str());
        }
    }
}

void Valter::delayGUIAction(string msg)
{
    MainWindow::getInstance()->addMsgToLog(msg);
}

void Valter::readControlDevicesCommandsFromFiles(bool printCommands)
{
    vector<string> commands;
    for(vector<string>::size_type i = 0; i != controlDeviceIds.size(); i++)
    {
        commands.clear();
        ifstream cmdfile(filePathPrefix + cmdFilesPath + (string)controlDeviceIds[i]);
        string line;
        while (getline(cmdfile, line, '\n'))
        {
            commands.push_back(line);
        }
        controlDevicesCommands[(string)controlDeviceIds[i]] = commands;
        cmdfile.close();
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

void Valter::clearControlDevicesMap()
{
    controlDevicesMap.clear();
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

    string controlDeviceSysPathCmdWithDev = Valter::format_string(ControlDevice::controlDeviceSysPathCmd, port.c_str());
    string sys_usb_bus_device = Valter::exec_shell(controlDeviceSysPathCmdWithDev);
    sys_usb_bus_device.erase(sys_usb_bus_device.length()-1);

    controlDevice->setSysDevicePath(sys_usb_bus_device);

    addControlDeviceToControlDevicesMap(controlDevice);
}

void Valter::addControlDeviceToControlDevicesMap(ControlDevice *controlDevice)
{
    controlDevicesMap[controlDevice->getControlDeviceId()] = controlDevice;
    Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->setControlDevice(controlDevice);
    Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->resetToDefault();
    Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->setReloadDefaults(true);
}

void Valter::updateControlDevice(string controlDeviceId, string port)
{
    serial::Serial *controlDevicePort = new serial::Serial(port, ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));
    ControlDevice *controlDevice = getControlDeviceById(controlDeviceId);
    controlDevice->setControlDeviceId(controlDeviceId);
    controlDevice->setControlDevicePort(controlDevicePort);
    controlDevice->getControlDevicePort()->close();
    controlDevice->setStatus(ControlDevice::StatusReady);
    controlDevice->setResetWDTimer(true);

    string controlDeviceSysPathCmdWithDev = Valter::format_string(ControlDevice::controlDeviceSysPathCmd, port.c_str());
    string sys_usb_bus_device = Valter::exec_shell(controlDeviceSysPathCmdWithDev);
    sys_usb_bus_device.erase(sys_usb_bus_device.length()-1);

    controlDevice->setSysDevicePath(sys_usb_bus_device);
    Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->resetToDefault();
    Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->setReloadDefaults(true);
}


ControlDevice *Valter::getControlDeviceById(string controlDeviceId)
{
    if (controlDevicesMap.find(controlDeviceId) != controlDevicesMap.end())
    {
        return controlDevicesMap[controlDeviceId];
    }
    else
    {
        return NULL;
    }
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
