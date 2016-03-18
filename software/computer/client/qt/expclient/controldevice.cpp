//Qt specific
#include <QtDebug>

#include "controldevice.h"
#include "valter.h"

const uint32_t ControlDevice::DefaultBaudRate = 115200;

const string ControlDevice::StatusActive        = "active";
const string ControlDevice::StatusReady         = "ready";

const string ControlDevice::controlDeviceSysPathCmd  = "echo /dev/bus/usb/`udevadm info --name=%s --attribute-walk | sed -n 's/\\s*ATTRS{\\(\\(devnum\\)\\|\\(busnum\\)\\)}==\\\"\\([^\\\"]\\+\\)\\\"/\\4/p' | head -n 2 | awk '{$1 = sprintf(\"%%03d\", $1); print}'` | tr \" \" \"/\"";

ControlDevice::ControlDevice()
{
    resetWDTimer = true;
    intentionalWDTimerResetOnAT91SAM7s = false;
    rescanningAfterPossibleReset = false;
    failedAfterRescanning = false;
    autoReActivation = false;
    wdTimerNotResetCnt = 0;
    rescanNum = 0;
}

void ControlDevice::listDevices()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    unsigned char ttyACMPortsNum = 0;

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        if (device.port.find("ttyACM") != string::npos)
        {
            string controlDeviceSysPathCmdWithDev = Valter::format_string(controlDeviceSysPathCmd, device.port.c_str());
            string sys_usb_bus_device = Valter::exec_shell(controlDeviceSysPathCmdWithDev);
            sys_usb_bus_device.erase(sys_usb_bus_device.length()-1);
            Valter::log(Valter::format_string("(%s, %s, %s, %s)", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str(), sys_usb_bus_device.c_str()));
            ttyACMPortsNum++;
        }
    }
    Valter::log(Valter::format_string("%u ttyACM* ports found", ttyACMPortsNum));
}

void ControlDevice::scanControlDevices()
{
    unsigned char ttyACMDevicesNum = 0;

    bool needsReScan = false;

    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo serialPortDeviceInfo = *iter++;
        if (serialPortDeviceInfo.port.find("ttyACM") != string::npos)
        {
            ttyACMDevicesNum++;
            Valter::log(Valter::format_string("Trying to connect %s port", serialPortDeviceInfo.port.c_str()));
            try
            {
                serial::Serial potentialControlDeviceSerialPort(serialPortDeviceInfo.port.c_str(), ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));

                if(potentialControlDeviceSerialPort.isOpen())
                {
                    potentialControlDeviceSerialPort.flush();

                    string result;
                    unsigned int scanStep = 0;
                    bool isValterControlDevicePort = false;
                    while (scanStep < 5 && !isValterControlDevicePort)
                    {
                        potentialControlDeviceSerialPort.write("GETID");
                        Valter::log(Valter::format_string("%s ← GETID", serialPortDeviceInfo.port.c_str()));
                        result = potentialControlDeviceSerialPort.readline();
                        sanitizeConrtolDeviceResponse(result);
                        Valter::log(Valter::format_string("%s → %s", serialPortDeviceInfo.port.c_str(), result.c_str()));
                        isValterControlDevicePort = (find(Valter::getInstance()->controlDeviceIds.begin(), Valter::getInstance()->controlDeviceIds.end(), result) != Valter::getInstance()->controlDeviceIds.end());
                        scanStep++;
                    }

                    if (isValterControlDevicePort)
                    {
                        potentialControlDeviceSerialPort.close();

                        Valter::getInstance()->addControlDevice(result, serialPortDeviceInfo.port);
                    }
                    else
                    {
                        potentialControlDeviceSerialPort.close();
                        //if ID 03eb:6125 Atmel Corp. detected
                        if (serialPortDeviceInfo.hardware_id.compare("03eb:6125"))
                        {
                            Valter::log("ID 03eb:6125 Atmel Corp. detected, resetting respective System Device");
                            string controlDeviceSysPathCmdWithDev = Valter::format_string(controlDeviceSysPathCmd, serialPortDeviceInfo.port.c_str());
                            string sys_usb_bus_device = Valter::exec_shell(controlDeviceSysPathCmdWithDev);
                            sys_usb_bus_device.erase(sys_usb_bus_device.length()-1);
                            USBSysDeviceReset(sys_usb_bus_device);
                            needsReScan = true;
                        }
                        else
                        {
                            Valter::log(Valter::format_string("Not a Valter Control Device\n"));
                        }
                    }
                }
                else
                {
                    Valter::log("busy port");
                }
            }
            catch (const exception &ex)
            {
                Valter::log(Valter::format_string("Exception [%s] fired while scanning ttyACM* ports...", ex.what()));
            }
        }
    }

    if (needsReScan) scanControlDevices();
}

void ControlDevice::reScanThisControlDevice()
{
    if (getControlDevicePort()->isOpen())
    {
        deactivate();
    }

    unsigned int cnt = getRescanNum();
    cnt++;
    this->setRescanNum(cnt);
    this->addMsgToDataExchangeLog(Valter::format_string("Rescanning Control Device [%s] port... Attempt #%d", this->getControlDeviceId().c_str(), cnt));

    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    bool rescanSuccessful = false;

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    bool portAlreadyOpenedAndInUseByAnotherControlDevice = false;

    while( iter != devices_found.end() )
    {
        serial::PortInfo serialPortDeviceInfo = *iter++;
        if (serialPortDeviceInfo.port.find("ttyACM") != string::npos)
        {
            portAlreadyOpenedAndInUseByAnotherControlDevice = false;
            for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
            {
                ControlDevice *controlDevice = controlDevicesMap[iterator->first];
                if (controlDevice->getControlDeviceId().compare(this->getControlDeviceId()) != 0) //ignore re-scanned Control Device
                {
                    if (controlDevice->getControlDevicePort()->getPort().compare(serialPortDeviceInfo.port) == 0 && controlDevice->getControlDevicePort()->isOpen())
                    {
                        this->addMsgToDataExchangeLog(Valter::format_string("Ignoring port %s which is opened and belongs to %s", controlDevice->getControlDevicePort()->getPort().c_str(), controlDevice->getControlDeviceId().c_str()));
                        portAlreadyOpenedAndInUseByAnotherControlDevice = true;
                    }
                }
            }
            if (!portAlreadyOpenedAndInUseByAnotherControlDevice)
            {
                this->addMsgToDataExchangeLog(Valter::format_string("Trying to connect %s port", serialPortDeviceInfo.port.c_str()));
                try
                {
                    try
                    {
                        this->controlDevicePort->close();
                    }
                    catch (const exception &ex){}

                    serial::Serial potentialControlDeviceSerialPort(serialPortDeviceInfo.port.c_str(), ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));

                    if(potentialControlDeviceSerialPort.isOpen())
                    {
                        potentialControlDeviceSerialPort.flush();

                        string result;
                        unsigned int scanStep = 0;
                        bool isValterRescanningControlDevice = false;
                        while (scanStep < 5 && !isValterRescanningControlDevice)
                        {
                            potentialControlDeviceSerialPort.write("GETID");
                            this->addMsgToDataExchangeLog(Valter::format_string("%s ← GETID", serialPortDeviceInfo.port.c_str()));
                            result = potentialControlDeviceSerialPort.readline();
                            sanitizeConrtolDeviceResponse(result);
                            this->addMsgToDataExchangeLog(Valter::format_string("%s → %s", serialPortDeviceInfo.port.c_str(), result.c_str()));
                            if (result.compare(this->getControlDeviceId()) == 0)
                            {
                                isValterRescanningControlDevice = true;
                            }
                            scanStep++;
                        }

                        if (isValterRescanningControlDevice)
                        {
                            potentialControlDeviceSerialPort.close();

                            Valter::getInstance()->updateControlDevice(result, serialPortDeviceInfo.port);

                            rescanSuccessful = true;

                            break;
                        }
                        else
                        {
                            this->addMsgToDataExchangeLog(Valter::format_string("Not a re-scanned [%s] Control Device\n", this->getControlDeviceId().c_str()));
                            try
                            {
                                potentialControlDeviceSerialPort.close();
                            }
                            catch (const exception &ex)
                            {
                                this->addMsgToDataExchangeLog(Valter::format_string("Exception [%s] fired while scanning %s port...", ex.what(), potentialControlDeviceSerialPort.getPort().c_str()));
                                continue;
                            }
                        }
                    }
                    else
                    {
                        this->addMsgToDataExchangeLog("busy port");
                    }
                }
                catch (const exception &ex)
                {
                    this->addMsgToDataExchangeLog(Valter::format_string("Exception [%s] fired while scanning ttyACM* ports...", ex.what()));
                    continue;
                }
            }
        }
    }
    if (rescanSuccessful)
    {
        this->addMsgToDataExchangeLog(Valter::format_string("Control Device [%s] re-scanned successfuly", this->getControlDeviceId().c_str()));
        setFailedAfterRescanning(false);
        setRescanningAfterPossibleReset(false);
    }
    else
    {
        if (cnt < 10)
        {
            this_thread::sleep_for(std::chrono::seconds(1));
            reScanThisControlDevice();
        }
        else
        {
            this->addMsgToDataExchangeLog(Valter::format_string("Rescanning Control Device [%s] port FAILED after Attempt #%d", this->getControlDeviceId().c_str(), cnt));
            setFailedAfterRescanning(true);
            setRescanningAfterPossibleReset(false);
        }
    }
}

void ControlDevice::USBSysDeviceReset(string sysDevicePath)
{
    Valter::log(Valter::format_string("Resetting %s system device", sysDevicePath.c_str()));

    int fd;
    fd = open(sysDevicePath.c_str(), O_WRONLY);
    if (fd < 0)
    {
        Valter::log(Valter::format_string("Error while opening %s system device file", sysDevicePath.c_str()));
        return;
    }
    int rc;
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0)
    {
        Valter::log(Valter::format_string("Error in IOCTL of %s system device file", sysDevicePath.c_str()));
        return;
    }
    Valter::log(Valter::format_string("%s system device file of Reset successful!", sysDevicePath.c_str()));
    close(fd);
}

serial::Serial *ControlDevice::getControlDevicePort() const
{
    return controlDevicePort;
}

void ControlDevice::setControlDevicePort(serial::Serial *value)
{
    controlDevicePort = value;
}

string ControlDevice::getControlDeviceId() const
{
    return controlDeviceId;
}

void ControlDevice::setControlDeviceId(const string &value)
{
    controlDeviceId = value;
}

string ControlDevice::sanitizeConrtolDeviceResponse(string &msg)
{
    if (!msg.empty() && msg[msg.length()-1] == '\n')
    {
        msg.erase(msg.length()-1);
    }
    return msg;
}

string ControlDevice::getSysDevicePath() const
{
    return sysDevicePath;
}

void ControlDevice::setSysDevicePath(const string &value)
{
    sysDevicePath = value;
}

void ControlDevice::controlDeviceThreadWorker()
{
    clearMessageQueue();

    unsigned int WDRESETTIMER = 0;
    bool USBRESET = false;
    bool RESCANAFTEREXCEPTION = false;

    this->addMsgToDataExchangeLog(Valter::format_string("%s worker started...", this->getControlDeviceId().c_str()));

    string request;
    string response;
    bool isWDReset = false;

    wdTimerNotResetCnt = 0;

    while (this->getStatus() == ControlDevice::StatusActive)
    {
        try
        {
            if (WDRESETTIMER == wdResetTime - 500)
            {
                isWDReset = false;
                if (resetWDTimer)
                {
                    this->getControlDevicePort()->write("WDRESET");
                    this->addMsgToDataExchangeLog(Valter::format_string("%s ← WDRESET", this->getControlDeviceId().c_str()));
                }
            }
            //process outgoing
            if (requestsAwainting() > 0)
            {
                request = pullRequest();
                if (request.length() == 0)
                {
                    this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                if (request.compare("WDINTENTIONALRESETON") == 0)
                {
                    intentionalWDTimerResetOnAT91SAM7s = true;
                }
                if (request.compare("WDINTENTIONALRESETOFF") == 0)
                {
                    intentionalWDTimerResetOnAT91SAM7s = false;
                }
                if (wdTimerNotResetCnt == 0)
                {
                    this->getControlDevicePort()->write(request.c_str());
                    if (Valter::getInstance()->getLogControlDeviceMessages())
                    {
                        this->addMsgToDataExchangeLog(Valter::format_string("%s ← %s", this->getControlDeviceId().c_str(), request.c_str()));
                    }
                }
            }
            //process incoming
            if (this->getControlDevicePort()->available() > 0)
            {
                response = this->getControlDevicePort()->readline();
                sanitizeConrtolDeviceResponse(response);
                addResponse(response);
                if (Valter::getInstance()->getLogControlDeviceMessages())
                {
                    this->addMsgToDataExchangeLog(Valter::format_string("%s → %s", this->getControlDeviceId().c_str(), response.c_str()));
                }
                if (response.compare("WDRST") == 0)
                {
                    isWDReset = true;
                    WDRESETTIMER = 0;
                    wdTimerNotResetCnt = 0;
                }
            }

            this_thread::sleep_for(std::chrono::milliseconds(1));

            WDRESETTIMER++;

            if (WDRESETTIMER > wdResetTime)
            {
                if (!isWDReset)
                {
                    if (resetWDTimer)
                    {
                        this->getControlDevicePort()->write("WDRESET");
                        this->addMsgToDataExchangeLog(Valter::format_string("%s ← WDRESET", this->getControlDeviceId().c_str()));
                    }
                    wdTimerNotResetCnt++;
                    this->addMsgToDataExchangeLog(Valter::format_string("WD was not Reset on %s Control Device [attempt #%d]", this->getControlDeviceId().c_str(), wdTimerNotResetCnt));
                    if (wdTimerNotResetCnt > 4)
                    {
                        if (intentionalWDTimerResetOnAT91SAM7s)
                        {
                            USBRESET = true;
                            break;
                        }
                        else
                        {
                            this->addMsgToDataExchangeLog(Valter::format_string("(EMULATED Reset %s system device of [%s] Control Device)", this->getSysDevicePath().c_str(), this->getControlDeviceId().c_str()));
                        }
                    }
                }
                WDRESETTIMER = 0;
            }
        }
        catch (const exception &ex)
        {
            this->addMsgToDataExchangeLog(Valter::format_string("Exception [%s] fired in the %s worker", ex.what(), this->getControlDeviceId().c_str()));
            RESCANAFTEREXCEPTION = true;
            break;
        }
    }

    wdTimerNotResetCnt = 0;

    this->addMsgToDataExchangeLog(Valter::format_string("%s worker stopped!", this->getControlDeviceId().c_str()));
    if (USBRESET)
    {
        setStatus(ControlDevice::StatusReady);
        intentionalWDTimerResetOnAT91SAM7s = false;
        rescanningAfterPossibleReset = true;
        this_thread::sleep_for(std::chrono::seconds(5));
        this->setRescanNum(0);
        resetUSBSysDevice();
    }
    if (RESCANAFTEREXCEPTION)
    {
        setStatus(ControlDevice::StatusReady);
        intentionalWDTimerResetOnAT91SAM7s = false;
        rescanningAfterPossibleReset = true;
        this_thread::sleep_for(std::chrono::seconds(5));
        this->setRescanNum(0);
        reScanThisControlDevice();
    }
}

void ControlDevice::resetUSBSysDevice()
{
    this->addMsgToDataExchangeLog(Valter::format_string("Resetting %s system device of [%s] Control Device", this->getSysDevicePath().c_str(), this->getControlDeviceId().c_str()));

    int fd;
    fd = open(this->getSysDevicePath().c_str(), O_WRONLY);
    if (fd < 0)
    {
        this->addMsgToDataExchangeLog(Valter::format_string("Error while opening %s system device file of [%s] Control Device", this->getSysDevicePath().c_str(), this->getControlDeviceId().c_str()));
        new std::thread(&ControlDevice::reScanThisControlDevice, this);
        return;
    }
    int rc;
    rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0)
    {
        this->addMsgToDataExchangeLog(Valter::format_string("Error in IOCTL of %s system device file of [%s] Control Device", this->getSysDevicePath().c_str(), this->getControlDeviceId().c_str()));
        new std::thread(&ControlDevice::reScanThisControlDevice, this);
        return;
    }
    this->addMsgToDataExchangeLog(Valter::format_string("%s system device file of [%s] Control Device Reset successful!", this->getSysDevicePath().c_str(), this->getControlDeviceId().c_str()));
    close(fd);

    reScanThisControlDevice();
}

unsigned char ControlDevice::getWdTimerNotResetCnt() const
{
    return wdTimerNotResetCnt;
}

void ControlDevice::setWdTimerNotResetCnt(unsigned char value)
{
    wdTimerNotResetCnt = value;
}
bool ControlDevice::getAutoReActivation() const
{
    return autoReActivation;
}

void ControlDevice::setAutoReActivation(bool value)
{
    autoReActivation = value;
}

bool ControlDevice::getIntentionalWDTimerResetOnAT91SAM7s() const
{
    return intentionalWDTimerResetOnAT91SAM7s;
}

void ControlDevice::setIntentionalWDTimerResetOnAT91SAM7s(bool value)
{
    intentionalWDTimerResetOnAT91SAM7s = value;
}

unsigned int ControlDevice::getRescanNum() const
{
    return rescanNum;
}

void ControlDevice::setRescanNum(unsigned int value)
{
    rescanNum = value;
}


bool ControlDevice::getFailedAfterRescanning() const
{
    return failedAfterRescanning;
}

void ControlDevice::setFailedAfterRescanning(bool value)
{
    failedAfterRescanning = value;
}

bool ControlDevice::getRescanningAfterPossibleReset() const
{
    return rescanningAfterPossibleReset;
}

void ControlDevice::setRescanningAfterPossibleReset(bool value)
{
    rescanningAfterPossibleReset = value;
    if (rescanningAfterPossibleReset)
    {
        setStatus(ControlDevice::StatusReady);
    }
    else
    {
        IValterModule *valterModule = Valter::getInstance()->getValterModule(getControlDeviceId());
        if (valterModule->getReloadDefaults())
        {
            valterModule->loadDefaults();
            valterModule->addActionToDelayedGUIActions(IValterModule::RELOAD_DEFAULTS);
        }
        if (autoReActivation)
        {
            if (!failedAfterRescanning)
            {
                activate();
            }
        }
    }
}

bool ControlDevice::getResetWDTimer() const
{
    return resetWDTimer;
}

void ControlDevice::setResetWDTimer(bool value)
{
    resetWDTimer = value;
}

std::thread *ControlDevice::getControlDeviceThread() const
{
    return controlDeviceThread;
}

void ControlDevice::setControlDeviceThread(std::thread *value)
{
    controlDeviceThread = value;
}

void ControlDevice::spawnControlDeviceThreadWorker()
{
    controlDeviceThread = new std::thread(&ControlDevice::controlDeviceThreadWorker, this);
    IValterModule *valterModule = Valter::getInstance()->getValterModule(getControlDeviceId());
    valterModule->spawnProcessMessagesQueueWorkerThread();
}

void ControlDevice::addRequest(string msg)
{
    if (controlDevicePort->isOpen())
    {
        std::lock_guard<std::mutex> guard(requests_mutex);
        requests.push_back(msg);
    }
    else
    {
        Valter::log(this->getControlDeviceId() + " port is closed");
    }
}

void ControlDevice::addResponse(string msg)
{
    std::lock_guard<std::mutex> guard(responses_mutex);
    responses.push_back(msg);
}

string ControlDevice::pullRequest()
{
    if (requests.size() > 0)
    {
        std::lock_guard<std::mutex> guard(requests_mutex);
        string request = (string)requests.front();
        requests.erase(requests.begin());
        return request;
    }
    return "";
}

string ControlDevice::pullResponse()
{
    if (!responses.empty())
    {
        std::lock_guard<std::mutex> guard(responses_mutex);
        string response = (string)responses.front();
        responses.erase(responses.begin());
        return response;
    }
    return "";
}

int ControlDevice::responsesAvailable()
{
    if (!responses.empty())
    {
        return (int)responses.size();
    }
    return 0;
}

int ControlDevice::requestsAwainting()
{
    return (int)requests.size();
}

void ControlDevice::clearMessageQueue()
{
    responses.clear();
    requests.clear();
}

void ControlDevice::addMsgToDataExchangeLog(string msg)
{
    if (Valter::getInstance()->getLogControlDeviceMessages())
    {
        std::lock_guard<std::mutex> guard(dataExchangeLog_mutex);
        dataExchangeLog.push_back(msg);
    }
}

string ControlDevice::getMsgFromDataExchangeLog()
{
    if (!dataExchangeLog.empty())
    {
        std::lock_guard<std::mutex> guard(dataExchangeLog_mutex);
        string logMsg = (string)dataExchangeLog.front();
        dataExchangeLog.erase(dataExchangeLog.begin());
        return logMsg;
    }
    return "";
}

int ControlDevice::dataExchangeLogAvailable()
{
    return (int)dataExchangeLog.size();
}

void ControlDevice::clearDataExchangeLog()
{
    dataExchangeLog.clear();
}

string ControlDevice::getStatus() const
{
    return status;
}

void ControlDevice::setStatus(const string &value)
{
    status = value;
}

void ControlDevice::activate()
{
    if (getStatus() == ControlDevice::StatusReady)
    {
        try
        {
            getControlDevicePort()->open();
        }
        catch (const exception &ex)
        {
            this->addMsgToDataExchangeLog(Valter::format_string("Exception [%s] fired in the %s worker", ex.what(), this->getControlDeviceId().c_str()));
        }
        setStatus(ControlDevice::StatusActive);
        spawnControlDeviceThreadWorker();
    }
}

void ControlDevice::deactivate()
{
    setStatus(ControlDevice::StatusReady);
    getControlDevicePort()->close();
}
