//Qt specific
#include <QtDebug>

#include "controldevice.h"
#include "valter.h"

const uint32_t ControlDevice::DefaultBaudRate = 115200;

const string ControlDevice::StatusActive        = "active";
const string ControlDevice::StatusReady         = "ready";

ControlDevice::ControlDevice()
{

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
            Valter::log(Valter::format_string("(%s, %s, %s)", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() ));
            ttyACMPortsNum++;
        }
    }
    Valter::log(Valter::format_string("%u ttyACM* ports found", ttyACMPortsNum));
}

void ControlDevice::scanControlDevices()
{
    unsigned char ttyACMDevicesNum = 0;


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
                    potentialControlDeviceSerialPort.write("GETID");
                    Valter::log(Valter::format_string("%s ← GETID", serialPortDeviceInfo.port.c_str()));

                    string result;
                    unsigned int scanStep = 0;
                    bool isValterControlDevicePort = false;
                    while (scanStep < 5 && !isValterControlDevicePort)
                    {
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
                        Valter::log(Valter::format_string("Not a Valter Control Device\n"));
                        potentialControlDeviceSerialPort.close();
                    }
                }
                else
                {
                    Valter::log("busy port");
                }
            }
            catch (...)
            {
                Valter::log("exception fired while scanning ttyACM* ports...");
            }
        }
    }
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

void ControlDevice::controlDeviceThreadWorker()
{
    this->addMsgToDataExchangeLog(Valter::format_string("%s worker started...", this->getControlDeviceId().c_str()));

    string request;
    string response;

    while (this->getStatus() == ControlDevice::StatusActive)
    {
        if (this->getControlDevicePort()->available() > 0)
        {
            response = this->getControlDevicePort()->readline();
            sanitizeConrtolDeviceResponse(response);
            addResponse(response);
            if (Valter::getInstance()->getLogControlDeviceMessages())
            {
                this->addMsgToDataExchangeLog(Valter::format_string("%s → %s", this->getControlDeviceId().c_str(), response.c_str()));
            }
        }
        if (requestsAwainting() > 0)
        {
            string request = pullRequest();
            this->getControlDevicePort()->write(request.c_str());
            if (Valter::getInstance()->getLogControlDeviceMessages())
            {
                this->addMsgToDataExchangeLog(Valter::format_string("%s ← %s", this->getControlDeviceId().c_str(), request.c_str()));
            }
        }

        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    this->addMsgToDataExchangeLog(Valter::format_string("%s worker stopped...", this->getControlDeviceId().c_str()));
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
}

void ControlDevice::addRequest(string msg)
{
    if (controlDevicePort->isOpen())
    {
        requests.push_back(msg);
    }
    else
    {
        Valter::log(this->getControlDeviceId() + " port is closed");
    }
}

void ControlDevice::addResponse(string msg)
{
    responses.push_back(msg);
}

string ControlDevice::pullRequest()
{
    string request = (string)requests.front();
    requests.pop_front();
    return request;
}

string ControlDevice::pullResponse()
{
    string response = (string)responses.front();
    responses.pop_front();
    return response;
}

int ControlDevice::responsesAvailable()
{
    return (int)responses.size();
}

int ControlDevice::requestsAwainting()
{
    return (int)requests.size();
}

void ControlDevice::addMsgToDataExchangeLog(string msg)
{
    if (Valter::getInstance()->getLogControlDeviceMessages())
    {
        dataExchangeLog.push_back(msg);
    }
}

string ControlDevice::getMsgFromDataExchangeLog()
{
    string logMsg = (string)dataExchangeLog.front();
    dataExchangeLog.pop_front();
    return logMsg;
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
