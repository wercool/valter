//Qt specific
#include <QtDebug>

#include "controldevice.h"
#include "valter.h"

const uint32_t ControlDevice::DefaultBaudRate = 115200;

const string ControlDevice::StatusActive = "active";
const string ControlDevice::StatusReady  = "ready";
const string ControlDevice::StatusBusy   = "busy";

ControlDevice::ControlDevice()
{

}

void ControlDevice::listDevices(bool fullInfo)
{
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    unsigned char ttyACMPortsNum = 0;

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;
        if (fullInfo)
        {
            qDebug( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
        }
        else
        {
            if (device.port.find("ttyACM") != std::string::npos)
            {
                qDebug( "(%s, %s, %s)", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
                ttyACMPortsNum++;
            }
        }
    }
    qDebug("%u ttyACM* ports found\n\n", ttyACMPortsNum);
}

vector<ControlDevice> ControlDevice::scanControlDevices()
{
    vector<ControlDevice> controlDevices;

    unsigned char ttyACMDevicesNum = 0;


    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo serialPortDeviceInfo = *iter++;
        if (serialPortDeviceInfo.port.find("ttyACM") != std::string::npos)
        {
            ttyACMDevicesNum++;
            qDebug("Trying to connect %s port", serialPortDeviceInfo.port.c_str());
            serial::Serial potentialControlDeviceSerialPort(serialPortDeviceInfo.port.c_str(), ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));

            if(potentialControlDeviceSerialPort.isOpen())
            {
                potentialControlDeviceSerialPort.flush();
                potentialControlDeviceSerialPort.write("GETID");
                qDebug("< GETID");

                std::string result;
                result = potentialControlDeviceSerialPort.readline();
                sanitizeConrtolDeviceResponse(result);
                qDebug("> %s", result.c_str());

                result = potentialControlDeviceSerialPort.readline();
                sanitizeConrtolDeviceResponse(result);

                qDebug("> %s", result.c_str());

                bool isValterControlDevicePort = (std::find(Valter::getInstance()->controlDeviceIds.begin(), Valter::getInstance()->controlDeviceIds.end(), result) != Valter::getInstance()->controlDeviceIds.end());

                if (isValterControlDevicePort)
                {
                    ControlDevice cd = ControlDevice();
                    cd.setControlDeviceName(result);
                    cd.setControlDevicePort(&potentialControlDeviceSerialPort);
                    cd.setControlDevicePortInfo(serialPortDeviceInfo);
                    cd.getControlDevicePort()->close();
                    cd.setStatus(ControlDevice::StatusReady);

                    qDebug("Control Device [%s] added to Valter\n\n", cd.getControlDeviceName().c_str());

                    controlDevices.push_back(cd);
                }
                else
                {
                    qDebug("Not a Valter Control Device");
                    potentialControlDeviceSerialPort.close();
                }
            }
            else
            {
                qDebug("busy port");
            }
        }
    }

    qDebug("%u ttyACM* devices scanned, %zu of them are Valter Control Devices\n\n", ttyACMDevicesNum, controlDevices.size()*1);

    return controlDevices;
}


serial::PortInfo ControlDevice::getControlDevicePortInfo() const
{
    return controlDevicePortInfo;
}

void ControlDevice::setControlDevicePortInfo(const serial::PortInfo &value)
{
    controlDevicePortInfo = value;
}

std::string ControlDevice::getStatus() const
{
    return status;
}

void ControlDevice::setStatus(const std::string &value)
{
    status = value;
}

serial::Serial *ControlDevice::getControlDevicePort() const
{
    return controlDevicePort;
}

void ControlDevice::setControlDevicePort(serial::Serial *value)
{
    controlDevicePort = value;
}

string ControlDevice::sanitizeConrtolDeviceResponse(string &msg)
{
    if (!msg.empty() && msg[msg.length()-1] == '\n')
    {
        msg.erase(msg.length()-1);
    }
    return msg;
}

void ControlDevice::readControlDeviceOutputWorker()
{

}

void ControlDevice::runReadControlDeviceOutputWorker()
{
    *readControlDeviceOutputThread = std::thread(&ControlDevice::readControlDeviceOutputWorker, this);
}


std::string ControlDevice::getControlDeviceName() const
{
    return controlDeviceName;
}

void ControlDevice::setControlDeviceName(const std::string &value)
{
    controlDeviceName = value;
}
