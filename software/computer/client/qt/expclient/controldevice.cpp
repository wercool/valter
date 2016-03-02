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
            if (device.port.find("ttyACM") != string::npos)
            {
                qDebug( "(%s, %s, %s)", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
                ttyACMPortsNum++;
            }
        }
    }
    qDebug("%u ttyACM* ports found\n\n", ttyACMPortsNum);
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
            qDebug("Trying to connect %s port", serialPortDeviceInfo.port.c_str());
            serial::Serial potentialControlDeviceSerialPort(serialPortDeviceInfo.port.c_str(), ControlDevice::DefaultBaudRate, serial::Timeout::simpleTimeout(500));

            if(potentialControlDeviceSerialPort.isOpen())
            {
                potentialControlDeviceSerialPort.flush();
                potentialControlDeviceSerialPort.write("GETID");
                qDebug("< GETID");

                string result;
                unsigned int scanStep = 0;
                bool isValterControlDevicePort = false;
                while (scanStep < 5 && !isValterControlDevicePort)
                {
                    result = potentialControlDeviceSerialPort.readline();
                    sanitizeConrtolDeviceResponse(result);
                    qDebug("> %s", result.c_str());
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
                    qDebug("Not a Valter Control Device\n");
                    potentialControlDeviceSerialPort.close();
                }
            }
            else
            {
                qDebug("busy port");
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

string ControlDevice::getStatus() const
{
    return status;
}

void ControlDevice::setStatus(const string &value)
{
    status = value;
}
