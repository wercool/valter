#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <QtSerialPort/QtSerialPort>

using namespace std;

class ControlDevice
{
public:
    ControlDevice();
    static void listDevices(bool fullInfo = false);
};

#endif // CONTROLDEVICE_H
