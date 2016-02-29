#include "controldevice.h"

ControlDevice::ControlDevice()
{

}

void ControlDevice::listDevices(bool fullInfo)
{
    QTextStream out(stdout);

    QList<QSerialPortInfo> serialPortInfoList = QSerialPortInfo::availablePorts();

    out << QObject::tr("Total number of ports available: ") << serialPortInfoList.count() << endl;

    const QString blankString = QObject::tr("N/A");
    QString description;
    QString manufacturer;
    QString serialNumber;

    foreach (const QSerialPortInfo &serialPortInfo, serialPortInfoList)
    {
        if (serialPortInfo.portName().indexOf("ttyACM") > -1)
        {
            if (fullInfo)
            {
                description = serialPortInfo.description();
                manufacturer = serialPortInfo.manufacturer();
                serialNumber = serialPortInfo.serialNumber();
                out << endl
                    << "Control devices' ports: "
                    << endl
                    << QObject::tr("Port: ") << serialPortInfo.portName() << endl
                    << QObject::tr("Location: ") << serialPortInfo.systemLocation() << endl
                    << QObject::tr("Description: ") << (!description.isEmpty() ? description : blankString) << endl
                    << QObject::tr("Manufacturer: ") << (!manufacturer.isEmpty() ? manufacturer : blankString) << endl
                    << QObject::tr("Serial number: ") << (!serialNumber.isEmpty() ? serialNumber : blankString) << endl
                    << QObject::tr("Vendor Identifier: ") << (serialPortInfo.hasVendorIdentifier() ? QByteArray::number(serialPortInfo.vendorIdentifier(), 16) : blankString) << endl
                    << QObject::tr("Product Identifier: ") << (serialPortInfo.hasProductIdentifier() ? QByteArray::number(serialPortInfo.productIdentifier(), 16) : blankString) << endl
                    << QObject::tr("Busy: ") << (serialPortInfo.isBusy() ? QObject::tr("Yes") : QObject::tr("No")) << endl;
            }
            else
            {
                out << endl
                    << "Control devices' ports: "
                    << endl
                    << QObject::tr("Location: ") << serialPortInfo.systemLocation() << endl
                    << QObject::tr("Busy: ") << (serialPortInfo.isBusy() ? QObject::tr("Yes") : QObject::tr("No")) << endl;
            }
        }
    }
}
