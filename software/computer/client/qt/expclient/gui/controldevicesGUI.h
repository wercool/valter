#ifndef CONTROLDEVICESGUI_H
#define CONTROLDEVICESGUI_H
#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void refreshControlDeviceTableWorker(Ui::MainWindow *ui)
{
    ui->controlDeviceTableWidget->clearContents();

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

    ui->controlDeviceTCPInterfaceTable->clearContents();
    ui->controlDeviceTCPInterfaceTable->setRowCount(controlDevicesMap.size());

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "TRUE" :"FALSE")));
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getIntentionalWDTimerResetOnAT91SAM7s() ? "ON" :"OFF"));
        ui->controlDeviceTableWidget->setItem(i, 4, new QTableWidgetItem(controlDevice->getSysDevicePath().c_str()));

        if (Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->getTcpInterface())
        {
            ui->controlDeviceTCPInterfaceTable->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
            ui->controlDeviceTCPInterfaceTable->setItem(i, 1, new QTableWidgetItem(Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->getTcpInterface()->getIp().c_str()));
            ui->controlDeviceTCPInterfaceTable->setItem(i, 2, new QTableWidgetItem(Valter::format_string("%d", Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->getTcpInterface()->getPort()).c_str()));
        }

        i++;
    }
}
#endif // CONTROLDEVICESGUI_H
