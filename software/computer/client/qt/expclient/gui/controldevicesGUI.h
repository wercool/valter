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
    typedef map<string, ControlDevice*>::iterator it_type;
    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    if (controlDevicesMap.size() > 0)
    {
        ui->controlDeviceTableWidget->clearContents();

        ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

        ui->controlDeviceTCPInterfaceTable->clearContents();
        ui->controlDeviceTCPInterfaceTable->setRowCount(controlDevicesMap.size());

        char i = 0;
        for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
        {
            ControlDevice *controlDevice = controlDevicesMap[iterator->first];
            ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort().c_str()));
            ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
            ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "TRUE" :"FALSE")));
            ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getIntentionalWDTimerResetOnAT91SAM7s() ? "ON" :"OFF"));
            ui->controlDeviceTableWidget->setItem(i, 4, new QTableWidgetItem(controlDevice->getSysDevicePath().c_str()));

            if (Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->getTcpInterface())
            {
                ui->controlDeviceTCPInterfaceTable->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
                ui->controlDeviceTCPInterfaceTable->setItem(i, 1, new QTableWidgetItem(Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->getTcpInterface()->getIp().c_str()));
                ui->controlDeviceTCPInterfaceTable->setItem(i, 2, new QTableWidgetItem(Valter::format_string("%d", Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId())->getTcpInterface()->getPort()).c_str()));
                ui->controlDeviceTCPInterfaceTable->setItem(i, 3, new QTableWidgetItem(controlDevice->getStatus().c_str()));
            }

            i++;
        }
    }
    else
    {
        map<string, ControlDevice*> remoteControlDevicesMap = Valter::getInstance()->getRemoteControlDevicesMap();

        if (remoteControlDevicesMap.size() > 0)
        {
            ui->controlDeviceTCPInterfaceTable->clearContents();
            ui->controlDeviceTCPInterfaceTable->setRowCount(remoteControlDevicesMap.size());

            char i = 0;
            for(it_type iterator = remoteControlDevicesMap.begin(); iterator != remoteControlDevicesMap.end(); iterator++)
            {
                ControlDevice *remoteControlDevice = remoteControlDevicesMap[iterator->first];

                ui->controlDeviceTCPInterfaceTable->setItem(i, 0, new QTableWidgetItem(remoteControlDevice->getControlDeviceId().c_str()));
                ui->controlDeviceTCPInterfaceTable->setItem(i, 1, new QTableWidgetItem(remoteControlDevice->getRemoteIPAddress().c_str()));
                ui->controlDeviceTCPInterfaceTable->setItem(i, 2, new QTableWidgetItem(Valter::format_string("%d", remoteControlDevice->getRemotePort()).c_str()));
                ui->controlDeviceTCPInterfaceTable->setItem(i, 3, new QTableWidgetItem(remoteControlDevice->getRemoteStatus().c_str()));

                i++;
            }
        }
    }
}
#endif // CONTROLDEVICESGUI_H
