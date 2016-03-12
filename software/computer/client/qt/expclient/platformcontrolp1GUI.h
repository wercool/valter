#ifndef MAINWINDOWUTILS_H
#define MAINWINDOWUTILS_H

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

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "TRUE" :"FALSE")));
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getIntentionalWDTimerResetOnAT91SAM7s() ? "ON" :"OFF"));
        ui->controlDeviceTableWidget->setItem(i, 4, new QTableWidgetItem(controlDevice->getSysDevicePath().c_str()));
        i++;
    }
}

void controlDevicesTableRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    int selectedControlDeviceRowIndex = ui->controlDeviceTableWidget->selectionModel()->currentIndex().row();

    ui->controlDeviceTableWidget->clearContents();

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        if (controlDevice->getRescanningAfterPossibleReset())
        {
            string rescanningMsg = Valter::format_string("rescanning... Attempt #%d", controlDevice->getRescanNum());
            QTableWidgetItem *item = new QTableWidgetItem(rescanningMsg.c_str());
            item->setBackground(Qt::yellow);
            ui->controlDeviceTableWidget->setItem(i, 2, item);
        }
        else
        {
            if (controlDevice->getWdTimerNotResetCnt() > 0)
            {
                string noWDResetMsg = Valter::format_string("no WDRESET answer... Attempt #%d", controlDevice->getWdTimerNotResetCnt());
                QTableWidgetItem *item = new QTableWidgetItem(noWDResetMsg.c_str());
                item->setBackground(Qt::yellow);
                ui->controlDeviceTableWidget->setItem(i, 2, item);
            }
            else if (controlDevice->getFailedAfterRescanning())
            {
                controlDevice->setWdTimerNotResetCnt(0);
                QTableWidgetItem *item = new QTableWidgetItem("FAILED");
                item->setBackground(Qt::red);
                ui->controlDeviceTableWidget->setItem(i, 2, item);
            }
            else
            {
                ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "TRUE" :"FALSE")));
            }
        }
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getIntentionalWDTimerResetOnAT91SAM7s() ? "ON" :"OFF"));
        ui->controlDeviceTableWidget->setItem(i, 4, new QTableWidgetItem(controlDevice->getSysDevicePath().c_str()));
        i++;
    }

    if (selectedControlDeviceRowIndex >= 0)
    {
        ui->controlDeviceTableWidget->selectRow(selectedControlDeviceRowIndex);
    }
}

void loadPlatformControlP1Defaults(Ui::MainWindow *ui)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());

    //platformControlP1WheelMotorsDutySlider
    ui->platformControlP1WheelMotorsDutySlider->setMinimum(platformControlP1->getLeftMotorDutyPresetMin()); //use left preset as common
    ui->platformControlP1WheelMotorsDutySlider->setMaximum(platformControlP1->getLeftMotorDutyPresetMax()); //use left preset as common
    if (platformControlP1->getLeftMotorDutyPresetCur() > platformControlP1->getRightMotorDutyPresetCur())
    {
        ui->platformControlP1WheelMotorsDutySlider->setValue(platformControlP1->getLeftMotorDutyPresetCur());
    }
    else
    {
        ui->platformControlP1WheelMotorsDutySlider->setValue(platformControlP1->getRightMotorDutyPresetCur());
    }

    //platformMovementDecelerationSlider
    ui->platformMovementDecelerationSlider->setMinimum(platformControlP1->getPlatformDecelerationPresetMin());
    ui->platformMovementDecelerationSlider->setMaximum(platformControlP1->getPlatformDecelerationPresetMax());
    ui->platformMovementDecelerationSlider->setValue(platformControlP1->getPlatformDecelerationPresetCur());
    ui->platformControlP1WheelMotorsDecelerationLabel->setText(Valter::format_string("[%d]", platformControlP1->getPlatformDecelerationPresetCur()).c_str());

    //platformMovementAccelerationSlider
    ui->platformMovementAccelerationSlider->setMinimum(platformControlP1->getPlatformAccelerationPresetMin());
    ui->platformMovementAccelerationSlider->setMaximum(platformControlP1->getPlatformAccelerationPresetMax());
    ui->platformMovementAccelerationSlider->setValue(platformControlP1->getPlatformAccelerationPresetCur());
    ui->platformControlP1WheelMotorsAccelerationLabel->setText(Valter::format_string("[%d]", platformControlP1->getPlatformAccelerationPresetCur()).c_str());

    //leftMotorPlatformControlP1DutySlider
    ui->leftMotorPlatformControlP1DutySlider->setMinimum(platformControlP1->getLeftMotorDutyPresetMin());
    ui->leftMotorPlatformControlP1DutySlider->setMaximum(platformControlP1->getLeftMotorDutyPresetMax());
    ui->leftMotorPlatformControlP1DutySlider->setValue(platformControlP1->getLeftMotorDutyPresetCur());

    //rightMotorPlatformControlP1DutySlider
    ui->rightMotorPlatformControlP1DutySlider->setMinimum(platformControlP1->getRightMotorDutyPresetMin());
    ui->rightMotorPlatformControlP1DutySlider->setMaximum(platformControlP1->getRightMotorDutyPresetMax());
    ui->rightMotorPlatformControlP1DutySlider->setValue(platformControlP1->getRightMotorDutyPresetCur());

    //turretRotationDutySlider
    ui->turretRotationDutySlider->setMinimum(platformControlP1->getTurretMotorDutyPresetMin());
    ui->turretRotationDutySlider->setMaximum(platformControlP1->getTurretMotorDutyPresetMax());
    ui->turretRotationDutySlider->setValue(platformControlP1->getTurretMotorDutyPresetCur());

    //decelerationTurretRotationSlider
    ui->decelerationTurretRotationSlider->setMinimum(platformControlP1->getTurretDecelerationPresetMin());
    ui->decelerationTurretRotationSlider->setMaximum(platformControlP1->getTurretDecelerationPresetMax());
    ui->decelerationTurretRotationSlider->setValue(platformControlP1->getTurretDecelerationPresetCur());
    ui->platformControlP1TurretMotorDecelerationLabel->setText(Valter::format_string("[%d]", platformControlP1->getTurretDecelerationPresetCur()).c_str());

    //accelerationTurretRotationSlider
    ui->accelerationTurretRotationSlider->setMinimum(platformControlP1->getTurretAccelerationPresetMin());
    ui->accelerationTurretRotationSlider->setMaximum(platformControlP1->getTurretAccelerationPresetMax());
    ui->accelerationTurretRotationSlider->setValue(platformControlP1->getTurretAccelerationPresetCur());
    ui->platformControlP1TurretMotorAccelerationLabel->setText(Valter::format_string("[%d]", platformControlP1->getTurretAccelerationPresetCur()).c_str());

    //leftMotorCurrentCheckBox
    ui->leftMotorCurrentCheckBox->setChecked(platformControlP1->getLeftMotorCurrentRead());
    //rightMotorCurrentCheckBox
    ui->rightMotorCurrentCheckBox->setChecked(platformControlP1->getRightMotorCurrentRead());
    //turretMotorCurrentCheckBox
    ui->turretMotorCurrentCheckBox->setChecked(platformControlP1->getTurretMotorCurrentRead());

    //platformControlP1LeftWheelEncoderCheckBox
    ui->platformControlP1LeftWheelEncoderCheckBox->setChecked(platformControlP1->getLeftWheelEncoderRead());
    //platformControlP1RightWheelEncoderCheckBox
    ui->platformControlP1RightWheelEncoderCheckBox->setChecked(platformControlP1->getRightWheelEncoderRead());
    //platformControlP1LeftWheelEncoderAutoresetCheckBox
    ui->platformControlP1LeftWheelEncoderAutoresetCheckBox->setChecked(platformControlP1->getLeftWheelEncoderAutoreset());
    //platformControlP1RightWheelEncoderAutoresetCheckBox
    ui->platformControlP1RightWheelEncoderAutoresetCheckBox->setChecked(platformControlP1->getRightWheelEncoderAutoreset());

    //turretPositionReadCheckBox
    ui->turretPositionReadCheckBox->setChecked(platformControlP1->getTurretPositionRead());

    //platformControlP1ReadingsTable
    QTableWidgetItem* mainAccumulatorVoltageReadADCPreset = ui->platformControlP1ReadingsTable->item(0, 2);
    QTableWidgetItem* leftAccumulatorVoltageReadADCPreset = ui->platformControlP1ReadingsTable->item(1, 2);
    QTableWidgetItem* rightAccumulatorVoltageReadADCPreset = ui->platformControlP1ReadingsTable->item(2, 2);
    QTableWidgetItem* mainAccumulatorAmperageTotalReadADCPreset = ui->platformControlP1ReadingsTable->item(3, 2);
    QTableWidgetItem* platformAmperageReadADCPreset = ui->platformControlP1ReadingsTable->item(4, 2);
    QTableWidgetItem* bodyAmperageReadADCPreset = ui->platformControlP1ReadingsTable->item(5, 2);
    QTableWidgetItem* leftAccumulatorAmperageReadADCPreset = ui->platformControlP1ReadingsTable->item(6, 2);
    QTableWidgetItem* rightAccumulatorAmperageReadADCPreset = ui->platformControlP1ReadingsTable->item(7, 2);
    QTableWidgetItem* chargerVoltageReadADCPreset = ui->platformControlP1ReadingsTable->item(8, 2);

    mainAccumulatorVoltageReadADCPreset->setCheckState((platformControlP1->getMainAccumulatorVoltageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    leftAccumulatorVoltageReadADCPreset->setCheckState((platformControlP1->getLeftAccumulatorVoltageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    rightAccumulatorVoltageReadADCPreset->setCheckState((platformControlP1->getRightAccumulatorVoltageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    mainAccumulatorAmperageTotalReadADCPreset->setCheckState((platformControlP1->getMainAccumulatorAmperageTotalReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    platformAmperageReadADCPreset->setCheckState((platformControlP1->getPlatformAmperageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    bodyAmperageReadADCPreset->setCheckState((platformControlP1->getBodyAmperageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    leftAccumulatorAmperageReadADCPreset->setCheckState((platformControlP1->getLeftAccumulatorAmperageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    rightAccumulatorAmperageReadADCPreset->setCheckState((platformControlP1->getRightAccumulatorAmperageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);
    chargerVoltageReadADCPreset->setCheckState((platformControlP1->getChargerVoltageReadADCPreset()) ? Qt::Checked : Qt::Unchecked);

    QTableWidgetItem* mainAccumulatorVoltageRead = ui->platformControlP1ReadingsTable->item(0, 3);
    QTableWidgetItem* leftAccumulatorVoltageRead = ui->platformControlP1ReadingsTable->item(1, 3);
    QTableWidgetItem* rightAccumulatorVoltageRead = ui->platformControlP1ReadingsTable->item(2, 3);
    QTableWidgetItem* mainAccumulatorAmperageTotalRead = ui->platformControlP1ReadingsTable->item(3, 3);
    QTableWidgetItem* platformAmperageRead = ui->platformControlP1ReadingsTable->item(4, 3);
    QTableWidgetItem* bodyAmperageRead = ui->platformControlP1ReadingsTable->item(5, 3);
    QTableWidgetItem* leftAccumulatorAmperageRead = ui->platformControlP1ReadingsTable->item(6, 3);
    QTableWidgetItem* rightAccumulatorAmperageRead = ui->platformControlP1ReadingsTable->item(7, 3);
    QTableWidgetItem* chargerVoltageRead = ui->platformControlP1ReadingsTable->item(8, 3);

    mainAccumulatorVoltageRead->setCheckState((platformControlP1->getMainAccumulatorVoltageRead()) ? Qt::Checked : Qt::Unchecked);
    leftAccumulatorVoltageRead->setCheckState((platformControlP1->getLeftAccumulatorVoltageRead()) ? Qt::Checked : Qt::Unchecked);
    rightAccumulatorVoltageRead->setCheckState((platformControlP1->getRightAccumulatorVoltageRead()) ? Qt::Checked : Qt::Unchecked);
    mainAccumulatorAmperageTotalRead->setCheckState((platformControlP1->getMainAccumulatorAmperageTotalRead()) ? Qt::Checked : Qt::Unchecked);
    platformAmperageRead->setCheckState((platformControlP1->getPlatformAmperageRead()) ? Qt::Checked : Qt::Unchecked);
    bodyAmperageRead->setCheckState((platformControlP1->getBodyAmperageRead()) ? Qt::Checked : Qt::Unchecked);
    leftAccumulatorAmperageRead->setCheckState((platformControlP1->getLeftAccumulatorAmperageRead()) ? Qt::Checked : Qt::Unchecked);
    rightAccumulatorAmperageRead->setCheckState((platformControlP1->getRightAccumulatorAmperageRead()) ? Qt::Checked : Qt::Unchecked);
    chargerVoltageRead->setCheckState((platformControlP1->getChargerVoltageRead()) ? Qt::Checked : Qt::Unchecked);
}

void platformControlP1TabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    if (ui->mainTabWidget->currentIndex() == 0) //PLATFROM-CONROL-P1 Tab
    {
        PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
        if (!platformControlP1->getScan220ACAvailable())
        {
            ui->power220VACAvailableRadioButton->setChecked(false);
            ui->power220VACAvailableRadioButton->setEnabled(false);
            ui->chargerVoltageLabel->setEnabled(false);
            ui->chargerVoltageADCCheckBox->setEnabled(false);
            ui->chargerVoltageLcdNumber->setEnabled(false);
            ui->charger35AhRadioButton->setEnabled(false);
            ui->charger35AhRadioButton->setChecked(false);
            ui->charger120AhRadioButton->setEnabled(false);
            ui->charger120AhRadioButton->setChecked(false);
            ui->chargingInProgressRadioButton->setEnabled(false);
            ui->chargingInProgressRadioButton->setChecked(false);
            ui->chargingCompleteRadioButton->setEnabled(false);
            ui->chargingCompleteRadioButton->setChecked(false);
            ui->chargerConnectedRadioButton->setEnabled(false);
            ui->chargerConnectedRadioButton->setChecked(false);
            ui->charger120Ah14v7RadioButton->setEnabled(false);
            ui->chargerButton->setEnabled(false);
            platformControlP1->setChargerVoltageADC(0);
        }
        else
        {
            ui->power220VACAvailableRadioButton->setEnabled(true);
            ui->chargerVoltageLabel->setEnabled(true);
            ui->chargerVoltageADCCheckBox->setEnabled(true);
            ui->chargerVoltageLcdNumber->setEnabled(true);
            ui->charger35AhRadioButton->setEnabled(true);
            ui->charger120AhRadioButton->setEnabled(true);
            ui->chargingInProgressRadioButton->setEnabled(true);
            ui->chargingCompleteRadioButton->setEnabled(true);
            ui->chargerButton->setEnabled(true);
            ui->chargerConnectedRadioButton->setEnabled(true);
            ui->charger120Ah14v7RadioButton->setEnabled(true);
        }

        ui->leftMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getLeftMotorDutyMax()).c_str());
        ui->rightMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getRightMotorDutyMax()).c_str());
        ui->turretMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getTurretMotorDutyMax()).c_str());
        ui->leftMotorCurDutyBar->setValue(platformControlP1->getLeftMotorDuty());
        ui->rightMotorCurDutyBar->setValue(platformControlP1->getRightMotorDuty());
        ui->turretMotorCurDutyBar->setValue(platformControlP1->getTurretMotorDuty());

        ui->powerSource5VRadioButton->setChecked(platformControlP1->getPower5VOnState());
        ui->leftAccumulatorConnectedRadioButton->setChecked(platformControlP1->getLeftAccumulatorConnected());
        ui->rightAccumulatorConnected->setChecked(platformControlP1->getRightAccumulatorConnected());
        ui->mainAccumulatorRelayRadioButton->setChecked(platformControlP1->getMainAccumulatorRelayOnState());
        ui->leftAccumulatorRelay->setChecked(platformControlP1->getLeftAccumulatorRelayOnState());
        ui->rightAccumulatorRelay->setChecked(platformControlP1->getRightAccumulatorRelayOnState());
        ui->power220VACAvailableRadioButton->setChecked(platformControlP1->getPower220VACAvailable());
        ui->charger35AhRadioButton->setChecked(platformControlP1->getCharger35Ah());
        ui->charger120AhRadioButton->setChecked(platformControlP1->getCharger120Ah());
        ui->chargingInProgressRadioButton->setChecked(platformControlP1->getChargingInProgress());
        ui->chargingCompleteRadioButton->setChecked(platformControlP1->getChargingComplete());
        ui->chargerConnectedRadioButton->setChecked(platformControlP1->getChargerConnected());
        if (ui->chargerVoltageADCCheckBox->isChecked())
        {
            ui->chargerVoltageLcdNumber->display(platformControlP1->getChargerVoltageADC());
        }
        else
        {
            ui->chargerVoltageLcdNumber->display(platformControlP1->getChargerVoltageVolts());
        }
        if (ui->wheelMotorsCurrentADCCheckBox->isChecked())
        {
            ui->leftMotorCurrentLcdNumber->display(platformControlP1->getLeftMotorCurrentADC());
            ui->rightMotorCurrentLcdNumber->display(platformControlP1->getRightMotorCurrentADC());
        }
        else
        {
            ui->leftMotorCurrentLcdNumber->display(platformControlP1->getLeftMotorCurrentAmps());
            ui->rightMotorCurrentLcdNumber->display(platformControlP1->getRightMotorCurrentAmps());
        }
        if (ui->turretMotorCurrentADCCheckBox->isChecked())
        {
            ui->turretMotorCurrentLcdNumber->display(platformControlP1->getTurretMotorCurrentADC());
        }
        else
        {
            ui->turretMotorCurrentLcdNumber->display(platformControlP1->getTurretMotorCurrentAmps());
        }
        if (ui->turretPositionReadCheckBox->isChecked())
        {
            if (ui->turretPositionADCRadioButton->isChecked())
            {
                ui->turretPositionLcdNumber->display(platformControlP1->getTurretPositionADC());
            }
            else if (ui->turretPositionDegRadioButton->isChecked())
            {
                ui->turretPositionLcdNumber->display(platformControlP1->getTurretPositionDeg());
            }
            else if (ui->turretPositionRadRadioButton->isChecked())
            {
                ui->turretPositionLcdNumber->display(platformControlP1->getTurretPositionRad());
            }
        }
        if (platformControlP1->getControlDeviceIsSet())
        {
            switch (platformControlP1->getChargerButtonPressStep())
            {
                    case 0:
                        ui->chargerButtonPressStep0->setText("←");
                        ui->chargerButtonPressStep1->setText("");
                        ui->chargerButtonPressStep2->setText("");
                        ui->chargerButtonPressStep3->setText("");
                    break;
                    case 1:
                        ui->chargerButtonPressStep0->setText("");
                        ui->chargerButtonPressStep1->setText("←");
                        ui->chargerButtonPressStep2->setText("");
                        ui->chargerButtonPressStep3->setText("");
                    break;
                    case 2:
                        ui->chargerButtonPressStep0->setText("");
                        ui->chargerButtonPressStep1->setText("");
                        ui->chargerButtonPressStep2->setText("←");
                        ui->chargerButtonPressStep3->setText("");
                    break;
                    case 3:
                        ui->chargerButtonPressStep0->setText("");
                        ui->chargerButtonPressStep1->setText("");
                        ui->chargerButtonPressStep2->setText("");
                        ui->chargerButtonPressStep3->setText("←");
                    break;
            }
        }
        else
        {
            ui->chargerButtonPressStep0->setText("");
            ui->chargerButtonPressStep1->setText("");
            ui->chargerButtonPressStep2->setText("");
            ui->chargerButtonPressStep3->setText("");
        }
        if (ui->platformControlP1LeftWheelEncoderCheckBox->isChecked())
        {
            ui->platformControlP1LeftWheelEncoderLcdNumber->display(platformControlP1->getLeftWheelEncoder());
        }
        if (ui->platformControlP1RightWheelEncoderCheckBox->isChecked())
        {
            ui->platformControlP1RightWheelEncoderLcdNumber->display(platformControlP1->getRightWheelEncoder());
        }
    }
}

void setPlatfromControlP1AdditionalReadings(QTableWidgetItem *item)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (item->column() == 3)
    {
        switch (item->row())
        {
            case 0:
                platformControlP1->setMainAccumulatorVoltageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 1:
                platformControlP1->setLeftAccumulatorVoltageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 2:
                platformControlP1->setRightAccumulatorVoltageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 3:
                platformControlP1->setMainAccumulatorAmperageTotalRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 4:
                platformControlP1->setPlatformAmperageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 5:
                platformControlP1->setBodyAmperageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 6:
                platformControlP1->setLeftAccumulatorAmperageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 7:
                platformControlP1->setRightAccumulatorAmperageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 8:
                platformControlP1->setChargerVoltageRead((item->checkState() == Qt::Checked) ? true : false);
            break;
        }
    }
    else if (item->column() == 2)
    {
        switch (item->row())
        {
        case 0:
                platformControlP1->setMainAccumulatorVoltageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 1:
                platformControlP1->setLeftAccumulatorVoltageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 2:
                platformControlP1->setRightAccumulatorVoltageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 3:
                platformControlP1->setMainAccumulatorAmperageTotalReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 4:
                platformControlP1->setPlatformAmperageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 5:
                platformControlP1->setBodyAmperageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 6:
                platformControlP1->setLeftAccumulatorAmperageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 7:
                platformControlP1->setRightAccumulatorAmperageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
            case 8:
                platformControlP1->setChargerVoltageReadADCPreset((item->checkState() == Qt::Checked) ? true : false);
            break;
        }
    }
}

#endif // MAINWINDOWUTILS_H
