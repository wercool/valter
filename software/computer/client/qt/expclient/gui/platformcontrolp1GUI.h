#ifndef PLATFORMCONTROLP1GUI_H
#define PLATFORMCONTROLP1GUI_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

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

    //platformControlP1additionalReadingsTrackingDelay
    ui->platformControlP1additionalReadingsTrackingDelay->setMinimum(platformControlP1->getAdditionalReadingsDelayPresetMin());
    ui->platformControlP1additionalReadingsTrackingDelay->setMaximum(platformControlP1->getAdditionalReadingsDelayPresetMax());
    ui->platformControlP1additionalReadingsTrackingDelay->setValue(platformControlP1->getAdditionalReadingsDelayCur());
    ui->platformControlP1additionalReadingsTrackingDelayLabel->setText(Valter::format_string("[%d]", platformControlP1->getAdditionalReadingsDelayCur()).c_str());
}

void platformControlP1TabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    if (ui->platformControlP1RedrawGUICheckBox->isChecked()) //PLATFROM-CONROL-P1 Tab
    {
        PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
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


        if (platformControlP1->getMainAccumulatorVoltageRead())
        {
            //main accumulator voltage
            QTableWidgetItem* mainAccumulatorVoltageReading = new QTableWidgetItem;
            if (platformControlP1->getMainAccumulatorVoltageReadADCPreset())
            {
                mainAccumulatorVoltageReading->setText(Valter::format_string("%d", platformControlP1->getMainAccumulatorVoltageADC()).c_str());
            }
            else
            {
                mainAccumulatorVoltageReading->setText(Valter::format_string("%.2f V", platformControlP1->getMainAccumulatorVoltageVolts()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(0, 1, mainAccumulatorVoltageReading);
        }
        if (platformControlP1->getLeftAccumulatorVoltageRead())
        {
            //left accumulator voltage
            QTableWidgetItem *leftAccumulatorVoltageReading = new QTableWidgetItem;
            if (platformControlP1->getLeftAccumulatorVoltageReadADCPreset())
            {
                leftAccumulatorVoltageReading->setText(Valter::format_string("%d", platformControlP1->getLeftAccumulatorVoltageADC()).c_str());
            }
            else
            {
                leftAccumulatorVoltageReading->setText(Valter::format_string("%.2f V", platformControlP1->getLeftAccumulatorVoltageVolts()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(1, 1, leftAccumulatorVoltageReading);
        }

        if (platformControlP1->getRightAccumulatorVoltageRead())
        {
            //right accumulator voltage
            QTableWidgetItem* rightAccumulatorVoltageReading = new QTableWidgetItem;
            if (platformControlP1->getRightAccumulatorVoltageReadADCPreset())
            {
                rightAccumulatorVoltageReading->setText(Valter::format_string("%d", platformControlP1->getRightAccumulatorVoltageADC()).c_str());
            }
            else
            {
                rightAccumulatorVoltageReading->setText(Valter::format_string("%.2f V", platformControlP1->getRightAccumulatorVoltageVolts()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(2, 1, rightAccumulatorVoltageReading);
        }
        if (platformControlP1->getMainAccumulatorAmperageTotalRead())
        {
            //main accumulator amperage total
            QTableWidgetItem* mainAccumulatorAmperageTotalReading = new QTableWidgetItem;
            if (platformControlP1->getMainAccumulatorAmperageTotalReadADCPreset())
            {
                mainAccumulatorAmperageTotalReading->setText(Valter::format_string("%d", platformControlP1->getMainAccumulatorAmperageTotalADC()).c_str());
            }
            else
            {
                mainAccumulatorAmperageTotalReading->setText(Valter::format_string("%.2f A", platformControlP1->getMainAccumulatorAmperageTotalAmps()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(3, 1, mainAccumulatorAmperageTotalReading);
        }
        if (platformControlP1->getPlatformAmperageRead())
        {
            //main accumulator amperage bottom (platform)
            QTableWidgetItem* platformAmperageReading = new QTableWidgetItem;
            if (platformControlP1->getPlatformAmperageReadADCPreset())
            {
                platformAmperageReading->setText(Valter::format_string("%d", platformControlP1->getPlatformAmperageADC()).c_str());
            }
            else
            {
                platformAmperageReading->setText(Valter::format_string("%.2f A", platformControlP1->getPlatformAmperageAmps()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(4, 1, platformAmperageReading);
        }
        if (platformControlP1->getBodyAmperageRead())
        {
            //main accumulator amperage top (body)
            QTableWidgetItem* bodyAmperageReading = new QTableWidgetItem;
            if (platformControlP1->getBodyAmperageReadADCPreset())
            {
                bodyAmperageReading->setText(Valter::format_string("%d", platformControlP1->getBodyAmperageADC()).c_str());
            }
            else
            {
                bodyAmperageReading->setText(Valter::format_string("%.2f A", platformControlP1->getBodyAmperageAmps()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(5, 1, bodyAmperageReading);
        }
        if (platformControlP1->getLeftAccumulatorAmperageRead())
        {
            //left accumulator amperage
            QTableWidgetItem* leftAccumulatorAmperageReading = new QTableWidgetItem;
            if (platformControlP1->getLeftAccumulatorAmperageReadADCPreset())
            {
                leftAccumulatorAmperageReading->setText(Valter::format_string("%d", platformControlP1->getLeftAccumulatorAmperageADC()).c_str());
            }
            else
            {
                leftAccumulatorAmperageReading->setText(Valter::format_string("%.2f A", platformControlP1->getLeftAccumulatorAmperageAmps()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(6, 1, leftAccumulatorAmperageReading);
        }
        if (platformControlP1->getRightAccumulatorAmperageRead())
        {
            //right accumulator amperage
            QTableWidgetItem* rightAccumulatorAmperageReading = new QTableWidgetItem;
            if (platformControlP1->getRightAccumulatorAmperageReadADCPreset())
            {
                rightAccumulatorAmperageReading->setText(Valter::format_string("%d", platformControlP1->getRightAccumulatorAmperageADC()).c_str());
            }
            else
            {
                rightAccumulatorAmperageReading->setText(Valter::format_string("%.2f A", platformControlP1->getRightAccumulatorAmperageAmps()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(7, 1, rightAccumulatorAmperageReading);
        }
        if (platformControlP1->getChargerVoltageRead())
        {
            //charger connected (charger voltage)
            QTableWidgetItem* chargerVoltageReading = new QTableWidgetItem;
            if (platformControlP1->getChargerVoltageReadADCPreset())
            {
                chargerVoltageReading->setText(Valter::format_string("%d", platformControlP1->getChargerVoltageADC()).c_str());
            }
            else
            {
                chargerVoltageReading->setText(Valter::format_string("%.2f V", platformControlP1->getChargerVoltageVolts()).c_str());
            }
            ui->platformControlP1ReadingsTable->setItem(8, 1, chargerVoltageReading);
        }
    }
}

void setPlatfromControlP1AdditionalReadings(QTableWidgetItem *item)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
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
