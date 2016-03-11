#ifndef MAINWINDOWUTILS_H
#define MAINWINDOWUTILS_H

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
}

#endif // MAINWINDOWUTILS_H
