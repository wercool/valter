#ifndef ARMCONTROLLEFTGUI_H
#define ARMCONTROLLEFTGUI_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void armControlLeftTabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRedrawGUICheckBox->isChecked()) //ARM-LEFT-CONTROL Tab
    {
        if (!armControlLeft->getLeftForearmMotorStop())
        {
            ui->leftForearmMotorDutyProgress->setValue(armControlLeft->getLeftForearmMotorDuty());
        }
        if (!armControlLeft->getLeftArmMotorStop())
        {
            ui->leftArmMotorDutyProgress->setValue(armControlLeft->getLeftArmMotorDuty());
        }
        if (!armControlLeft->getLeftLimbMotorStop())
        {
            ui->leftLimbMotorDutyProgress->setValue(armControlLeft->getLeftLimbMotorDuty());
        }

        ui->forearmRollMotorStateRadio->setChecked(armControlLeft->getForearmRollMotorState());
    }
}


void loadArmControlLeftDefaults(Ui::MainWindow *ui)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    ui->leftForearmMotorDutyScroller->setMinimum(armControlLeft->getLeftForearmMotorDutyPresetMin());
    ui->leftForearmMotorDutyScroller->setMaximum(armControlLeft->getLeftForearmMotorDutyPresetMax());
    ui->leftForearmMotorDutyScroller->setValue(armControlLeft->getLeftForearmMotorDutyPresetCur());
    ui->leftForearmMotorDutyLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftForearmMotorDutyPresetCur()).c_str());

    ui->leftForearmMotorDecelerationScroller->setValue(armControlLeft->getLeftForearmMotorDeceleration());
    ui->leftForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftForearmMotorDeceleration()).c_str());

    ui->leftForearmAccelerationScroller->setValue(armControlLeft->getLeftForearmMotorAcceleration());
    ui->leftForearmAccelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftForearmMotorAcceleration()).c_str());


    ui->leftArmMotorDutyScroller_2->setMinimum(armControlLeft->getLeftArmMotorDutyPresetMin());
    ui->leftArmMotorDutyScroller_2->setMaximum(armControlLeft->getLeftArmMotorDutyPresetMax());
    ui->leftArmMotorDutyScroller_2->setValue(armControlLeft->getLeftArmMotorDutyPresetCur());
    ui->leftArmMotorDutyLabel_2->setText(Valter::format_string("[%d]", armControlLeft->getLeftArmMotorDutyPresetCur()).c_str());

    ui->leftArmMotorDecelerationScroller_2->setValue(armControlLeft->getLeftArmMotorDeceleration());
    ui->leftArmMotorDecelerationLabel_2->setText(Valter::format_string("[%d]", armControlLeft->getLeftArmMotorDeceleration()).c_str());

    ui->leftArmMotorAccelerationScroller_2->setValue(armControlLeft->getLeftArmMotorAcceleration());
    ui->leftArmMotorAccelerationLabel_2->setText(Valter::format_string("[%d]", armControlLeft->getLeftArmMotorAcceleration()).c_str());


    ui->leftLimbMotorDutyScroller->setMinimum(armControlLeft->getLeftLimbMotorDutyPresetMin());
    ui->leftLimbMotorDutyScroller->setMaximum(armControlLeft->getLeftLimbMotorDutyPresetMax());
    ui->leftLimbMotorDutyScroller->setValue(armControlLeft->getLeftLimbMotorDutyPresetCur());
    ui->leftLimbMotorDutyLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftLimbMotorDutyPresetCur()).c_str());

    ui->leftLimbMotorDecelerationScroller->setValue(armControlLeft->getLeftLimbMotorDeceleration());
    ui->leftLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftLimbMotorDeceleration()).c_str());

    ui->leftLimbMotorAccelerationScroller->setValue(armControlLeft->getLeftLimbMotorAcceleration());
    ui->leftLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftLimbMotorAcceleration()).c_str());

}

//armControlLeftTabRefreshTimer

#endif // ARMCONTROLLEFTGUI_H
