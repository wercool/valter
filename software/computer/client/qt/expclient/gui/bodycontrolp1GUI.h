#ifndef BODYCONTROLP1GUI_H
#define BODYCONTROLP1GUI_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void bodyControlP1TabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1CheckBox->isChecked()) //BODY-CONROL-P1 Tab
    {
        if (bodyControlP1->getHead24VState())
        {
            ui->head24VPowerSourceStateRadioButton->setChecked(true);
        }
        else
        {
            ui->head24VPowerSourceStateRadioButton->setChecked(false);
        }
        if (bodyControlP1->getHeadYawMotorState())
        {
            ui->headYawMotorStateRadioButton->setChecked(true);
        }
        else
        {
            ui->headYawMotorStateRadioButton->setChecked(false);
        }
        if (bodyControlP1->getHeadPitchMotorState())
        {
            ui->headPitchMotorStateRadioButton->setChecked(true);
        }
        else
        {
            ui->headPitchMotorStateRadioButton->setChecked(false);
        }
    }
}

void loadBodyControlP1Defaults(Ui::MainWindow *ui)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();

    ui->bodyPitchMotorDutyScroller->setMinimum(bodyControlP1->getBodyPitchMotorDutyPresetMin());
    ui->bodyPitchMotorDutyScroller->setMaximum(bodyControlP1->getBodyPitchMotorDutyPresetMax());
    ui->bodyPitchMotorDutyScroller->setValue(bodyControlP1->getBodyPitchMotorDutyPresetCur());
    ui->bodyPitchMotorDutyLabel->setText(Valter::format_string("[%d]", bodyControlP1->getBodyPitchMotorDutyPresetCur()).c_str());

    ui->bodyPitchMotorDecelerationScroller->setValue(bodyControlP1->getBodyPitchMotorDeceleration());
    ui->bodyPitchMotorDecelerationLabel->setText(Valter::format_string("[%d]", bodyControlP1->getBodyPitchMotorDeceleration()).c_str());

    ui->bodyPitchMotorAccelerationScroller->setValue(bodyControlP1->getBodyPitchMotorAcceleration());
    ui->bodyPitchMotorAccelerationLabel->setText(Valter::format_string("[%d]", bodyControlP1->getBodyPitchMotorAcceleration()).c_str());

    ui->rightArmMotorDutyScroller->setMinimum(bodyControlP1->getRightArmYawMotorDutyPresetMin());
    ui->rightArmMotorDutyScroller->setMaximum(bodyControlP1->getRightArmYawMotorDutyPresetMax());
    ui->rightArmMotorDutyScroller->setValue(bodyControlP1->getRightArmYawMotorDutyPresetCur());
    ui->rightArmMotorDutyLabel->setText(Valter::format_string("[%d]", bodyControlP1->getRightArmYawMotorDutyPresetCur()).c_str());

    ui->rightArmMotorDecelerationScroller->setValue(bodyControlP1->getRightArmYawMotorDeceleration());
    ui->rightArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", bodyControlP1->getRightArmYawMotorDeceleration()).c_str());

    ui->rightArmMotorAccelerationScroller->setValue(bodyControlP1->getRightArmYawMotorAcceleration());
    ui->rightArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", bodyControlP1->getRightArmYawMotorAcceleration()).c_str());

    ui->leftArmMotorDutyScroller->setMinimum(bodyControlP1->getLeftArmYawMotorDutyPresetMin());
    ui->leftArmMotorDutyScroller->setMaximum(bodyControlP1->getLeftArmYawMotorDutyPresetMax());
    ui->leftArmMotorDutyScroller->setValue(bodyControlP1->getLeftArmYawMotorDutyPresetCur());
    ui->leftArmMotorDutyLabel->setText(Valter::format_string("[%d]", bodyControlP1->getLeftArmYawMotorDutyPresetCur()).c_str());

    ui->leftArmMotorDecelerationScroller->setValue(bodyControlP1->getLeftArmYawMotorDeceleration());
    ui->leftArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", bodyControlP1->getLeftArmYawMotorDeceleration()).c_str());

    ui->leftArmMotorAccelerationScroller->setValue(bodyControlP1->getLeftArmYawMotorAcceleration());
    ui->leftArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", bodyControlP1->getLeftArmYawMotorAcceleration()).c_str());
}

#endif // BODYCONTROLP1GUI_H
