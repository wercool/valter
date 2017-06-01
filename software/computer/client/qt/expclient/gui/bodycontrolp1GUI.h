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
        if (bodyControlP1->getPowerSource5V5State())
        {
            ui->powerSource5V5StateRadio->setChecked(true);
        }
        else
        {
            ui->powerSource5V5StateRadio->setChecked(false);
        }
        if (bodyControlP1->getWifiPowerState())
        {
            ui->wifiStateRadio->setChecked(true);
        }
        else
        {
            ui->wifiStateRadio->setChecked(false);
        }
        if (bodyControlP1->getLeftArm24VPowerSourceState())
        {
            ui->leftArm24VRadio->setChecked(true);
        }
        else
        {
            ui->leftArm24VRadio->setChecked(false);
        }
        if (bodyControlP1->getRightArm24VPowerSourceState())
        {
            ui->rightArm24VRadio->setChecked(true);
        }
        else
        {
            ui->rightArm24VRadio->setChecked(false);
        }
        if (bodyControlP1->getRightAccumulatorConnectedState())
        {
            ui->rightAccumulatorStateRadio->setChecked(true);
        }
        else
        {
            ui->rightAccumulatorStateRadio->setChecked(false);
        }
        if (bodyControlP1->getLeftAccumulatorConnectedState())
        {
            ui->leftAccumulatorStateRadio->setChecked(true);
        }
        else
        {
            ui->leftAccumulatorStateRadio->setChecked(false);
        }
        if (bodyControlP1->getLeftArm12VPowerSourceState())
        {
            ui->leftArm12VRadio->setChecked(true);
        }
        else
        {
            ui->leftArm12VRadio->setChecked(false);
        }
        if (bodyControlP1->getRightArm12VPowerSourceState())
        {
            ui->rightArm12VRadio->setChecked(true);
        }
        else
        {
            ui->rightArm12VRadio->setChecked(false);
        }
        if (bodyControlP1->getKinect1PowerState())
        {
            ui->kinect1StateRadio->setChecked(true);
        }
        else
        {
            ui->kinect1StateRadio->setChecked(false);
        }
        if (bodyControlP1->getKinect2PowerState())
        {
            ui->kinect2StateRadio->setChecked(true);
        }
        else
        {
            ui->kinect2StateRadio->setChecked(false);
        }
        if (bodyControlP1->getBodyPitchPositionTrack())
        {
            QTableWidgetItem* bodyPitchPositionQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getBodyPitchPositionADC())
            {
                bodyPitchPositionQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getBodyPitchADCPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(0, 2, bodyPitchPositionQWidgetItem);
                ui->bodyPitchPositionLcdNumber->display(bodyControlP1->getBodyPitchADCPosition());
            }
        }
        if (bodyControlP1->getRightArmYawPositionTrack())
        {
            QTableWidgetItem* rightArmYawPositionQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getRightArmYawPositionADC())
            {
                rightArmYawPositionQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getRightArmYawADCPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(2, 2, rightArmYawPositionQWidgetItem);
                ui->rightArmYawPositionLcdNumber->display(bodyControlP1->getRightArmYawADCPosition());
            }
        }
        if (bodyControlP1->getLeftArmYawPositionTrack())
        {
            QTableWidgetItem* leftArmYawPositionQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getLeftArmYawPositionADC())
            {
                leftArmYawPositionQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getLeftArmYawADCPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(4, 2, leftArmYawPositionQWidgetItem);
                ui->leftArmYawPositionLcdNumber->display(bodyControlP1->getLeftArmYawADCPosition());
            }
        }
        if (bodyControlP1->getHeadPitchPositionTrack())
        {
            QTableWidgetItem* headPitchPositionQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getHeadPitchPositionADC())
            {
                headPitchPositionQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getHeadPitchADCPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(6, 2, headPitchPositionQWidgetItem);
                ui->headPitchPositionLcdNumber->display(bodyControlP1->getHeadPitchADCPosition());
            }
            else
            {
                headPitchPositionQWidgetItem->setText(Valter::format_string("%.2f", bodyControlP1->getHeadPitchPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(6, 2, headPitchPositionQWidgetItem);
                ui->headPitchPositionLcdNumber->display(bodyControlP1->getHeadPitchPosition());
            }
            ui->headPitchStepPositionLcdNumber->display(bodyControlP1->getHeadPitchStepPosition());
        }
        if (bodyControlP1->getHeadYawPositionTrack())
        {
            QTableWidgetItem* headYawPositionQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getHeadYawPositionADC())
            {
                headYawPositionQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getHeadYawADCPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(7, 2, headYawPositionQWidgetItem);
                ui->headYawPositionLcdNumber->display(bodyControlP1->getHeadYawADCPosition());
            }
            else
            {
                headYawPositionQWidgetItem->setText(Valter::format_string("%.2f", bodyControlP1->getHeadYawPosition()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(7, 2, headYawPositionQWidgetItem);
                ui->headYawPositionLcdNumber->display(bodyControlP1->getHeadYawPosition());
            }
            ui->headYawStepPositionLcdNumber->display(bodyControlP1->getHeadYawStepPostion());
        }
        if (bodyControlP1->getBodyPitchCurrentTrack())
        {
            QTableWidgetItem* bodyPitchCurrentQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getBodyPitchCurrentADC())
            {
                bodyPitchCurrentQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getBodyPitchADCCurrent()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(1, 2, bodyPitchCurrentQWidgetItem);
                ui->bodyPitchCurrentLcdNumber->display(bodyControlP1->getBodyPitchADCCurrent());
            }
        }
        if (bodyControlP1->getRightArmYawCurrentTrack())
        {
            QTableWidgetItem* rightArmYawCurrentQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getRightArmYawCurrentADC())
            {
                rightArmYawCurrentQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getRightArmYawADCCurrent()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(3, 2, rightArmYawCurrentQWidgetItem);
                ui->rightArmYawCurrentLcdNumber->display(bodyControlP1->getRightArmYawADCCurrent());
            }
        }
        if (bodyControlP1->getLeftArmYawCurrentTrack())
        {
            QTableWidgetItem* leftArmYawCurrentQWidgetItem = new QTableWidgetItem;
            if (bodyControlP1->getLeftArmYawCurrentADC())
            {
                leftArmYawCurrentQWidgetItem->setText(Valter::format_string("%d", bodyControlP1->getLeftArmYawADCCurrent()).c_str());
                ui->bodyControlP1ReadingsTable->setItem(5, 2, leftArmYawCurrentQWidgetItem);
                ui->leftArmYawCurrentLcdNumber->display(bodyControlP1->getLeftArmYawADCCurrent());
            }
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

    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(0, 0))->setCheckState((bodyControlP1->getBodyPitchPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(0, 1))->setCheckState((bodyControlP1->getBodyPitchPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(1, 0))->setCheckState((bodyControlP1->getBodyPitchCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(1, 1))->setCheckState((bodyControlP1->getBodyPitchCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(2, 0))->setCheckState((bodyControlP1->getRightArmYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(2, 1))->setCheckState((bodyControlP1->getRightArmYawPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(3, 0))->setCheckState((bodyControlP1->getRightArmYawCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(3, 1))->setCheckState((bodyControlP1->getRightArmYawCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(4, 0))->setCheckState((bodyControlP1->getLeftArmYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(4, 1))->setCheckState((bodyControlP1->getLeftArmYawPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(5, 0))->setCheckState((bodyControlP1->getLeftArmYawCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(5, 1))->setCheckState((bodyControlP1->getLeftArmYawCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(6, 0))->setCheckState((bodyControlP1->getHeadPitchPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(6, 1))->setCheckState((bodyControlP1->getHeadPitchPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(7, 0))->setCheckState((bodyControlP1->getHeadYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->bodyControlP1ReadingsTable->item(7, 1))->setCheckState((bodyControlP1->getHeadYawPositionADC()) ? Qt::Checked : Qt::Unchecked);

    ui->bodyCameraPositionScroller->setMinimum(bodyControlP1->getBodyCameraUpperPosition());
    ui->bodyCameraPositionScroller->setMaximum(bodyControlP1->getBodyCameraLowerPosition());
    ui->bodyCameraPositionScroller->setValue(bodyControlP1->getBodyCameraCenterPosition());
}

#endif // BODYCONTROLP1GUI_H
