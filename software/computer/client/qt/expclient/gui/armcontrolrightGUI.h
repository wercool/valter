#ifndef ARMCONTROLRIGHTGUI_H
#define ARMCONTROLRIGHTGUI_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void armControlRightTabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

    if (ui->armControlRightRedrawGUICheckBox->isChecked()) //ARM-CONTROL-RIGHT Tab
    {
        if (!armControlRight->getRightForearmMotorStop())
        {
            ui->rightForearmMotorDutyProgress->setValue(armControlRight->getRightForearmMotorDuty());
        }
        if (!armControlRight->getRightArmMotorStop())
        {
            ui->rightArmMotorDutyProgress->setValue(armControlRight->getRightArmMotorDuty());
        }
        if (!armControlRight->getRightLimbMotorStop())
        {
            ui->rightLimbMotorDutyProgress->setValue(armControlRight->getRightLimbMotorDuty());
        }

        ui->rightForearmRollMotorStateRadio->setChecked(armControlRight->getForearmRollMotorState());

        if (armControlRight->getForearmRollMotorState())
        {
            if (!armControlRight->getForearmRollResettingStepPosition())
            {
                ui->rightForearmRollStepPositionLcdNumber->display(armControlRight->getForearmRollStepPosition());
                if (armControlRight->getForearmRollCWLimit())
                {
                    ui->rightForearmLimitLabel->setText("⟳");
                }
                else if (armControlRight->getForearmRollCCWLimit())
                {
                    ui->rightForearmLimitLabel->setText("⟲");
                }
                else
                {
                    ui->rightForearmLimitLabel->setText("");
                }
            }
            else
            {
                ui->rightForearmLimitLabel->setText("0 <-");
            }
        }

        //right arm readings
        if (armControlRight->getForearmPositionTrack())
        {
            QTableWidgetItem* forearmPositionQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getForearmPositionADC())
            {
                forearmPositionQWidgetItem->setText(Valter::format_string("%d", armControlRight->getForearmADCPosition()).c_str());
                ui->rightArmReadingsTable->setItem(0, 2, forearmPositionQWidgetItem);
                ui->rightForearmPositionLcdNumber->display(armControlRight->getForearmADCPosition());
            }
        }
        if (armControlRight->getArmPositionTrack())
        {
            QTableWidgetItem* armPositionQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getArmPositionADC())
            {
                armPositionQWidgetItem->setText(Valter::format_string("%d", armControlRight->getArmADCPosition()).c_str());
                ui->rightArmReadingsTable->setItem(1, 2, armPositionQWidgetItem);
                ui->rightArmPositionLcdNumber->display(armControlRight->getArmADCPosition());
            }
        }
        if (armControlRight->getLimbPositionTrack())
        {
            QTableWidgetItem* limbPositionQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getLimbPositionADC())
            {
                limbPositionQWidgetItem->setText(Valter::format_string("%d", armControlRight->getLimbADCPosition()).c_str());
                ui->rightArmReadingsTable->setItem(2, 2, limbPositionQWidgetItem);
                ui->rightLimbPositionLcdNumber->display(armControlRight->getLimbADCPosition());
            }
        }
        if (armControlRight->getForearmMotorCurrentTrack())
        {
            QTableWidgetItem* forearmMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getForearmMotorCurrentADC())
            {
                forearmMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlRight->getForearmMotorADCCurrent()).c_str());
                ui->rightArmReadingsTable->setItem(3, 2, forearmMotorCurrentQWidgetItem);
                ui->rightForearmCurrentLcdNumber->display(armControlRight->getForearmMotorADCCurrent());
            }
        }
        if (armControlRight->getArmMotorCurrentTrack())
        {
            QTableWidgetItem* armMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getArmMotorCurrentADC())
            {
                armMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlRight->getArmMotorADCCurrent()).c_str());
                ui->rightArmReadingsTable->setItem(4, 2, armMotorCurrentQWidgetItem);
                ui->rightArmCurrentLcdNumber->display(armControlRight->getArmMotorADCCurrent());
            }
        }
        if (armControlRight->getLimbMotorCurrentTrack())
        {
            QTableWidgetItem* limbMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getLimbMotorCurrentADC())
            {
                limbMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlRight->getLimbMotorADCCurrent()).c_str());
                ui->rightArmReadingsTable->setItem(5, 2, limbMotorCurrentQWidgetItem);
                ui->rightLimbCurrentLcdNumber->display(armControlRight->getLimbMotorADCCurrent());
            }
        }
        if (armControlRight->getHandYawPositionTrack())
        {
            QTableWidgetItem* handYawPositionQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getHandYawPositionADC())
            {
                handYawPositionQWidgetItem->setText(Valter::format_string("%d", armControlRight->getHandYawADCPosition()).c_str());
                ui->rightArmReadingsTable->setItem(6, 2, handYawPositionQWidgetItem);
                ui->rightHandYawPositionLcdNumber->display(armControlRight->getHandYawADCPosition());
            }
        }
        if (armControlRight->getHandPitchPositionTrack())
        {
            QTableWidgetItem* handPitchPositionQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getHandPitchPositionADC())
            {
                handPitchPositionQWidgetItem->setText(Valter::format_string("%d", armControlRight->getHandPitchADCPosition()).c_str());
                ui->rightArmReadingsTable->setItem(7, 2, handPitchPositionQWidgetItem);
                ui->rightHandPitchPositionLcdNumber->display(armControlRight->getHandPitchADCPosition());
            }
        }
        if (armControlRight->getForearmYawPositionTrack())
        {
            QTableWidgetItem* forearmYawPositionQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getForearmYawPositionADC())
            {
                forearmYawPositionQWidgetItem->setText(Valter::format_string("%d", armControlRight->getForearmYawADCPosition()).c_str());
                ui->rightArmReadingsTable->setItem(8, 2, forearmYawPositionQWidgetItem);
                ui->rightForearmYawPositionLcdNumber->display(armControlRight->getForearmYawADCPosition());
            }
        }
        if (armControlRight->getForearmYawMotorCurrentTrack())
        {
            QTableWidgetItem* forearmYawMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getForearmYawMotorCurrentADC())
            {
                forearmYawMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlRight->getForearmYawMotorADCCurrent()).c_str());
                ui->rightArmReadingsTable->setItem(9, 2, forearmYawMotorCurrentQWidgetItem);
                ui->rightForearmYawCurrentLcdNumber->display(armControlRight->getForearmYawMotorADCCurrent());
            }
        }
        if (armControlRight->getHandYawMotorCurrentTrack())
        {
            QTableWidgetItem* handYawMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getHandYawMotorCurrentADC())
            {
                handYawMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlRight->getHandYawMotorADCCurrent()).c_str());
                ui->rightArmReadingsTable->setItem(10, 2, handYawMotorCurrentQWidgetItem);
                ui->rightHandYawCurrentLcdNumber->display(armControlRight->getHandYawMotorADCCurrent());
            }
        }
        if (armControlRight->getHandPitchMotorCurrentTrack())
        {
            QTableWidgetItem* handPitchMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getHandPitchMotorCurrentADC())
            {
                handPitchMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlRight->getHandPitchMotorADCCurrent()).c_str());
                ui->rightArmReadingsTable->setItem(11, 2, handPitchMotorCurrentQWidgetItem);
                ui->rightHandPitchCurrentLcdNumber->display(armControlRight->getHandPitchMotorADCCurrent());
            }
        }

        //track hand finger sensors
        for (int idx = 0; idx < 13; idx++)
        {
            QTableWidgetItem* handFingerSensorReadingQWidgetItem = new QTableWidgetItem;
            if (armControlRight->getHandSensorsTrack(idx))
            {
                handFingerSensorReadingQWidgetItem->setText(Valter::format_string("%d", armControlRight->getHandSensorsADCForce(idx)).c_str());
                ui->rightHandSensorsTable->setItem(idx, 1, handFingerSensorReadingQWidgetItem);
            }
        }

        ui->rightPalmJambLineEdit->setText(Valter::format_string("%d", armControlRight->getPalmJambReading()).c_str());
        ui->rightFinger5TipLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger5TipReading()).c_str());
        ui->rightFinger5PhalanxLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger5PhalanxReading()).c_str());
        ui->rightFinger4TipLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger4TipReading()).c_str());
        ui->rightFinger4PhalanxLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger4PhalanxReading()).c_str());
        ui->rightFinger3PhalanxLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger3PhalanxReading()).c_str());
        ui->rightFinger2PhalanxLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger2PhalanxReading()).c_str());
        ui->rightFinger1TipLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger1TipReading()).c_str());
        ui->rightFinger1PhalanxLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger1PhalanxReading()).c_str());
        ui->rightFinger0PhalanxLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger0PhalanxReading()).c_str());
        ui->rightFinger0TipLineEdit->setText(Valter::format_string("%d", armControlRight->getFinger0TipReading()).c_str());
        ui->rightPalmUpperLineEdit->setText(Valter::format_string("%d", armControlRight->getPalmUpperReading()).c_str());
        ui->rightPalmLowerLineEdit->setText(Valter::format_string("%d", armControlRight->getPalmLowerReading()).c_str());

        ui->rightPalmJambLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getPalmJambReadingRelative()).c_str());
        ui->rightFinger5TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger5TipReadingRelative()).c_str());
        ui->rightFinger5PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger5PhalanxReadingRelative()).c_str());
        ui->rightFinger4TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger4TipReadingRelative()).c_str());
        ui->rightFinger4PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger4PhalanxReadingRelative()).c_str());
        ui->rightFinger3PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger3PhalanxReadingRelative()).c_str());
        ui->rightFinger2PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger2PhalanxReadingRelative()).c_str());
        ui->rightFinger1TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger1TipReadingRelative()).c_str());
        ui->rightFinger1PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger1PhalanxReadingRelative()).c_str());
        ui->rightFinger0PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger0PhalanxReadingRelative()).c_str());
        ui->rightFinger0TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getFinger0TipReadingRelative()).c_str());
        ui->rightPalmUpperLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getPalmUpperReadingRelative()).c_str());
        ui->rightPalmLowerLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlRight->getPalmLowerReadingRelative()).c_str());
    }
}

void loadArmControlRightDefaults(Ui::MainWindow *ui)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();

    armControlRight->setDefaultsLoading(true);

    ui->rightForearmMotorDutyScroller->setMinimum(armControlRight->getRightForearmMotorDutyPresetMin());
    ui->rightForearmMotorDutyScroller->setMaximum(armControlRight->getRightForearmMotorDutyPresetMax());
    ui->rightForearmMotorDutyScroller->setValue(armControlRight->getRightForearmMotorDutyPresetCur());
    ui->rightForearmMotorDutyLabel->setText(Valter::format_string("[%d]", armControlRight->getRightForearmMotorDutyPresetCur()).c_str());

    ui->rightForearmMotorDecelerationScroller->setValue(armControlRight->getRightForearmMotorDeceleration());
    ui->rightForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlRight->getRightForearmMotorDeceleration()).c_str());

    ui->rightForearmAccelerationScroller->setValue(armControlRight->getRightForearmMotorAcceleration());
    ui->rightForearmAccelerationLabel->setText(Valter::format_string("[%d]", armControlRight->getRightForearmMotorAcceleration()).c_str());

    ui->armControlRightArmMotorDutyScroller->setMinimum(armControlRight->getRightArmMotorDutyPresetMin());
    ui->armControlRightArmMotorDutyScroller->setMaximum(armControlRight->getRightArmMotorDutyPresetMax());
    ui->armControlRightArmMotorDutyScroller->setValue(armControlRight->getRightArmMotorDutyPresetCur());
    ui->armControlRightArmMotorDutyLabel->setText(Valter::format_string("[%d]", armControlRight->getRightArmMotorDutyPresetCur()).c_str());

    ui->armControlRightArmMotorDecelerationScroller->setValue(armControlRight->getRightArmMotorDeceleration());
    ui->armControlRightArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlRight->getRightArmMotorDeceleration()).c_str());

    ui->armControlRightMotorAccelerationScroller->setValue(armControlRight->getRightArmMotorAcceleration());
    ui->armControlRightArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", armControlRight->getRightArmMotorAcceleration()).c_str());

    ui->rightLimbMotorDutyScroller->setMinimum(armControlRight->getRightLimbMotorDutyPresetMin());
    ui->rightLimbMotorDutyScroller->setMaximum(armControlRight->getRightLimbMotorDutyPresetMax());
    ui->rightLimbMotorDutyScroller->setValue(armControlRight->getRightLimbMotorDutyPresetCur());
    ui->rightLimbMotorDutyLabel->setText(Valter::format_string("[%d]", armControlRight->getRightLimbMotorDutyPresetCur()).c_str());

    ui->rightLimbMotorDecelerationScroller->setValue(armControlRight->getRightLimbMotorDeceleration());
    ui->rightLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlRight->getRightLimbMotorDeceleration()).c_str());

    ui->rightLimbMotorAccelerationScroller->setValue(armControlRight->getRightLimbMotorAcceleration());
    ui->rightLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", armControlRight->getRightLimbMotorAcceleration()).c_str());

    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(0, 0))->setCheckState((armControlRight->getForearmPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(0, 1))->setCheckState((armControlRight->getForearmPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(1, 0))->setCheckState((armControlRight->getArmPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(1, 1))->setCheckState((armControlRight->getArmPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(2, 0))->setCheckState((armControlRight->getLimbPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(2, 1))->setCheckState((armControlRight->getLimbPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(3, 0))->setCheckState((armControlRight->getForearmMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(3, 1))->setCheckState((armControlRight->getForearmMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(4, 0))->setCheckState((armControlRight->getArmMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(4, 1))->setCheckState((armControlRight->getArmMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(5, 0))->setCheckState((armControlRight->getLimbMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(5, 1))->setCheckState((armControlRight->getLimbMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(6, 0))->setCheckState((armControlRight->getHandYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(6, 1))->setCheckState((armControlRight->getHandYawPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(7, 0))->setCheckState((armControlRight->getHandPitchPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(7, 1))->setCheckState((armControlRight->getHandPitchPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(8, 0))->setCheckState((armControlRight->getForearmYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(8, 1))->setCheckState((armControlRight->getForearmYawPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(9, 0))->setCheckState((armControlRight->getForearmYawMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(9, 1))->setCheckState((armControlRight->getForearmYawMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(10, 0))->setCheckState((armControlRight->getHandYawMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(10, 1))->setCheckState((armControlRight->getHandYawMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(11, 0))->setCheckState((armControlRight->getHandPitchMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->rightArmReadingsTable->item(11, 1))->setCheckState((armControlRight->getHandPitchMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);

    for (int idx = 0; idx < 13; idx++)
    {
        ((QTableWidgetItem*)ui->rightHandSensorsTable->item(idx, 0))->setCheckState((armControlRight->getHandSensorsTrack(idx)) ? Qt::Checked : Qt::Unchecked);
    }

    if (armControlRight->getFingerInitialPosition(0) > armControlRight->getFingerGraspedPosition(0))
    {
        ui->armControlRightFinger0PositionScoller->setMinimum(armControlRight->getFingerGraspedPosition(0));
        ui->armControlRightFinger0PositionScoller->setMaximum(armControlRight->getFingerInitialPosition(0));
    }
    else
    {
        ui->armControlRightFinger0PositionScoller->setMaximum(armControlRight->getFingerGraspedPosition(0));
        ui->armControlRightFinger0PositionScoller->setMinimum(armControlRight->getFingerInitialPosition(0));
    }
    ui->armControlRightFinger0PositionScoller->setValue(armControlRight->getFingerInitialPosition(0));

    if (armControlRight->getFingerInitialPosition(1) > armControlRight->getFingerGraspedPosition(1))
    {
        ui->armControlRightFinger1PositionScoller->setMinimum(armControlRight->getFingerGraspedPosition(1));
        ui->armControlRightFinger1PositionScoller->setMaximum(armControlRight->getFingerInitialPosition(1));
    }
    else
    {
        ui->armControlRightFinger1PositionScoller->setMaximum(armControlRight->getFingerGraspedPosition(1));
        ui->armControlRightFinger1PositionScoller->setMinimum(armControlRight->getFingerInitialPosition(1));
    }
    ui->armControlRightFinger1PositionScoller->setValue(armControlRight->getFingerInitialPosition(1));
    ui->armControlRightFinger1PositionScoller->setInvertedAppearance(true);

    if (armControlRight->getFingerInitialPosition(2) > armControlRight->getFingerGraspedPosition(2))
    {
        ui->armControlRightFinger2PositionScoller->setMinimum(armControlRight->getFingerGraspedPosition(2));
        ui->armControlRightFinger2PositionScoller->setMaximum(armControlRight->getFingerInitialPosition(2));
    }
    else
    {
        ui->armControlRightFinger2PositionScoller->setMaximum(armControlRight->getFingerGraspedPosition(2));
        ui->armControlRightFinger2PositionScoller->setMinimum(armControlRight->getFingerInitialPosition(2));
    }
    ui->armControlRightFinger2PositionScoller->setValue(armControlRight->getFingerInitialPosition(2));
    ui->armControlRightFinger2PositionScoller->setInvertedAppearance(true);

    if (armControlRight->getFingerInitialPosition(3) > armControlRight->getFingerGraspedPosition(3))
    {
        ui->armControlRightFinger3PositionScoller->setMinimum(armControlRight->getFingerGraspedPosition(3));
        ui->armControlRightFinger3PositionScoller->setMaximum(armControlRight->getFingerInitialPosition(3));
    }
    else
    {
        ui->armControlRightFinger3PositionScoller->setMaximum(armControlRight->getFingerGraspedPosition(3));
        ui->armControlRightFinger3PositionScoller->setMinimum(armControlRight->getFingerInitialPosition(3));
    }

    if (armControlRight->getFingerInitialPosition(4) > armControlRight->getFingerGraspedPosition(4))
    {
        ui->armControlRightFinger4PositionScoller->setMinimum(armControlRight->getFingerGraspedPosition(4));
        ui->armControlRightFinger4PositionScoller->setMaximum(armControlRight->getFingerInitialPosition(4));
    }
    else
    {
        ui->armControlRightFinger4PositionScoller->setMaximum(armControlRight->getFingerGraspedPosition(4));
        ui->armControlRightFinger4PositionScoller->setMinimum(armControlRight->getFingerInitialPosition(4));
    }

    if (armControlRight->getFingerInitialPosition(5) > armControlRight->getFingerGraspedPosition(5))
    {
        ui->armControlRightFinger5PositionScoller->setMinimum(armControlRight->getFingerGraspedPosition(5));
        ui->armControlRightFinger5PositionScoller->setMaximum(armControlRight->getFingerInitialPosition(5));
    }
    else
    {
        ui->armControlRightFinger5PositionScoller->setMaximum(armControlRight->getFingerGraspedPosition(5));
        ui->armControlRightFinger5PositionScoller->setMinimum(armControlRight->getFingerInitialPosition(5));
    }
    ui->armControlRightFinger5PositionScoller->setValue(armControlRight->getFingerInitialPosition(5));
    ui->armControlRightFinger5PositionScoller->setInvertedAppearance(true);

    armControlRight->setDefaultsLoading(false);
}

void setRightArmReadingsPresets(QTableWidgetItem *item)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    switch (item->row())
    {
        case 0://forearmPosition
            if (item->column() == 0)
            {
                armControlRight->setForearmPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setForearmPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 1://armPosition
            if (item->column() == 0)
            {
                armControlRight->setArmPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setArmPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 2://limbPosition
            if (item->column() == 0)
            {
                armControlRight->setLimbPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setLimbPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 3://forearmMotorCurrent
            if (item->column() == 0)
            {
                armControlRight->setForearmMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setForearmMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 4://armMotorCurrent
            if (item->column() == 0)
            {
                armControlRight->setArmMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setArmMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 5://limbMotorCurrent
            if (item->column() == 0)
            {
                armControlRight->setLimbMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setLimbMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 6://handYawPosition
            if (item->column() == 0)
            {
                armControlRight->setHandYawPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setHandYawPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 7://handPitchPosition
            if (item->column() == 0)
            {
                armControlRight->setHandPitchPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setHandPitchPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 8://forearmYawPosition
            if (item->column() == 0)
            {
                armControlRight->setForearmYawPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setForearmYawPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 9://forearmYawMotorCurrent
            if (item->column() == 0)
            {
                armControlRight->setForearmYawMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setForearmYawMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 10://handYawMotorCurrent
            if (item->column() == 0)
            {
                armControlRight->setHandYawMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setHandYawMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 11://handPitchMotorCurrent
            if (item->column() == 0)
            {
                armControlRight->setHandPitchMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlRight->setHandPitchMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
    }
}

void setRightHandForceSensorsReadingsPresets(QTableWidgetItem *item)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (item->column() == 0)
    {
        armControlRight->setHandSensorsTrack(item->row(), ((item->checkState() == Qt::Checked) ? true : false));
    }
}

#endif // ARMCONTROLRIGHTGUI_H
