#ifndef ARMCONTROLLEFTGUI_H
#define ARMCONTROLLEFTGUI_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void armControlLeftTabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRedrawGUICheckBox->isChecked()) //ARM-CONTROL-LEFT Tab
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

        ui->leftForearmRollMotorStateRadio->setChecked(armControlLeft->getForearmRollMotorState());

        if (armControlLeft->getForearmRollMotorState())
        {
            if (!armControlLeft->getForearmRollResettingStepPosition())
            {
                ui->leftForearmRollStepPositionLcdNumber->display(armControlLeft->getForearmRollStepPosition());
                if (armControlLeft->getForearmRollCWLimit())
                {
                    ui->leftForearmLimitLabel->setText("⟳");
                }
                else if (armControlLeft->getForearmRollCCWLimit())
                {
                    ui->leftForearmLimitLabel->setText("⟲");
                }
                else
                {
                    ui->leftForearmLimitLabel->setText("");
                }
            }
            else
            {
                ui->leftForearmLimitLabel->setText("0 <-");
            }
        }

        //left arm readings
        if (armControlLeft->getForearmPositionTrack())
        {
            QTableWidgetItem* forearmPositionQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getForearmPositionADC())
            {
                forearmPositionQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getForearmADCPosition()).c_str());
                ui->leftArmReadingsTable->setItem(0, 2, forearmPositionQWidgetItem);
                ui->leftForearmPositionLcdNumber->display(armControlLeft->getForearmADCPosition());
            }
        }
        if (armControlLeft->getArmPositionTrack())
        {
            QTableWidgetItem* armPositionQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getArmPositionADC())
            {
                armPositionQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getArmADCPosition()).c_str());
                ui->leftArmReadingsTable->setItem(1, 2, armPositionQWidgetItem);
                ui->leftArmPositionLcdNumber->display(armControlLeft->getArmADCPosition());
            }
        }
        if (armControlLeft->getLimbPositionTrack())
        {
            QTableWidgetItem* limbPositionQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getLimbPositionADC())
            {
                limbPositionQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getLimbADCPosition()).c_str());
                ui->leftArmReadingsTable->setItem(2, 2, limbPositionQWidgetItem);
                ui->leftLimbPositionLcdNumber->display(armControlLeft->getLimbADCPosition());
            }
        }
        if (armControlLeft->getForearmMotorCurrentTrack())
        {
            QTableWidgetItem* forearmMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getForearmMotorCurrentADC())
            {
                forearmMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getForearmMotorADCCurrent()).c_str());
                ui->leftArmReadingsTable->setItem(3, 2, forearmMotorCurrentQWidgetItem);
                ui->leftForearmCurrentLcdNumber->display(armControlLeft->getForearmMotorADCCurrent());
            }
        }
        if (armControlLeft->getArmMotorCurrentTrack())
        {
            QTableWidgetItem* armMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getArmMotorCurrentADC())
            {
                armMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getArmMotorADCCurrent()).c_str());
                ui->leftArmReadingsTable->setItem(4, 2, armMotorCurrentQWidgetItem);
                ui->leftArmCurrentLcdNumber->display(armControlLeft->getArmMotorADCCurrent());
            }
        }
        if (armControlLeft->getLimbMotorCurrentTrack())
        {
            QTableWidgetItem* limbMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getLimbMotorCurrentADC())
            {
                limbMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getLimbMotorADCCurrent()).c_str());
                ui->leftArmReadingsTable->setItem(5, 2, limbMotorCurrentQWidgetItem);
                ui->leftLimbCurrentLcdNumber->display(armControlLeft->getLimbMotorADCCurrent());
            }
        }
        if (armControlLeft->getHandYawPositionTrack())
        {
            QTableWidgetItem* handYawPositionQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getHandYawPositionADC())
            {
                handYawPositionQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getHandYawADCPosition()).c_str());
                ui->leftArmReadingsTable->setItem(6, 2, handYawPositionQWidgetItem);
                ui->leftHandYawPositionLcdNumber->display(armControlLeft->getHandYawADCPosition());
            }
        }
        if (armControlLeft->getHandPitchPositionTrack())
        {
            QTableWidgetItem* handPitchPositionQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getHandPitchPositionADC())
            {
                handPitchPositionQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getHandPitchADCPosition()).c_str());
                ui->leftArmReadingsTable->setItem(7, 2, handPitchPositionQWidgetItem);
                ui->leftHandPitchPositionLcdNumber->display(armControlLeft->getHandPitchADCPosition());
            }
        }
        if (armControlLeft->getForearmYawPositionTrack())
        {
            QTableWidgetItem* forearmYawPositionQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getForearmYawPositionADC())
            {
                forearmYawPositionQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getForearmYawADCPosition()).c_str());
                ui->leftArmReadingsTable->setItem(8, 2, forearmYawPositionQWidgetItem);
                ui->leftForearmYawPositionLcdNumber->display(armControlLeft->getForearmYawADCPosition());
            }
        }
        if (armControlLeft->getForearmYawMotorCurrentTrack())
        {
            QTableWidgetItem* forearmYawMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getForearmYawMotorCurrentADC())
            {
                forearmYawMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getForearmYawMotorADCCurrent()).c_str());
                ui->leftArmReadingsTable->setItem(9, 2, forearmYawMotorCurrentQWidgetItem);
                ui->leftForearmYawCurrentLcdNumber->display(armControlLeft->getForearmYawMotorADCCurrent());
            }
        }
        if (armControlLeft->getHandYawMotorCurrentTrack())
        {
            QTableWidgetItem* handYawMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getHandYawMotorCurrentADC())
            {
                handYawMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getHandYawMotorADCCurrent()).c_str());
                ui->leftArmReadingsTable->setItem(10, 2, handYawMotorCurrentQWidgetItem);
                ui->leftHandYawCurrentLcdNumber->display(armControlLeft->getHandYawMotorADCCurrent());
            }
        }
        if (armControlLeft->getHandPitchMotorCurrentTrack())
        {
            QTableWidgetItem* handPitchMotorCurrentQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getHandPitchMotorCurrentADC())
            {
                handPitchMotorCurrentQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getHandPitchMotorADCCurrent()).c_str());
                ui->leftArmReadingsTable->setItem(11, 2, handPitchMotorCurrentQWidgetItem);
                ui->leftHandPitchCurrentLcdNumber->display(armControlLeft->getHandPitchMotorADCCurrent());
            }
        }

        //track hand finger sensors
        for (int idx = 0; idx < 13; idx++)
        {
            QTableWidgetItem* handFingerSensorReadingQWidgetItem = new QTableWidgetItem;
            if (armControlLeft->getHandSensorsTrack(idx))
            {
                handFingerSensorReadingQWidgetItem->setText(Valter::format_string("%d", armControlLeft->getHandSensorsADCForce(idx)).c_str());
                ui->leftHandSensorsTable->setItem(idx, 1, handFingerSensorReadingQWidgetItem);
            }
        }

        ui->leftPalmJambLineEdit->setText(Valter::format_string("%d", armControlLeft->getPalmJambReading()).c_str());
        ui->leftFinger11TipLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger11TipReading()).c_str());
        ui->leftFinger11PhalanxLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger11PhalanxReading()).c_str());
        ui->leftFinger10PhalanxLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger10PhalanxReading()).c_str());
        ui->leftFinger10TipLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger10TipReading()).c_str());
        ui->leftFinger9PhalanxLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger9PhalanxReading()).c_str());
        ui->leftFinger8PhalanxLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger8PhalanxReading()).c_str());
        ui->leftFinger7TipLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger7TipReading()).c_str());
        ui->leftFinger7PhalanxLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger7PhalanxReading()).c_str());
        ui->leftFinger6PhalanxLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger6PhalanxReading()).c_str());
        ui->leftFinger6TipLineEdit->setText(Valter::format_string("%d", armControlLeft->getFinger6TipReading()).c_str());
        ui->leftPalmUpperLineEdit->setText(Valter::format_string("%d", armControlLeft->getPalmUpperReading()).c_str());
        ui->leftPalmLowerLineEdit->setText(Valter::format_string("%d", armControlLeft->getPalmLowerReading()).c_str());

        ui->leftPalmJambLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getPalmJambReadingRelative()).c_str());
        ui->leftFinger11TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger11TipReadingRelative()).c_str());
        ui->leftFinger11PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger11PhalanxReadingRelative()).c_str());
        ui->leftFinger10PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger10PhalanxReadingRelative()).c_str());
        ui->leftFinger10TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger10TipReadingRelative()).c_str());
        ui->leftFinger9PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger9PhalanxReadingRelative()).c_str());
        ui->leftFinger8PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger8PhalanxReadingRelative()).c_str());
        ui->leftFinger7TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger7TipReadingRelative()).c_str());
        ui->leftFinger7PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger7PhalanxReadingRelative()).c_str());
        ui->leftFinger6PhalanxLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger6PhalanxReadingRelative()).c_str());
        ui->leftFinger6TipLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getFinger6TipReadingRelative()).c_str());
        ui->leftPalmUpperLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getPalmUpperReadingRelative()).c_str());
        ui->leftPalmLowerLineEdit->setStyleSheet(Valter::format_string("background-color: rgba(255, 0, 0, %f);", armControlLeft->getPalmLowerReadingRelative()).c_str());
    }
}


void loadArmControlLeftDefaults(Ui::MainWindow *ui)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();

    armControlLeft->setDefaultsLoading(true);

    ui->leftForearmMotorDutyScroller->setMinimum(armControlLeft->getLeftForearmMotorDutyPresetMin());
    ui->leftForearmMotorDutyScroller->setMaximum(armControlLeft->getLeftForearmMotorDutyPresetMax());
    ui->leftForearmMotorDutyScroller->setValue(armControlLeft->getLeftForearmMotorDutyPresetCur());
    ui->leftForearmMotorDutyLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftForearmMotorDutyPresetCur()).c_str());

    ui->leftForearmMotorDecelerationScroller->setValue(armControlLeft->getLeftForearmMotorDeceleration());
    ui->leftForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftForearmMotorDeceleration()).c_str());

    ui->leftForearmAccelerationScroller->setValue(armControlLeft->getLeftForearmMotorAcceleration());
    ui->leftForearmAccelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftForearmMotorAcceleration()).c_str());


    ui->armControlLeftArmMotorDutyScroller->setMinimum(armControlLeft->getLeftArmMotorDutyPresetMin());
    ui->armControlLeftArmMotorDutyScroller->setMaximum(armControlLeft->getLeftArmMotorDutyPresetMax());
    ui->armControlLeftArmMotorDutyScroller->setValue(armControlLeft->getLeftArmMotorDutyPresetCur());
    ui->armControlLeftArmMotorDutyLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftArmMotorDutyPresetCur()).c_str());

    ui->armControlLeftArmMotorDecelerationScroller->setValue(armControlLeft->getLeftArmMotorDeceleration());
    ui->armControlLeftArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftArmMotorDeceleration()).c_str());

    ui->armControlLeftArmMotorAccelerationScroller->setValue(armControlLeft->getLeftArmMotorAcceleration());
    ui->armControlLeftArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftArmMotorAcceleration()).c_str());


    ui->leftLimbMotorDutyScroller->setMinimum(armControlLeft->getLeftLimbMotorDutyPresetMin());
    ui->leftLimbMotorDutyScroller->setMaximum(armControlLeft->getLeftLimbMotorDutyPresetMax());
    ui->leftLimbMotorDutyScroller->setValue(armControlLeft->getLeftLimbMotorDutyPresetCur());
    ui->leftLimbMotorDutyLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftLimbMotorDutyPresetCur()).c_str());

    ui->leftLimbMotorDecelerationScroller->setValue(armControlLeft->getLeftLimbMotorDeceleration());
    ui->leftLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftLimbMotorDeceleration()).c_str());

    ui->leftLimbMotorAccelerationScroller->setValue(armControlLeft->getLeftLimbMotorAcceleration());
    ui->leftLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", armControlLeft->getLeftLimbMotorAcceleration()).c_str());

    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(0, 0))->setCheckState((armControlLeft->getForearmPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(0, 1))->setCheckState((armControlLeft->getForearmPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(1, 0))->setCheckState((armControlLeft->getArmPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(1, 1))->setCheckState((armControlLeft->getArmPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(2, 0))->setCheckState((armControlLeft->getLimbPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(2, 1))->setCheckState((armControlLeft->getLimbPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(3, 0))->setCheckState((armControlLeft->getForearmMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(3, 1))->setCheckState((armControlLeft->getForearmMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(4, 0))->setCheckState((armControlLeft->getArmMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(4, 1))->setCheckState((armControlLeft->getArmMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(5, 0))->setCheckState((armControlLeft->getLimbMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(5, 1))->setCheckState((armControlLeft->getLimbMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(6, 0))->setCheckState((armControlLeft->getHandYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(6, 1))->setCheckState((armControlLeft->getHandYawPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(7, 0))->setCheckState((armControlLeft->getHandPitchPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(7, 1))->setCheckState((armControlLeft->getHandPitchPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(8, 0))->setCheckState((armControlLeft->getForearmYawPositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(8, 1))->setCheckState((armControlLeft->getForearmYawPositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(9, 0))->setCheckState((armControlLeft->getForearmYawMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(9, 1))->setCheckState((armControlLeft->getForearmYawMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(10, 0))->setCheckState((armControlLeft->getHandYawMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(10, 1))->setCheckState((armControlLeft->getHandYawMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(11, 0))->setCheckState((armControlLeft->getHandPitchMotorCurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->leftArmReadingsTable->item(11, 1))->setCheckState((armControlLeft->getHandPitchMotorCurrentADC()) ? Qt::Checked : Qt::Unchecked);

    for (int idx = 0; idx < 13; idx++)
    {
        ((QTableWidgetItem*)ui->leftHandSensorsTable->item(idx, 0))->setCheckState((armControlLeft->getHandSensorsTrack(idx)) ? Qt::Checked : Qt::Unchecked);
    }

    if (armControlLeft->getFingerInitialPosition(0) > armControlLeft->getFingerGraspedPosition(0))
    {
        ui->armControlLeftFinger6PositionScoller->setMinimum(armControlLeft->getFingerGraspedPosition(0));
        ui->armControlLeftFinger6PositionScoller->setMaximum(armControlLeft->getFingerInitialPosition(0));
    }
    else
    {
        ui->armControlLeftFinger6PositionScoller->setMaximum(armControlLeft->getFingerGraspedPosition(0));
        ui->armControlLeftFinger6PositionScoller->setMinimum(armControlLeft->getFingerInitialPosition(0));
    }
    ui->armControlLeftFinger6PositionScoller->setValue(armControlLeft->getFingerInitialPosition(0));

    if (armControlLeft->getFingerInitialPosition(1) > armControlLeft->getFingerGraspedPosition(1))
    {
        ui->armControlLeftFinger7PositionScoller->setMinimum(armControlLeft->getFingerGraspedPosition(1));
        ui->armControlLeftFinger7PositionScoller->setMaximum(armControlLeft->getFingerInitialPosition(1));
    }
    else
    {
        ui->armControlLeftFinger7PositionScoller->setMaximum(armControlLeft->getFingerGraspedPosition(1));
        ui->armControlLeftFinger7PositionScoller->setMinimum(armControlLeft->getFingerInitialPosition(1));
    }
    ui->armControlLeftFinger7PositionScoller->setValue(armControlLeft->getFingerInitialPosition(1));
    ui->armControlLeftFinger7PositionScoller->setInvertedAppearance(true);

    if (armControlLeft->getFingerInitialPosition(2) > armControlLeft->getFingerGraspedPosition(2))
    {
        ui->armControlLeftFinger8PositionScoller->setMinimum(armControlLeft->getFingerGraspedPosition(2));
        ui->armControlLeftFinger8PositionScoller->setMaximum(armControlLeft->getFingerInitialPosition(2));
    }
    else
    {
        ui->armControlLeftFinger8PositionScoller->setMaximum(armControlLeft->getFingerGraspedPosition(2));
        ui->armControlLeftFinger8PositionScoller->setMinimum(armControlLeft->getFingerInitialPosition(2));
    }
    ui->armControlLeftFinger8PositionScoller->setValue(armControlLeft->getFingerInitialPosition(2));
    ui->armControlLeftFinger8PositionScoller->setInvertedAppearance(true);

    if (armControlLeft->getFingerInitialPosition(3) > armControlLeft->getFingerGraspedPosition(3))
    {
        ui->armControlLeftFinger9PositionScoller->setMinimum(armControlLeft->getFingerGraspedPosition(3));
        ui->armControlLeftFinger9PositionScoller->setMaximum(armControlLeft->getFingerInitialPosition(3));
    }
    else
    {
        ui->armControlLeftFinger9PositionScoller->setMaximum(armControlLeft->getFingerGraspedPosition(3));
        ui->armControlLeftFinger9PositionScoller->setMinimum(armControlLeft->getFingerInitialPosition(3));
    }
    ui->armControlLeftFinger9PositionScoller->setValue(armControlLeft->getFingerInitialPosition(3));

    if (armControlLeft->getFingerInitialPosition(4) > armControlLeft->getFingerGraspedPosition(4))
    {
        ui->armControlLeftFinger10PositionScoller->setMinimum(armControlLeft->getFingerGraspedPosition(4));
        ui->armControlLeftFinger10PositionScoller->setMaximum(armControlLeft->getFingerInitialPosition(4));
    }
    else
    {
        ui->armControlLeftFinger10PositionScoller->setMaximum(armControlLeft->getFingerGraspedPosition(4));
        ui->armControlLeftFinger10PositionScoller->setMinimum(armControlLeft->getFingerInitialPosition(4));
    }
    ui->armControlLeftFinger10PositionScoller->setValue(armControlLeft->getFingerInitialPosition(4));

    if (armControlLeft->getFingerInitialPosition(5) > armControlLeft->getFingerGraspedPosition(5))
    {
        ui->armControlLeftFinger11PositionScoller->setMinimum(armControlLeft->getFingerGraspedPosition(5));
        ui->armControlLeftFinger11PositionScoller->setMaximum(armControlLeft->getFingerInitialPosition(5));
    }
    else
    {
        ui->armControlLeftFinger11PositionScoller->setMaximum(armControlLeft->getFingerGraspedPosition(5));
        ui->armControlLeftFinger11PositionScoller->setMinimum(armControlLeft->getFingerInitialPosition(5));
    }
    ui->armControlLeftFinger11PositionScoller->setValue(armControlLeft->getFingerInitialPosition(5));
    ui->armControlLeftFinger11PositionScoller->setInvertedAppearance(true);

    armControlLeft->setDefaultsLoading(false);
}

void setLeftArmReadingsPresets(QTableWidgetItem *item)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    switch (item->row())
    {
        case 0://forearmPosition
            if (item->column() == 0)
            {
                armControlLeft->setForearmPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setForearmPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 1://armPosition
            if (item->column() == 0)
            {
                armControlLeft->setArmPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setArmPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 2://limbPosition
            if (item->column() == 0)
            {
                armControlLeft->setLimbPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setLimbPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 3://forearmMotorCurrent
            if (item->column() == 0)
            {
                armControlLeft->setForearmMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setForearmMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 4://armMotorCurrent
            if (item->column() == 0)
            {
                armControlLeft->setArmMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setArmMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 5://limbMotorCurrent
            if (item->column() == 0)
            {
                armControlLeft->setLimbMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setLimbMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 6://handYawPosition
            if (item->column() == 0)
            {
                armControlLeft->setHandYawPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setHandYawPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 7://handPitchPosition
            if (item->column() == 0)
            {
                armControlLeft->setHandPitchPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setHandPitchPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 8://forearmYawPosition
            if (item->column() == 0)
            {
                armControlLeft->setForearmYawPositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setForearmYawPositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 9://forearmYawMotorCurrent
            if (item->column() == 0)
            {
                armControlLeft->setForearmYawMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setForearmYawMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 10://handYawMotorCurrent
            if (item->column() == 0)
            {
                armControlLeft->setHandYawMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setHandYawMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 11://handPitchMotorCurrent
            if (item->column() == 0)
            {
                armControlLeft->setHandPitchMotorCurrentTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                armControlLeft->setHandPitchMotorCurrentADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
    }
}

void setLeftHandForceSensorsReadingsPresets(QTableWidgetItem *item)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (item->column() == 0)
    {
        armControlLeft->setHandSensorsTrack(item->row(), ((item->checkState() == Qt::Checked) ? true : false));
    }
}


#endif // ARMCONTROLLEFTGUI_H
