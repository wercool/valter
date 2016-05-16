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
