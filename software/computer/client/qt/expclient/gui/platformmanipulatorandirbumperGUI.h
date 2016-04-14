#ifndef PLATFORMMANIPULATORANDIRBUMPERGUI_H
#define PLATFORMMANIPULATORANDIRBUMPERGUI_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void platformManipulatorAndIRBumperRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    if (ui->platformManipulatorAndIRBumperRedrawGUICheckBox->isChecked())
    {
        //ui->platformManipulatorAngleALcdNumber->display(Valter::format_string("%.2f", PlatformManipulatorAndIRBumper::man_a * 180 / M_PI).c_str());
        //ui->platformManipulatorAngleBLcdNumber->display(Valter::format_string("%.2f", PlatformManipulatorAndIRBumper::man_b * 180 / M_PI).c_str());
        //ui->platformManipulatorAngleGLcdNumber->display(Valter::format_string("%.2f", PlatformManipulatorAndIRBumper::man_g * 180 / M_PI).c_str());

        ui->is24VOnRadioButton->setChecked(platformManipulatorAndIRBumper->getPower24VOnOff());

        ui->manLink1DutyProgressBar->setValue(platformManipulatorAndIRBumper->getLink1MotorDuty());
        ui->manLink2DutyProgressBar->setValue(platformManipulatorAndIRBumper->getLink2MotorDuty());

        if (platformManipulatorAndIRBumper->getLink1PositionTrack())
        {
            QTableWidgetItem* link1PositionQWidgetItem = new QTableWidgetItem;
            if (platformManipulatorAndIRBumper->getLink1PositionADC()) // show ADC value
            {
                link1PositionQWidgetItem->setText(Valter::format_string("%d", platformManipulatorAndIRBumper->getLink1ADCPosition()).c_str());
            }
            ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->setItem(0, 2, link1PositionQWidgetItem);
        }
        if (platformManipulatorAndIRBumper->getLink2PositionTrack())
        {
            QTableWidgetItem* link2PositionQWidgetItem = new QTableWidgetItem;
            if (platformManipulatorAndIRBumper->getLink2PositionADC()) // show ADC value
            {
                link2PositionQWidgetItem->setText(Valter::format_string("%d", platformManipulatorAndIRBumper->getLink2ADCPosition()).c_str());
            }
            ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->setItem(1, 2, link2PositionQWidgetItem);
        }
    }
}

void getAndSetCurrentABGangles()
{
    Ui::MainWindow *ui = MainWindow::getInstance()->getUi();
    ui->platformManipulatorAngleASpinBox->setValue(PlatformManipulatorAndIRBumper::man_a * 180 / M_PI);
    ui->platformManipulatorAngleBSpinBox->setValue(PlatformManipulatorAndIRBumper::man_b * 180 / M_PI);
    ui->platformManipulatorAngleGSpinBox->setValue(PlatformManipulatorAndIRBumper::man_g * 180 / M_PI);
}

void loadPlatformManipulatorAndIRBumperDefaults(Ui::MainWindow *ui)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();

    ui->manipulatorLiknk1MotorDutyScroller->setMinimum(platformManipulatorAndIRBumper->getLink1MotorDutyPresetMin());
    ui->manipulatorLiknk1MotorDutyScroller->setMaximum(platformManipulatorAndIRBumper->getLink1MotorDutyPresetMax());
    ui->manipulatorLiknk1MotorDutyScroller->setValue(platformManipulatorAndIRBumper->getLink1MotorDutyPresetCur());

    ui->manipulatorLiknk1DecelerationScroller->setValue(platformManipulatorAndIRBumper->getLink1MotorDeceleration());
    ui->manipulatorLiknk1AccelerationScroller->setValue(platformManipulatorAndIRBumper->getLink1MotorAcceleration());

    ui->manipulatorLiknk2MotorDutyScroller->setMinimum(platformManipulatorAndIRBumper->getLink2MotorDutyPresetMin());
    ui->manipulatorLiknk2MotorDutyScroller->setMaximum(platformManipulatorAndIRBumper->getLink2MotorDutyPresetMax());
    ui->manipulatorLiknk2MotorDutyScroller->setValue(platformManipulatorAndIRBumper->getLink2MotorDutyPresetCur());

    ui->manipulatorLiknk2DecelerationScroller->setValue(platformManipulatorAndIRBumper->getLink2MotorDeceleration());
    ui->manipulatorLiknk2AccelerationScroller->setValue(platformManipulatorAndIRBumper->getLink2MotorAcceleration());

    ui->manGripperRotationMotorDutyScroller->setValue(platformManipulatorAndIRBumper->getManGripperRotationMotorDuty());

    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(0, 0))->setCheckState((platformManipulatorAndIRBumper->getLink1PositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(0, 1))->setCheckState((platformManipulatorAndIRBumper->getLink1PositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(1, 0))->setCheckState((platformManipulatorAndIRBumper->getLink2PositionTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(1, 1))->setCheckState((platformManipulatorAndIRBumper->getLink2PositionADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(2, 0))->setCheckState((platformManipulatorAndIRBumper->getLink1CurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(2, 1))->setCheckState((platformManipulatorAndIRBumper->getLink1CurrentADC()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(3, 0))->setCheckState((platformManipulatorAndIRBumper->getLink2CurrentTrack()) ? Qt::Checked : Qt::Unchecked);
    ((QTableWidgetItem*)ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(3, 1))->setCheckState((platformManipulatorAndIRBumper->getLink2CurrentADC()) ? Qt::Checked : Qt::Unchecked);

    ui->platformManipulatorAndIRBumperRedrawGUICheckBox->setChecked(true);
}

void setPlatformManipulatorReadingsPresets(QTableWidgetItem *item)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    switch (item->row())
    {
        case 0://Link1 Rotation
            if (item->column() == 0)
            {
                platformManipulatorAndIRBumper->setLink1PositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                platformManipulatorAndIRBumper->setLink1PositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 1://Link2 Rotation
            if (item->column() == 0)
            {
                platformManipulatorAndIRBumper->setLink2PositionTrack((item->checkState() == Qt::Checked) ? true : false);
            }
            if (item->column() == 1)
            {
                platformManipulatorAndIRBumper->setLink2PositionADC((item->checkState() == Qt::Checked) ? true : false);
            }
        break;
        case 2://Link1 Motor Current
        break;
        case 3://Link2 Motor Current
        break;
        case 4://CH0 Gripper Tilt
        break;
        case 5://CH0 Gripper Rotation
        break;
        case 6://CH2 Gripper Position
        break;
        case 7://CH3 Gripper Force Sensor 1
        break;
        case 8://CH4 Gripper Force Sensor 2
        break;
        case 9://CH5 Gripper Force Sensor 3
        break;
        case 10://CH6 Gripper Object Detector
        break;
        case 11://CH7 Gripper Tilt Motor Current
        break;
        case 12://CH8 Gripper Open/Close Current
        break;
        case 13://CH8 Gripper Rotation Motor Current
        break;
    }
}

#endif // PLATFORMMANIPULATORANDIRBUMPERGUI_H
