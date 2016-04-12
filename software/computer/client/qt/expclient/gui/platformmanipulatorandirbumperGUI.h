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

    ui->platformManipulatorAndIRBumperRedrawGUICheckBox->setChecked(true);
}

#endif // PLATFORMMANIPULATORANDIRBUMPERGUI_H
