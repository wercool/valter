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
    if (ui->platformManipulatorAndIRBumperRedrawGUICheckBox->isChecked()) //PLATFROM-LOCATION-P1 Tab
    {
        ui->platformManipulatorAngleALcdNumber->display(Valter::format_string("%.2f", PlatformManipulatorAndIRBumper::man_a * 180 / M_PI).c_str());
        ui->platformManipulatorAngleBLcdNumber->display(Valter::format_string("%.2f", PlatformManipulatorAndIRBumper::man_b * 180 / M_PI).c_str());
        ui->platformManipulatorAngleGLcdNumber->display(Valter::format_string("%.2f", PlatformManipulatorAndIRBumper::man_g * 180 / M_PI).c_str());
    }
}

#endif // PLATFORMMANIPULATORANDIRBUMPERGUI_H
