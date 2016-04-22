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

    }
}

void loadArmControlRightDefaults(Ui::MainWindow *ui)
{

}

void setRightArmReadingsPresets(QTableWidgetItem *item)
{

}

void setLeftHandForceSensorsReadingsPresets()
{

}

#endif // ARMCONTROLRIGHTGUI_H
