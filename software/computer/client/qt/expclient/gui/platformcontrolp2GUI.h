#ifndef PLATFORMCONTROLP2GUI_H
#define PLATFORMCONTROLP2GUI_H

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void loadPlatformControlP2Defaults(Ui::MainWindow *ui)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();

    ui->platformControlP2LeftWheelEncoderCheckBox->setChecked(platformControlP2->getReadLeftEncoder());
    ui->platformControlP2RightWheelEncoderCheckBox->setChecked(platformControlP2->getReadRightEncoder());

    ui->platformControlP2LeftWheelEncoderAutoresetCheckBox->setChecked(platformControlP2->getLeftEncoderAutoreset());
    ui->platformControlP2RightWheelEncoderAutoresetCheckBox->setChecked(platformControlP2->getRightEncoderAutoreset());

    ui->chargerMotorDutyScrollBar->setMinimum(platformControlP2->getChargerMotorDutyPresetMin());
    ui->chargerMotorDutyScrollBar->setMaximum(platformControlP2->getChargerMotorDutyPresetMax());
    ui->chargerMotorDutyScrollBar->setValue(platformControlP2->getChargerMotorDutyPresetCur());
    ui->chargerMotorDutyLabel->setText(Valter::format_string("[%d]", platformControlP2->getChargerMotorDutyPresetCur()).c_str());

    ui->chargerMotorPushDurationScrollBar->setMinimum(platformControlP2->getChargerMotorPushDurationPresetMin());
    ui->chargerMotorPushDurationScrollBar->setMaximum(platformControlP2->getChargerMotorPushDurationPresetMax());
    ui->chargerMotorPushDurationScrollBar->setValue(platformControlP2->getChargerMotorPushDuration());
    ui->chargerMotorPushDurationLabel->setText(Valter::format_string("[%d]", platformControlP2->getChargerMotorPushDuration()).c_str());

    ui->beepDurationScrollBar->setMinimum(platformControlP2->getBeepDurationPresetMin());
    ui->beepDurationScrollBar->setMaximum(platformControlP2->getBeepDurationPresetMax());
    ui->beepDurationScrollBar->setValue(platformControlP2->getBeepDuration());

    ui->beepDurationLabel->setText(Valter::format_string("[%d]", platformControlP2->getBeepDuration()).c_str());

    ui->chargerLedsButton->setChecked(platformControlP2->getChargerLeds());
    ui->bottomFronLedsButton->setChecked(platformControlP2->getBottomFrontLeds());
    ui->bottomRearLedsButton->setChecked(platformControlP2->getBottomRearLeds());

    ui->irScanningButton->setChecked(platformControlP2->getIrScanningWorkerActivated());
}

void platformControlP2TabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platfromControlP2RedrawGUICheckBox->isChecked()) //PLATFROM-CONTROL-P2 Tab
    {
        if (platformControlP2->getReadLeftEncoder() || platformControlP2->leftEncoderGetOnce)
        {
            ui->platformControlP2LeftWheelEncoderLcdNumber->display(platformControlP2->getLeftEncoder());
            if (platformControlP2->leftEncoderGetOnce)
            {
                platformControlP2->leftEncoderGetOnce = false;
                MainWindow::getInstance()->platformControlP2EncodersRefreshTimer->stop();
            }
        }
        if (platformControlP2->getReadRightEncoder() || platformControlP2->rightEncoderGetOnce)
        {
            ui->platformControlP2RightWheelEncoderLcdNumber->display(platformControlP2->getRightEncoder());
            if (platformControlP2->rightEncoderGetOnce)
            {
                platformControlP2->rightEncoderGetOnce = false;
                MainWindow::getInstance()->platformControlP2EncodersRefreshTimer->stop();
            }
        }
    }
}

void platformControlP2IRScannerRefresh(Ui::MainWindow *ui)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platfromControlP2RedrawGUICheckBox->isChecked()) //PLATFROM-CONTROL-P2 Tab
    {
        qDebug("!!!!!");
    }
}

#endif // PLATFORMCONTROLP2GUI_H
