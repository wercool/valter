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

    ui->irScannerAngleScrollBar->setMinimum(platformControlP2->getIRScannerMinAngle());
    ui->irScannerAngleScrollBar->setMaximum(platformControlP2->getIRScannerMaxAngle());

    ui->irScannerAngleLabel->setText(Valter::format_string("[%d]", platformControlP2->getIRScannerAngle()).c_str());
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
        if (platformControlP2->getIntetntionalIRScannerReadingReuqest() == 2)
        {
            ui->platfromControlP2IRScannerReadingLCDNumber->display(platformControlP2->getIrScannerReadingADC());
            platformControlP2->setIntetntionalIRScannerReadingReuqest(0);
        }
    }
}

void platformControlP2IRScannerRefresh(Ui::MainWindow *ui)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platfromControlP2RedrawGUICheckBox->isChecked()) //PLATFROM-CONTROL-P2 Tab
    {
        if (platformControlP2->getIrScanningWorkerActivated() || platformControlP2->getIRScannerIntentionalAngleSet())
        {
            double endX;
            double endY;
            if (!platformControlP2->getIRScannerIntentionalAngleSet() && platformControlP2->getIRScannerScan(platformControlP2->getIRScannerAngle()) != 0.0)
            {
                endX = 293 + 0.7 * platformControlP2->getIRScannerScan(platformControlP2->getIRScannerAngle()) * sin((180 - platformControlP2->getIRScannerAngle()) * M_PI / 180);
                endY = 296 + 0.7 * platformControlP2->getIRScannerScan(platformControlP2->getIRScannerAngle()) * cos((180 - platformControlP2->getIRScannerAngle()) * M_PI / 180);
                MainWindow::getInstance()->IRScannerVector->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
            }
            else
            {
                endX = 293 + 250 * sin((180 - platformControlP2->getIRScannerAngle()) * M_PI / 180);
                endY = 296 + 250 * cos((180 - platformControlP2->getIRScannerAngle()) * M_PI / 180);
                MainWindow::getInstance()->IRScannerVector->setPen(QPen(Qt::yellow, 1.0, Qt::DashLine));
            }

            MainWindow::getInstance()->IRScannerVector->setLine(293, 296, endX, endY);
            if (!platformControlP2->getIRScannerIntentionalAngleSet())
            {
                ui->irScannerAngleScrollBar->setValue(platformControlP2->getIRScannerAngle());
            }

            if (ui->platfromControlP2IRScannerADCCheckBox->isChecked())
            {
                ui->platfromControlP2IRScannerReadingLCDNumber->display(platformControlP2->getIRScannerScan(platformControlP2->getIRScannerAngle()));
            }

            if (platformControlP2->getIrScanningWorkerActivated() && platformControlP2->getIRScannerScan(platformControlP2->getIRScannerAngle()) != 0.0)
            {
                if (!MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()])
                {
                    MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()] = new QGraphicsEllipseItem;
                    ((QGraphicsEllipseItem*)MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()])->setPos(QPointF(endX - 8, endY - 8));
                    ((QGraphicsEllipseItem*)MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()])->setRect( 0, 0, 16, 16 );
                    MainWindow::getInstance()->irScanningGraphicsViewScene->addItem(((QGraphicsEllipseItem*)MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()]));
                }
                else
                {
                    //((QGraphicsEllipseItem*)MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()])->setRect( endX - 8, endY - 8, 16, 16 );
                    ((QGraphicsEllipseItem*)MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()])->setPos(QPointF(endX - 8, endY - 8));
                    ((QGraphicsEllipseItem*)MainWindow::getInstance()->IRScannerDots[platformControlP2->getIRScannerAngle()])->setPen(QPen(Qt::black));
                }
            }
        }
    }
}

#endif // PLATFORMCONTROLP2GUI_H
