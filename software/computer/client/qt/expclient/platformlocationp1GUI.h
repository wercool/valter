#ifndef PLATFORMLOCATIONP1GUI_H
#define PLATFORMLOCATIONP1GUI_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

#define PI 3.14159

void platformLocationP1TabRefreshTimerUpdateWorker(Ui::MainWindow *ui)
{
    if (ui->platformLocationP1RedrawGUICheckBox->isChecked()) //PLATFROM-LOCATION-P1 Tab
    {
        PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
        QLCDNumber* sensorIRLCDNumber;
        QLCDNumber* sensorUSLCDNumber;
        for (int ch = 0; ch < 12; ch++)
        {
            sensorIRLCDNumber = ui->platformLocationP1Tab->findChild<QLCDNumber*>(Valter::format_string("ir%dLcdNumber", ch).c_str());
            if (platformLocationP1->getReadIRSensor(ch))
            {
                if (platformLocationP1->getReadIRSensorADCPreset(ch))
                {
                    sensorIRLCDNumber->display(platformLocationP1->getIRSensorADC(ch));
                }
                else
                {
                    sensorIRLCDNumber->display(platformLocationP1->getIRSensorMeters(ch));
                }
                sensorIRLCDNumber->setStyleSheet("background-color: rgb(255, 255, 255);");
            }
            else
            {
                sensorIRLCDNumber->setStyleSheet("background-color: rgb(239, 239, 239);");
            }

            sensorUSLCDNumber = ui->platformLocationP1Tab->findChild<QLCDNumber*>(Valter::format_string("us%dLcdNumber", ch).c_str());
            if (platformLocationP1->getReadUSSensor(ch))
            {
                if (platformLocationP1->getReadUSSensorTicksPreset(ch))
                {
                    sensorUSLCDNumber->display(platformLocationP1->getUSSensorTicks(ch));
                }
                else
                {
                    sensorUSLCDNumber->display(platformLocationP1->getUSSensorMeters(ch));
                }
                if (platformLocationP1->getUSSensorTicks(ch) > 4999)
                {
                    sensorUSLCDNumber->setStyleSheet("background-color: rgb(255, 254, 157);");
                }
                else
                {
                    sensorUSLCDNumber->setStyleSheet("background-color: rgb(255, 255, 255);");
                }
            }
            else
            {
                sensorUSLCDNumber->setStyleSheet("background-color: rgb(239, 239, 239);");
            }


            if (platformLocationP1->getLedStatesSet())
            {
                QPushButton* redLedWidget = ui->platformLocationP1Tab->findChild<QPushButton*>(Valter::format_string("ch%dRedLed", ch).c_str());
                QPushButton* greenLedWidget = ui->platformLocationP1Tab->findChild<QPushButton*>(Valter::format_string("ch%dGreenLed", ch).c_str());

                if (platformLocationP1->getReadIRSensor(ch) || platformLocationP1->getReadUSSensor(ch))
                {
                    if (platformLocationP1->getRedLedState(ch))
                    {
                        redLedWidget->setIcon(MainWindow::getInstance()->redLedOnIcon);
                        greenLedWidget->setIcon(MainWindow::getInstance()->greenLedOffIcon);
                    }
                    if (platformLocationP1->getGreenLedState(ch))
                    {
                        redLedWidget->setIcon(MainWindow::getInstance()->redLedOffIcon);
                        greenLedWidget->setIcon(MainWindow::getInstance()->greenLedOnIcon);
                    }
                }
                else
                {
                    redLedWidget->setIcon(MainWindow::getInstance()->redLedOffIcon);
                    greenLedWidget->setIcon(MainWindow::getInstance()->greenLedOffIcon);
                }
            }
        }

        //sonar animation goes here
        double endX;
        double endY;

        if (platformLocationP1->getLeftSonarActivated() || platformLocationP1->getLeftSonarIntentionalAngleSet())
        {
            if (platformLocationP1->getLeftSonarScan(platformLocationP1->getLeftSonarAngle()) != 0.0)
            {
                endX = 345 + platformLocationP1->getLeftSonarScan(platformLocationP1->getLeftSonarAngle()) * sin((180 - platformLocationP1->getLeftSonarAngle()) * PI / 180);
                endY = 285 + platformLocationP1->getLeftSonarScan(platformLocationP1->getLeftSonarAngle()) * cos((180 - platformLocationP1->getLeftSonarAngle()) * PI / 180);
                MainWindow::getInstance()->leftUSSonarVector->setLine(345, 285, endX, endY);
                ui->leftSonarAngleScroller->setValue(platformLocationP1->getLeftSonarAngle());

                if (platformLocationP1->getLeftSonarActivated())
                {
                    if (!MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])
                    {
                        MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()] = new QGraphicsEllipseItem;
                        MainWindow::getInstance()->platformLocationP1SonarsGraphicsViewScene->addItem(((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()]));
                    }
                    else
                    {
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setRect( endX - 8, endY - 8, 16, 16 );
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setPen(QPen(Qt::black));
                    }
                }
            }
        }

        if (platformLocationP1->getRightSonarActivated() || platformLocationP1->getRightSonarIntentionalAngleSet())
        {
            if (platformLocationP1->getRightSonarScan(platformLocationP1->getRightSonarAngle()) != 0.0)
            {
                endX = 445 + platformLocationP1->getRightSonarScan(platformLocationP1->getRightSonarAngle()) * sin((180 - platformLocationP1->getRightSonarAngle()) * PI / 180);
                endY = 285 + platformLocationP1->getRightSonarScan(platformLocationP1->getRightSonarAngle()) * cos((180 - platformLocationP1->getRightSonarAngle()) * PI / 180);
                MainWindow::getInstance()->rightUSSonarVector->setLine(445, 285, endX, endY);
                ui->rightSonarAngleScroller->setValue(platformLocationP1->getRightSonarAngle());

                if (platformLocationP1->getRightSonarActivated())
                {
                    if (!MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])
                    {
                        MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()] = new QGraphicsEllipseItem;
                        MainWindow::getInstance()->platformLocationP1SonarsGraphicsViewScene->addItem(((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()]));
                    }
                    else
                    {
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setRect( endX - 8, endY - 8, 16, 16 );
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setPen(QPen(Qt::black));
                    }
                }
            }
        }

        if (platformLocationP1->getAccelerometerWorkerActivated())
        {
            ui->accelerometerXLcdNumber->display(platformLocationP1->getAccelerometerReadings()[0]);
            ui->accelerometerYLcdNumber->display(platformLocationP1->getAccelerometerReadings()[1]);
            ui->accelerometerZLcdNumber->display(platformLocationP1->getAccelerometerReadings()[2]);
        }

        if (platformLocationP1->getMagnetometerWorkerActivated())
        {
            ui->magnetometerXLcdNumber->display(platformLocationP1->getMagnetometerReadings()[0]);
            ui->magnetometerYLcdNumber->display(platformLocationP1->getMagnetometerReadings()[1]);
            ui->magnetometerZLcdNumber->display(platformLocationP1->getMagnetometerReadings()[2]);
        }
    }
}

void accelerometerRefreshGraphicsView()
{
    if (PlatformLocationP1::getInstance()->getAccelerometerWorkerActivated())
    {
        static qreal x = 0.0;
        static qreal y = MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 3;
        static qreal x1 = 0.0;
        static qreal y1 = 0.0;

        x1 = x + 1;
        if (rand() % 1)
        {
            y1 = 25 - rand() % 100;
        }
        else
        {
            y1 = 25 + rand() % 100;
        }
        MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->addLine(x, y, x1, y1, QPen(Qt::red));
        x = x1;
        y = y1;

        if (x > MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->width())
        {
            MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->clear();
            x = 0.0;
        }
    }
}

void loadPlatformLocationP1Defaults(Ui::MainWindow *ui)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());

    //IRSensorsReadTable
    for (int i = 0; i < 12; i++)
    {
        ((QTableWidgetItem*)ui->irSensorsPresetsTable->item(i, 1))->setCheckState((platformLocationP1->getReadIRSensor(i)) ? Qt::Checked : Qt::Unchecked);
        ((QTableWidgetItem*)ui->irSensorsPresetsTable->item(i, 2))->setCheckState((platformLocationP1->getReadIRSensorADCPreset(i)) ? Qt::Checked : Qt::Unchecked);
        ((QTableWidgetItem*)ui->usSensorsPresetsTable->item(i, 1))->setCheckState((platformLocationP1->getReadUSSensor(i)) ? Qt::Checked : Qt::Unchecked);
        ((QTableWidgetItem*)ui->usSensorsPresetsTable->item(i, 2))->setCheckState((platformLocationP1->getReadUSSensorTicksPreset(i)) ? Qt::Checked : Qt::Unchecked);
    }

    ui->USVoltageRegulatorLabel->setText(Valter::format_string("[%d] relative to initial", platformLocationP1->getRelativeUSSensorVoltage()).c_str());
    ui->USSignalDutyLabel->setText(Valter::format_string("[%d]", platformLocationP1->getUsSignalDuty()).c_str());
    ui->USSignalBurstLabel->setText(Valter::format_string("[%d]", platformLocationP1->getUsSignalBurst()).c_str());
    ui->USSignalDelayLabel->setText(Valter::format_string("[%d]", platformLocationP1->getUsSignalDelay()).c_str());

    ui->leftSonarAngleScroller->setMinimum(platformLocationP1->getLeftSonarMinAngle());
    ui->leftSonarAngleScroller->setMaximum(platformLocationP1->getLeftSonarMaxAngle());
    ui->rightSonarAngleScroller->setMinimum(platformLocationP1->getRightSonarMinAngle());
    ui->rightSonarAngleScroller->setMaximum(platformLocationP1->getRightSonarMaxAngle());

    ui->leftSonarAngleLabel->setText(Valter::format_string("[%d]", platformLocationP1->getLeftSonarAngle()).c_str());
    ui->rightSonarAngleLabel->setText(Valter::format_string("[%d]", platformLocationP1->getRightSonarAngle()).c_str());
}

void setPlatformLocationIRSensorPresets(QTableWidgetItem *item)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    //track IR sensor
    if (item->column() == 1)
    {
        platformLocationP1->setReadIRSensor(item->row(), (item->checkState() == Qt::Checked) ? true : false);
    }
    //track IR sensor and show ADC/native reading
    if (item->column() == 2)
    {
        platformLocationP1->setReadIRSensorADCPreset(item->row(), (item->checkState() == Qt::Checked) ? true : false);
    }
}

void setPlatformLocationUSSensorPresets(QTableWidgetItem *item)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    //track US sensor
    if (item->column() == 1)
    {
        platformLocationP1->setReadUSSensor(item->row(), (item->checkState() == Qt::Checked) ? true : false);
    }
    //track US sensor and show ADC/native reading
    if (item->column() == 2)
    {
        platformLocationP1->setReadUSSensorTicksPreset(item->row(), (item->checkState() == Qt::Checked) ? true : false);
    }
}

void initLedButtons(MainWindow *mainWindow)
{
    mainWindow->getUi()->ch0RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch1RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch2RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch3RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch4RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch5RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch6RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch7RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch8RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch9RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch10RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch11RedLed->setIcon(mainWindow->redLedOffIcon);

    mainWindow->getUi()->ch0GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch1GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch2GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch3GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch4GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch5GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch6GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch7GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch8GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch9GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch10GreenLed->setIcon(mainWindow->greenLedOffIcon);
    mainWindow->getUi()->ch11GreenLed->setIcon(mainWindow->greenLedOffIcon);
}

void setRedLedButtonOn(MainWindow *mainWindow, QPushButton *btn, bool state)
{
    if (state)
    {
        btn->setIcon(mainWindow->redLedOnIcon);
    }
    else
    {
        btn->setIcon(mainWindow->redLedOffIcon);
    }
}

void setRedLedButtonOff(MainWindow *mainWindow, QPushButton *btn)
{
    btn->setIcon(mainWindow->redLedOffIcon);
}

void setGreenLedButtonOn(MainWindow *mainWindow, QPushButton *btn, bool state)
{
    if (state)
    {
        btn->setIcon(mainWindow->greenLedOnIcon);
    }
    else
    {
        btn->setIcon(mainWindow->greenLedOffIcon);
    }
}

void setGreenLedButtonOff(MainWindow *mainWindow, QPushButton *btn)
{
    btn->setIcon(mainWindow->greenLedOffIcon);
}

#endif // PLATFORMLOCATIONP1GUI_H
