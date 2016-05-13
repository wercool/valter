#ifndef PLATFORMLOCATIONP1GUI_H
#define PLATFORMLOCATIONP1GUI_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

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
                endX = 345 + platformLocationP1->getLeftSonarScan(platformLocationP1->getLeftSonarAngle()) * sin((180 - platformLocationP1->getLeftSonarAngle()) * M_PI / 180);
                endY = 285 + platformLocationP1->getLeftSonarScan(platformLocationP1->getLeftSonarAngle()) * cos((180 - platformLocationP1->getLeftSonarAngle()) * M_PI / 180);
                MainWindow::getInstance()->leftUSSonarVector->setLine(345, 285, endX, endY);
                ui->leftSonarAngleScroller->setValue(platformLocationP1->getLeftSonarAngle());

                if (platformLocationP1->getLeftSonarActivated())
                {
                    if (!MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])
                    {
                        MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()] = new QGraphicsEllipseItem;
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setPos(QPointF(endX - 8, endY - 8));
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setRect( 0, 0, 16, 16 );
                        MainWindow::getInstance()->platformLocationP1SonarsGraphicsViewScene->addItem(((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()]));
                    }
                    else
                    {
                        //((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setRect( endX - 8, endY - 8, 16, 16 );
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setPos(QPointF(endX - 8, endY - 8));
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->leftSonarDots[platformLocationP1->getLeftSonarAngle()])->setPen(QPen(Qt::black));
                    }
                }
            }
        }

        if (platformLocationP1->getRightSonarActivated() || platformLocationP1->getRightSonarIntentionalAngleSet())
        {
            if (platformLocationP1->getRightSonarScan(platformLocationP1->getRightSonarAngle()) != 0.0)
            {
                endX = 445 + platformLocationP1->getRightSonarScan(platformLocationP1->getRightSonarAngle()) * sin((180 - platformLocationP1->getRightSonarAngle()) * M_PI / 180);
                endY = 285 + platformLocationP1->getRightSonarScan(platformLocationP1->getRightSonarAngle()) * cos((180 - platformLocationP1->getRightSonarAngle()) * M_PI / 180);
                MainWindow::getInstance()->rightUSSonarVector->setLine(445, 285, endX, endY);
                ui->rightSonarAngleScroller->setValue(platformLocationP1->getRightSonarAngle());

                if (platformLocationP1->getRightSonarActivated())
                {
                    if (!MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])
                    {
                        MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()] = new QGraphicsEllipseItem;
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setPos(QPointF(endX - 8, endY - 8));
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setRect( 0, 0, 16, 16 );
                        MainWindow::getInstance()->platformLocationP1SonarsGraphicsViewScene->addItem(((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()]));
                    }
                    else
                    {
                        //((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setRect( endX - 8, endY - 8, 16, 16 );
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setPos(QPointF(endX - 8, endY - 8));
                        ((QGraphicsEllipseItem*)MainWindow::getInstance()->rightSonarDots[platformLocationP1->getRightSonarAngle()])->setPen(QPen(Qt::black));
                    }
                }
            }
        }

        static double prevAccX, prevAccY, prevAccZ;

        double AccX = platformLocationP1->getAccelerometerReadings()[0];
        double AccY = platformLocationP1->getAccelerometerReadings()[1];
        double AccZ = platformLocationP1->getAccelerometerReadings()[2];

        bool updateAcc = false;

        if (prevAccX != AccX || prevAccY != AccY || prevAccZ != AccZ)
        {
            updateAcc = true;
        }

        prevAccX = AccX;
        prevAccY = AccY;
        prevAccZ = AccZ;

        if (platformLocationP1->getAccelerometerWorkerActivated() || updateAcc)
        {
            if (ui->accelerometerRawCheckBox->isChecked())
            {
                ui->accelerometerXLcdNumber->display(AccX);
                ui->accelerometerYLcdNumber->display(AccY);
                ui->accelerometerZLcdNumber->display(AccZ);
            }
            else
            {
                ui->accelerometerXLcdNumber->display(platformLocationP1->getAccelerometerNaturalReadings()[0]);
                ui->accelerometerYLcdNumber->display(platformLocationP1->getAccelerometerNaturalReadings()[1]);
                ui->accelerometerZLcdNumber->display(platformLocationP1->getAccelerometerNaturalReadings()[2]);
            }
            if (ui->inclinometerGraphicsViewRedrawCheckbox->isChecked())
            {
                MainWindow::getInstance()->xInclination->setRotation(180 * platformLocationP1->getAccelerometerNaturalReadings()[0]);
                MainWindow::getInstance()->yInclination->setRotation(180 * platformLocationP1->getAccelerometerNaturalReadings()[1]);
                MainWindow::getInstance()->zInclination->setRotation(180 * platformLocationP1->getAccelerometerNaturalReadings()[2]);
            }
        }

        static double prevMagX, prevMagY, prevMagZ;

        double MagX = platformLocationP1->getMagnetometerReadings()[0];
        double MagY = platformLocationP1->getMagnetometerReadings()[1];
        double MagZ = platformLocationP1->getMagnetometerReadings()[2];

        bool updateMag = false;

        if (prevMagX != MagX || prevMagY != MagY || prevMagZ != MagZ)
        {
            updateMag = true;
        }

        prevMagX = MagX;
        prevMagY = MagY;
        prevMagZ = MagZ;

        if (platformLocationP1->getMagnetometerWorkerActivated() || updateMag)
        {
            if (ui->magnetometerRawCheckBox->isChecked())
            {
                ui->magnetometerXLcdNumber->display(MagX);
                ui->magnetometerYLcdNumber->display(MagY);
                ui->magnetometerZLcdNumber->display(MagZ);
            }
            else
            {
                ui->magnetometerXLcdNumber->display(platformLocationP1->getMagnetometerNaturalReadings()[0]);
                ui->magnetometerYLcdNumber->display(platformLocationP1->getMagnetometerNaturalReadings()[1]);
                ui->magnetometerZLcdNumber->display(platformLocationP1->getMagnetometerNaturalReadings()[2]);
            }
        }

        if (platformLocationP1->getCompassHeadingWorkerActivated())
        {
            if (platformLocationP1->getCompassHeading())
            {
                ui->compassHeadingLcdNumber->display(platformLocationP1->getCompassHeading());
            }
            else
            {
                ui->compassHeadingLcdNumber->display("");
            }
        }
    }
}

void accelerometerRefreshGraphicsView()
{
    Ui::MainWindow *ui = MainWindow::getInstance()->getUi();
    if (ui->platformLocationP1RedrawGUICheckBox->isChecked()) //PLATFROM-LOCATION-P1 Tab
    {
        if (PlatformLocationP1::getInstance()->getAccelerometerWorkerActivated())
        {
            PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();

            static qreal x = 0.0;
            static qreal x1 = 0.0;

            static qreal yX = MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.2;
            static qreal y1X = 0.0;

            static qreal yY = MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.4;
            static qreal y1Y = 0.0;

            static qreal yZ = MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.6;
            static qreal y1Z = 0.0;

            x1 = x + 0.5;
            y1X = (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.2) + (double)(platformLocationP1->getAccelerometerNaturalReadings()[0]) * (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 7);
            y1Y = (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.4) + (double)(platformLocationP1->getAccelerometerNaturalReadings()[1]) * (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 7);
            y1Z = (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.6) + (double)(platformLocationP1->getAccelerometerNaturalReadings()[2]) * (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 7);

            MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->addLine(x, yX, x1, y1X, QPen(Qt::red));
            MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->addLine(x, yY, x1, y1Y, QPen(Qt::blue));
            MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->addLine(x, yZ, x1, y1Z, QPen(Qt::green));

            x = x1;
            yX = y1X;
            yY = y1Y;
            yZ = y1Z;

            if (x > MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->width())
            {
                MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->clear();
                x = 0.0;
            }
        }
    }
}

void magnetometerRefreshGraphicsView()
{
    Ui::MainWindow *ui = MainWindow::getInstance()->getUi();
    if (ui->platformLocationP1RedrawGUICheckBox->isChecked()) //PLATFROM-LOCATION-P1 Tab
    {
        if (PlatformLocationP1::getInstance()->getMagnetometerWorkerActivated())
        {
            PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();

            static qreal x = 0.0;
            static qreal x1 = 0.0;

            static qreal yX = MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->height() * 0.2;
            static qreal y1X = 0.0;

            static qreal yY = MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->height() * 0.4;
            static qreal y1Y = 0.0;

            static qreal yZ = MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->height() * 0.6;
            static qreal y1Z = 0.0;

            x1 = x + 0.5;
            y1X = (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.2) + (double)(platformLocationP1->getMagnetometerReadings()[0]) * (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 1000);
            y1Y = (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.4) + (double)(platformLocationP1->getMagnetometerReadings()[1]) * (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 1000);
            y1Z = (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() * 0.6) + (double)(platformLocationP1->getMagnetometerReadings()[2]) * (MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->height() / 1000);

            MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->addLine(x, yX, x1, y1X, QPen(Qt::red));
            MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->addLine(x, yY, x1, y1Y, QPen(Qt::blue));
            MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->addLine(x, yZ, x1, y1Z, QPen(Qt::green));

            x = x1;
            yX = y1X;
            yY = y1Y;
            yZ = y1Z;

            if (x > MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->width())
            {
                MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->clear();
                x = 0.0;
            }
        }
    }
}

void compassHeadingRefreshView()
{
    Ui::MainWindow *ui = MainWindow::getInstance()->getUi();
    if (ui->platformLocationP1RedrawGUICheckBox->isChecked()) //PLATFROM-LOCATION-P1 Tab
    {
        static int cnt = 0;
        static double x = 0;
        static double y = 0;

        PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();

        if (cnt > 5)
        {
            x = x / 5;
            y = y / 5;
            MainWindow::getInstance()->northDirection->setLine(180, 140, x, y);
            cnt = 0;
            x = 0;
            y = 0;
        }
        else
        {
            double endX = 180 + 140 * sin((180 - platformLocationP1->getHeading()) * M_PI / 180);
            double endY = 140 + 140 * cos((180 - platformLocationP1->getHeading()) * M_PI / 180);

            x += endX;
            y += endY;
        }
        cnt++;
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


void setAllLedsButtonsOn(MainWindow *mainWindow)
{
    mainWindow->getUi()->ch0RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch1RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch2RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch3RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch4RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch5RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch6RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch7RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch8RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch9RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch10RedLed->setIcon(mainWindow->redLedOnIcon);
    mainWindow->getUi()->ch11RedLed->setIcon(mainWindow->redLedOnIcon);

    mainWindow->getUi()->ch0GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch1GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch2GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch3GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch4GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch5GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch6GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch7GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch8GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch9GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch10GreenLed->setIcon(mainWindow->greenLedOnIcon);
    mainWindow->getUi()->ch11GreenLed->setIcon(mainWindow->greenLedOnIcon);
}

void setAllLedsButtonsOff(MainWindow *mainWindow)
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

void initLedButtons(MainWindow *mainWindow)
{
    setAllLedsButtonsOff(mainWindow);
}

void setRedLedButtonOn(QString buttonName, bool state)
{
    MainWindow *mainWindow = MainWindow::getInstance();
    Ui::MainWindow* ui = mainWindow->getUi();
    QPushButton *btn = ui->platformLocationP1Tab->findChild<QPushButton*>(buttonName, Qt::FindChildrenRecursively);
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

void setGreenLedButtonOn(QString buttonName, bool state)
{
    MainWindow *mainWindow = MainWindow::getInstance();
    Ui::MainWindow* ui = mainWindow->getUi();
    QPushButton *btn = ui->platformLocationP1Tab->findChild<QPushButton*>(buttonName, Qt::FindChildrenRecursively);
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
