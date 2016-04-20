#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/platformlocationp1GUI.h>


void MainWindow::initPlatfromLocationP1(Ui::MainWindow *ui)
{
    platformLocationP1TabRefreshTimer = new QTimer(this);
    connect(platformLocationP1TabRefreshTimer, SIGNAL(timeout()), this, SLOT(platformLocationP1TabRefreshTimerUpdate()));
    platformLocationP1TabRefreshTimer->start(100);

    redLedOffPix = QPixmap(":/red-led-off.png");
    redLedOffIcon = QIcon(redLedOffPix);
    redLedOnPix = QPixmap(":/red-led-on.png");
    redLedOnIcon = QIcon(redLedOnPix);
    greenLedOffPix = QPixmap(":/green-led-off.png");
    greenLedOffIcon = QIcon(greenLedOffPix);
    greenLedOnPix = QPixmap(":/green-led-on.png");
    greenLedOnIcon = QIcon(greenLedOnPix);
    initLedButtons(this);

    connect(ui->ch0RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch1RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch2RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch3RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch4RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch5RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch6RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch7RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch8RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch9RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch10RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch11RedLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));

    connect(ui->ch0GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch1GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch2GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch3GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch4GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch5GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch6GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch7GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch8GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch9GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch10GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));
    connect(ui->ch11GreenLed, SIGNAL(clicked()), this, SLOT (platfromLocationP1LEDHandler()));

    ui->USSignalDutyScroller->installEventFilter(new WheelEventFilter());
    ui->USSignalBurstScroller->installEventFilter(new WheelEventFilter());
    ui->USSignalDelayScroller->installEventFilter(new WheelEventFilter());
    ui->leftSonarAngleScroller->installEventFilter(new WheelEventFilter());
    ui->rightSonarAngleScroller->installEventFilter(new WheelEventFilter());

    platformLocationP1SonarsGraphicsViewScene = new QGraphicsScene;
    platformLocationP1SonarsGraphicsViewScene->setSceneRect(0, 0, 790, 400);
    ui->platformLocationP1GraphicsView->setScene(platformLocationP1SonarsGraphicsViewScene);

    leftUSSonarVector = new QGraphicsLineItem;
    leftUSSonarVector->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
    leftUSSonarVector->setLine(345, 285, 345, 50);
    platformLocationP1SonarsGraphicsViewScene->addItem(leftUSSonarVector);

    rightUSSonarVector = new QGraphicsLineItem;
    rightUSSonarVector->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
    rightUSSonarVector->setLine(445, 285, 445, 50);
    platformLocationP1SonarsGraphicsViewScene->addItem(rightUSSonarVector);

    //accelerometer
    platformLocationP1AccelerometerGraphicsViewScene = new QGraphicsScene;
    platformLocationP1AccelerometerGraphicsViewScene->setSceneRect(0, 0, 370, 290);
    ui->accelerometerGraphicsView->setScene(platformLocationP1AccelerometerGraphicsViewScene);

    platformLocationP1AccelerometerRefreshTimer = new QTimer(this);
    connect(platformLocationP1AccelerometerRefreshTimer, SIGNAL(timeout()), this, SLOT(platformLocationP1AccelerometerRefreshTimerUpdate()));

    //magnetometer
    platformLocationP1MagnetometerGraphicsViewScene = new QGraphicsScene;
    platformLocationP1MagnetometerGraphicsViewScene->setSceneRect(0, 0, 370, 290);
    ui->magnetometerGraphicsView->setScene(platformLocationP1MagnetometerGraphicsViewScene);

    platformLocationP1MagnetometerRefreshTimer = new QTimer(this);
    connect(platformLocationP1MagnetometerRefreshTimer, SIGNAL(timeout()), this, SLOT(platformLocationP1MagnetometerRefreshTimerUpdate()));

    platformLocationP1CompassHeadingRefreshTimer = new QTimer(this);
    connect(platformLocationP1CompassHeadingRefreshTimer, SIGNAL(timeout()), this, SLOT(platformLocationP1CompassHeadingRefreshTimerUpdate()));


    //compass
    platformLocationP1CompassGraphicsViewScene = new QGraphicsScene;
    platformLocationP1CompassGraphicsViewScene->setSceneRect(0, 0, 360, 280);
    ui->compassGraphicsView->setScene(platformLocationP1CompassGraphicsViewScene);

    QGraphicsLineItem *northDirectionPointer = new QGraphicsLineItem;
    northDirectionPointer->setPen(QPen(Qt::blue, 3.0, Qt::SolidLine));
    northDirectionPointer->setLine(180, 140, 180, 5);
    platformLocationP1CompassGraphicsViewScene->addItem(northDirectionPointer);

    northDirection = new QGraphicsLineItem;
    northDirection->setPen(QPen(Qt::blue, 2.0, Qt::DotLine));
    northDirection->setLine(180, 140, 180, 20);
    platformLocationP1CompassGraphicsViewScene->addItem(northDirection);

    //inclinometer
    platformLocationP1InclinometerGraphicsViewScene = new QGraphicsScene;
    platformLocationP1InclinometerGraphicsViewScene->setSceneRect(0, 0, 360, 300);
    ui->inclinometerGraphicsView->setScene(platformLocationP1InclinometerGraphicsViewScene);

    xInclination = new QGraphicsLineItem;
    xInclination->setPen(QPen(Qt::red, 5.0, Qt::SolidLine));
    xInclination->setLine(100, 72, 260, 72);
    platformLocationP1InclinometerGraphicsViewScene->addItem(xInclination);
    xInclination->setTransformOriginPoint(180, 72);
    xInclination->setRotation(0);

    QGraphicsEllipseItem *xInclinationCenter = new QGraphicsEllipseItem;
    xInclinationCenter->setRect(175, 67, 10, 10);
    platformLocationP1InclinometerGraphicsViewScene->addItem(xInclinationCenter);

    yInclination = new QGraphicsLineItem;
    yInclination->setPen(QPen(Qt::blue, 5.0, Qt::SolidLine));
    yInclination->setLine(100, 144, 260, 144);
    platformLocationP1InclinometerGraphicsViewScene->addItem(yInclination);
    yInclination->setTransformOriginPoint(180, 144);
    yInclination->setRotation(0);

    QGraphicsEllipseItem *yInclinationCenter = new QGraphicsEllipseItem;
    yInclinationCenter->setRect(175, 139, 10, 10);
    platformLocationP1InclinometerGraphicsViewScene->addItem(yInclinationCenter);

    zInclination = new QGraphicsLineItem;;
    zInclination->setPen(QPen(Qt::green, 5.0, Qt::SolidLine));
    zInclination->setLine(100, 216, 260, 216);
    platformLocationP1InclinometerGraphicsViewScene->addItem(zInclination);
    zInclination->setTransformOriginPoint(180, 216);
    zInclination->setRotation(0);

    QGraphicsEllipseItem *zInclinationCenter = new QGraphicsEllipseItem;
    zInclinationCenter->setRect(175, 211, 10, 10);
    platformLocationP1InclinometerGraphicsViewScene->addItem(zInclinationCenter);
}

//PLATFORM-LOCATION-P1

void MainWindow::platformLocationP1TabRefreshTimerUpdate()
{
    platformLocationP1TabRefreshTimerUpdateWorker(ui);
}

void MainWindow::platfromLocationP1LEDHandler()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    QPushButton* btn = (QPushButton*) sender();
    int channel = atoi(btn->objectName().toStdString().substr(2, (btn->objectName().length() - 8)).c_str());
    if (btn->objectName().endsWith("RedLed"))
    {
        platformLocationP1->setRedLedState(channel, !platformLocationP1->getRedLedState(channel));
        setRedLedButtonOn(this, btn, platformLocationP1->getRedLedState(channel));
    }
    else if (btn->objectName().endsWith("GreenLed"))
    {
        platformLocationP1->setGreenLedState(channel, !platformLocationP1->getGreenLedState(channel));
        setGreenLedButtonOn(this, btn, platformLocationP1->getGreenLedState(channel));
    }
}

void MainWindow::on_loadDefaultsPlatformLocationP1Button_clicked()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->loadDefaults();
    loadPlatformLocationP1Defaults(ui);
}

void MainWindow::on_irSensorsPresetsTable_itemClicked(QTableWidgetItem *item)
{
    setPlatformLocationIRSensorPresets(item);
}

void MainWindow::on_usSensorsPresetsTable_itemClicked(QTableWidgetItem *item)
{
    setPlatformLocationUSSensorPresets(item);
}

void MainWindow::on_LEDStatesButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setLedStatesSet(checked);
}

void MainWindow::on_USSignalDutyScroller_valueChanged(int value)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setUsSignalDuty(value);

    ui->USSignalDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_USSignalBurstScroller_valueChanged(int value)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setUsSignalBurst(value);

    ui->USSignalBurstLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_USSignalDelayScroller_valueChanged(int value)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setUsSignalDelay(value);

    ui->USSignalDelayLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_USVoltageUpButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRelativeUSSensorVoltageUp();
    ui->USVoltageRegulatorLabel->setText(Valter::format_string("[%d] relative to initial", platformLocationP1->getRelativeUSSensorVoltage()).c_str());
}

void MainWindow::on_USVoltageDownButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRelativeUSSensorVoltageDown();
    ui->USVoltageRegulatorLabel->setText(Valter::format_string("[%d] relative to initial", platformLocationP1->getRelativeUSSensorVoltage()).c_str());
}

void MainWindow::on_leftSonarScanButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setLeftSonarActivated(checked);
    if (!platformLocationP1->getLeftSonarActivated())
    {
        typedef map<int, QGraphicsEllipseItem*>::iterator it_type;
        for(it_type iterator = leftSonarDots.begin(); iterator != leftSonarDots.end(); iterator++)
        {
            QGraphicsEllipseItem* dot = leftSonarDots[iterator->first];
            if (dot)
            {
                dot->setPen(QPen(Qt::lightGray));
            }
        }
    }
}

void MainWindow::on_rightSonarScanButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRightSonarActivated(checked);
    if (!platformLocationP1->getRightSonarActivated())
    {
        typedef map<int, QGraphicsEllipseItem*>::iterator it_type;
        for(it_type iterator = rightSonarDots.begin(); iterator != rightSonarDots.end(); iterator++)
        {
            QGraphicsEllipseItem* dot = rightSonarDots[iterator->first];
            if (dot)
            {
                dot->setPen(QPen(Qt::lightGray));
            }
        }
    }
}

void MainWindow::on_leftSonarAngleScroller_valueChanged(int value)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    if (!platformLocationP1->getLeftSonarActivated())
    {
        platformLocationP1->setLeftSonarAngle(value);
    }

    ui->leftSonarAngleLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightSonarAngleScroller_valueChanged(int value)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    if (!platformLocationP1->getRightSonarActivated())
    {
        platformLocationP1->setRightSonarAngle(value);
    }

    ui->rightSonarAngleLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftSonarAngleScroller_sliderPressed()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setLeftSonarIntentionalAngleSet(true);
}

void MainWindow::on_rightSonarAngleScroller_sliderPressed()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRightSonarIntentionalAngleSet(true);
}

void MainWindow::on_leftSonarAngleScroller_sliderReleased()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setLeftSonarIntentionalAngleSet(false);
}

void MainWindow::on_rightSonarAngleScroller_sliderReleased()
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRightSonarIntentionalAngleSet(false);
}

void MainWindow::on_detatchSonarsFrameButton_clicked()
{
    QWidget* pWidget = ui->platfromLocationP1FronSonarsFrame;
    pWidget->installEventFilter(new FrontSonarsFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Front Sonars");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}

void MainWindow::on_accelerometerTrackCheckBox_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setAccelerometerWorkerActivated(checked);
}

void MainWindow::on_magnetometerTrackCheckBox_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setMagnetometerWorkerActivated(checked);
}

void MainWindow::on_accelerometerGraphicsViewRedrawCheckbox_toggled(bool checked)
{
    if (checked)
    {
        MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->clear();
        platformLocationP1AccelerometerRefreshTimer->start(20);
    }
    else
    {
        platformLocationP1AccelerometerRefreshTimer->stop();
    }
}

void MainWindow::platformLocationP1AccelerometerRefreshTimerUpdate()
{
    accelerometerRefreshGraphicsView();
}

void MainWindow::platformLocationP1MagnetometerRefreshTimerUpdate()
{
    magnetometerRefreshGraphicsView();
}

void MainWindow::on_detatchAccAndMagFrameButton_clicked()
{
    QWidget* pWidget = ui->accAndMagFrame;
    pWidget->installEventFilter(new ACCAndMAGFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Accelerometer and Magnetometer");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
    MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->clear();
}

void MainWindow::on_magnetometerGraphicsViewRedrawCheckbox_toggled(bool checked)
{
    if (checked)
    {
        MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->clear();
        platformLocationP1MagnetometerRefreshTimer->start(20);
    }
    else
    {
        platformLocationP1MagnetometerRefreshTimer->stop();
    }
}

void MainWindow::on_compassHeadingTrackCheckBox_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setCompassHeadingWorkerActivated(checked);
}

void MainWindow::platformLocationP1CompassHeadingRefreshTimerUpdate()
{
    compassHeadingRefreshView();
}

void MainWindow::on_compassGraphicsViewRedrawCheckbox_toggled(bool checked)
{
    if (checked)
    {
        platformLocationP1CompassHeadingRefreshTimer->start(20);
    }
    else
    {
        platformLocationP1CompassHeadingRefreshTimer->stop();
    }
}

void MainWindow::on_inclinometerXCheckbox_toggled(bool checked)
{
    if (!checked)
    {
        xInclination->setOpacity(0.05);
    }
    else
    {
        xInclination->setOpacity(1.0);
    }
}

void MainWindow::on_inclinometerYCheckbox_toggled(bool checked)
{
    if (!checked)
    {
        yInclination->setOpacity(0.05);
    }
    else
    {
        yInclination->setOpacity(1.0);
    }
}

void MainWindow::on_inclinometerZCheckbox_toggled(bool checked)
{
    if (!checked)
    {
        zInclination->setOpacity(0.05);
    }
    else
    {
        zInclination->setOpacity(1.0);
    }
}

void MainWindow::on_llLedToggleButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setLLLedState(checked);
}

void MainWindow::on_lrLedToggleButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setLRLedState(checked);
}

void MainWindow::on_rlLedToggleButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRLLedState(checked);
}

void MainWindow::on_rrLedToggleButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setRRLedState(checked);
}

void MainWindow::on_allSonarsLedsToggleButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setAllSonarsLedsState(checked);
    ui->llLedToggleButton->setChecked(checked);
    ui->lrLedToggleButton->setChecked(checked);
    ui->rlLedToggleButton->setChecked(checked);
    ui->rrLedToggleButton->setChecked(checked);
}

void MainWindow::on_manLedToggleButton_toggled(bool checked)
{
    PlatformLocationP1 *platformLocationP1 = (PlatformLocationP1*)Valter::getInstance()->getValterModule(PlatformLocationP1::getControlDeviceId());
    platformLocationP1->setManLedState(checked);
}

void MainWindow::on_detatchCompassFrameButton_clicked()
{
    QWidget* pWidget = ui->compassFrame;
    pWidget->installEventFilter(new CompassFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Compass");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}

void MainWindow::on_inclinometerFrameDetatchButton_clicked()
{
    QWidget* pWidget = ui->inclinometerFrame;
    pWidget->installEventFilter(new InclinometerFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Inclinomter");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}

void MainWindow::on_platformLocationP1EnableSensorsButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->enableSensors();
}

void MainWindow::on_platformLocationP1DisableSensorsButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->disableSensors();
}

void MainWindow::on_platformLocationP1AllLEDsONButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->setAllLEDsOn();
    setAllLedsButtonsOn(this);
}

void MainWindow::on_platformLocationP1AllLEDsOFFButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->setAllLEDsOff();
    setAllLedsButtonsOff(this);
}

void MainWindow::on_leftSonarReleaseButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->setLeftSonarActivated(false);
}

void MainWindow::on_rightSonarReleaseButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->setRightSonarActivated(false);
}

void MainWindow::on_updateCompassHeadingButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->updateCompassHeading();
}


void MainWindow::on_updateAccelerometerButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->updateAccelerometer();
}

void MainWindow::on_updateMagnetometerButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->updateMagnetometer();
}


void MainWindow::on_enableAllIRSensorsButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    for (unsigned int i = 0; i < 12; i++)
    {
        platformLocationP1->setReadIRSensor(i, true);
        ((QTableWidgetItem*)ui->irSensorsPresetsTable->item(i, 1))->setCheckState((platformLocationP1->getReadIRSensor(i)) ? Qt::Checked : Qt::Unchecked);
    }
}

void MainWindow::on_disableAllIRSensorsButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    for (unsigned int i = 0; i < 12; i++)
    {
        platformLocationP1->setReadIRSensor(i, false);
        ((QTableWidgetItem*)ui->irSensorsPresetsTable->item(i, 1))->setCheckState((platformLocationP1->getReadIRSensor(i)) ? Qt::Checked : Qt::Unchecked);
    }
}

void MainWindow::on_enableAllUSSensorsButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    for (unsigned int i = 0; i < 12; i++)
    {
        platformLocationP1->setReadUSSensor(i, true);
        ((QTableWidgetItem*)ui->usSensorsPresetsTable->item(i, 1))->setCheckState((platformLocationP1->getReadUSSensor(i)) ? Qt::Checked : Qt::Unchecked);
    }
}

void MainWindow::on_disableAllUSSensorsButton_clicked()
{
    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    for (unsigned int i = 0; i < 12; i++)
    {
        platformLocationP1->setReadUSSensor(i, false);
        ((QTableWidgetItem*)ui->usSensorsPresetsTable->item(i, 1))->setCheckState((platformLocationP1->getReadUSSensor(i)) ? Qt::Checked : Qt::Unchecked);
    }
}
