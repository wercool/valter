#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<guihelpers.h>

#include <valter.h>

#include <platformcontrolp1GUI.h>
#include <platformlocationp1GUI.h>

MainWindow* MainWindow::pMainWindow = NULL;
bool MainWindow::instanceFlag = false;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    statusBarText = new QLabel();
    statusBar()->addWidget(statusBarText, 1);

    setWindowIcon(QIcon(":/valter_head_icon.png"));

    //controlDeviceTableWidget
    ui->controlDeviceTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->controlDeviceTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    QHeaderView* controlDeviceTableWidgetHeaderView = new QHeaderView(Qt::Horizontal);
    controlDeviceTableWidgetHeaderView->setStretchLastSection(true);
    controlDeviceTableWidgetHeaderView->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->controlDeviceTableWidget->setHorizontalHeader(controlDeviceTableWidgetHeaderView);

    //platfrom control p1
    ui->platformControlP1ReadingsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->platformControlP1ReadingsTable->setSelectionBehavior(QAbstractItemView::SelectItems);
    QHeaderView* platformControlP1ReadingsTableHeaderView = new QHeaderView(Qt::Horizontal);
    platformControlP1ReadingsTableHeaderView->setStretchLastSection(true);
    platformControlP1ReadingsTableHeaderView->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->platformControlP1ReadingsTable->setHorizontalHeader(platformControlP1ReadingsTableHeaderView);

    ui->platformControlP1WheelMotorsDutySlider->installEventFilter(new WheelEventFilter());
    ui->platformMovementDecelerationSlider->installEventFilter(new WheelEventFilter());
    ui->platformMovementAccelerationSlider->installEventFilter(new WheelEventFilter());
    ui->leftMotorPlatformControlP1DutySlider->installEventFilter(new WheelEventFilter());
    ui->rightMotorPlatformControlP1DutySlider->installEventFilter(new WheelEventFilter());
    ui->turretRotationDutySlider->installEventFilter(new WheelEventFilter());
    ui->decelerationTurretRotationSlider->installEventFilter(new WheelEventFilter());
    ui->accelerationTurretRotationSlider->installEventFilter(new WheelEventFilter());
    ui->platformControlP1MotorsPWMFrequencySpinBox->installEventFilter(new WheelEventFilter());

    logLength = 0;
    allConnect = true;

    controlDevicesDataExchangeLogTimer = new QTimer(this);
    connect(controlDevicesDataExchangeLogTimer, SIGNAL(timeout()), this, SLOT(controlDevicesDataExchangeLogTimerUpdate()));
    controlDevicesDataExchangeLogTimer->start(1);

    controlDevicesTableRefreshTimer = new QTimer(this);
    connect(controlDevicesTableRefreshTimer, SIGNAL(timeout()), this, SLOT(controlDevicesTableRefreshTimerUpdate()));
    controlDevicesTableRefreshTimer->start(1000);

    platformControlP1TabRefreshTimer = new QTimer(this);
    connect(platformControlP1TabRefreshTimer, SIGNAL(timeout()), this, SLOT(platformControlP1TabRefreshTimerUpdate()));
    platformControlP1TabRefreshTimer->start(100);

    delayedGUIActionsProcessingTimer = new QTimer(this);
    connect(delayedGUIActionsProcessingTimer, SIGNAL(timeout()), this, SLOT(delayedGUIActionsProcessingTimerUpdate()));
    delayedGUIActionsProcessingTimer->start(250);


    //platform location p1
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
    connect(platformLocationP1AccelerometerRefreshTimer, SIGNAL(timeout()), this, SLOT(platformLocationP1AccelerometerRefreshTimerTimerUpdate()));

}

MainWindow::~MainWindow()
{
    delete ui;
}

MainWindow *MainWindow::getInstance()
{
    if(!instanceFlag)
    {
        pMainWindow = new MainWindow();
        instanceFlag = true;
        return pMainWindow;
    }
    else
    {
        return pMainWindow;
    }
}

Ui::MainWindow *MainWindow::getUi() const
{
    return ui;
}

void MainWindow::setUi(Ui::MainWindow *value)
{
    ui = value;
}

void MainWindow::refreshControlDeviceTableWidget()
{
    refreshControlDeviceTableWorker(ui);
}

void MainWindow::addMsgToLog(string msg)
{
    if (ui->addTimeToLogCheckBox->isChecked())
    {
        timeval curTime;
        gettimeofday(&curTime, NULL);
        int milli = curTime.tv_usec / 1000;

        char buffer [80];
        strftime(buffer, 80, "%H:%M:%S", localtime(&curTime.tv_sec));

        char currentTime[84] = "";
        sprintf(currentTime, "%s:%03d", buffer, milli);

        msg = Valter::format_string("%s: %s", currentTime, msg.c_str());
    }

    ui->loggingTextEdit->moveCursor(QTextCursor::End);
    ui->loggingTextEdit->appendPlainText(msg.c_str());
    logLength++;
    if (ui->autoclearLogBox->isChecked())
    {
        if (logLength > logMaxLength)
        {
            ui->loggingTextEdit->clear();
            logLength = 0;
        }
    }

}

void MainWindow::delayGUIAction(IValterModule *valterModule)
{
    int action = valterModule->getActionFromDelayedGUIActions();

    //from PlatformControlP1
    if (valterModule->getControlDevice()->getControlDeviceId().compare(PlatformControlP1::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadPlatformControlP1Defaults(ui);
            break;
        }
    }
    //from PlatformLocationP1
    if (valterModule->getControlDevice()->getControlDeviceId().compare(PlatformLocationP1::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadPlatformLocationP1Defaults(ui);
            break;
        }
    }
}

void MainWindow::on_clearLogButton_clicked()
{
    if (ui->clearBufferCheckBox->isChecked())
    {
        map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
        typedef map<string, ControlDevice*>::iterator it_type;

        for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
        {
            ControlDevice *controlDevice = controlDevicesMap[iterator->first];
            controlDevice->clearDataExchangeLog();
        }
    }
    ui->loggingTextEdit->clear();
    statusBarText->setText("");
}


void MainWindow::on_scanControlDevicesBtn_clicked()
{
    ui->selectedControlDeviceListWidget->clear();
    ui->commandEdit->clear();
    selectedControlDeviceId = "";
    ui->connectAllPushButton->setText("Connect All");

    allConnect = true;

    Valter::getInstance()->closeAllControlDevicePorts();
    Valter::getInstance()->clearControlDevicesMap();
    Valter::getInstance()->scanControlDevices();
    refreshControlDeviceTableWidget();
}

void MainWindow::on_connectDisconnectControlDeviceButton_clicked()
{
    int selectedControlDeviceRowIndex = ui->controlDeviceTableWidget->selectionModel()->currentIndex().row();
    if (selectedControlDeviceRowIndex >= 0)
    {
        QTableWidgetItem *item = new QTableWidgetItem();
        item = ui->controlDeviceTableWidget->item(selectedControlDeviceRowIndex,1);
        string controlDeviceId = item->text().toStdString();

        ControlDevice *controlDevice = Valter::getInstance()->getControlDeviceById(controlDeviceId);
        IValterModule *valterModule = Valter::getInstance()->getValterModule(controlDeviceId);

        controlDevice->setResetWDTimer(true);

        if (controlDevice->getControlDevicePort()->isOpen())
        {
            controlDevice->deactivate();
        }
        else
        {
            controlDevice->activate();

            valterModule->loadDefaults();
            valterModule->addActionToDelayedGUIActions(IValterModule::RELOAD_DEFAULTS);
        }
    }
    refreshControlDeviceTableWidget();
    ui->controlDeviceTableWidget->selectRow(selectedControlDeviceRowIndex);
}

void MainWindow::on_controlDeviceTableWidget_clicked(const QModelIndex &index)
{
    QTableWidgetItem *selectedControlDeviceItem = new QTableWidgetItem();
    selectedControlDeviceItem = ui->controlDeviceTableWidget->item(index.row(), 1);
    selectedControlDeviceId = selectedControlDeviceItem->text().toStdString();

    ui->selectedControlDeviceListWidget->clear();
    ui->commandEdit->clear();

    map<string, vector<string>> controlDevicesCommands = Valter::getInstance()->getControlDevicesCommands();
    vector<string> controlDeviceCommands = controlDevicesCommands[selectedControlDeviceId];

    for(vector<string>::size_type i = 0; i != controlDeviceCommands.size(); i++)
    {
        QListWidgetItem *item = new QListWidgetItem;
        item->setText(((string)controlDeviceCommands[i]).c_str());
        ui->selectedControlDeviceListWidget->insertItem(i, item);
    }
}

void MainWindow::on_selectedControlDeviceListWidget_doubleClicked(const QModelIndex &index)
{
    QListWidgetItem *selectedCommandsItem= new QListWidgetItem;
    selectedCommandsItem = ui->selectedControlDeviceListWidget->item(index.row());
    string selectedCommand = selectedCommandsItem->text().toStdString();

    ui->commandEdit->setText(selectedCommand.c_str());
}

void MainWindow::on_clearCommandButton_clicked()
{
    ui->commandEdit->clear();
}

void MainWindow::on_pauseLoggingButton_clicked()
{
    if (Valter::getInstance()->getLogControlDeviceMessages())
    {
        map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
        typedef map<string, ControlDevice*>::iterator it_type;

        for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
        {
            ControlDevice *controlDevice = controlDevicesMap[iterator->first];
            controlDevice->clearDataExchangeLog();
        }
        Valter::getInstance()->setLogControlDeviceMessages(false);
        ui->pauseLoggingButton->setText("Resume");
    }
    else
    {
        Valter::getInstance()->setLogControlDeviceMessages(true);
        ui->pauseLoggingButton->setText("Pause");
    }
}

void MainWindow::on_sendCommandButton_clicked()
{
    if (ui->commandEdit->text() != "" || !ui->selectedControlDeviceListWidget->selectedItems().isEmpty())
    {
        ControlDevice *controlDevice = Valter::getInstance()->getControlDeviceById(selectedControlDeviceId);
        if (ui->commandEdit->text() != "")
        {
            string request = ui->commandEdit->text().toStdString();
            controlDevice->addRequest(request);
        }
        else
        {
            if (!ui->selectedControlDeviceListWidget->selectedItems().isEmpty())
            {
                QListWidgetItem *selectedCommandsItem = new QListWidgetItem;
                selectedCommandsItem = ui->selectedControlDeviceListWidget->selectedItems().first();
                string selectedCommand = selectedCommandsItem->text().toStdString();
                controlDevice->addRequest(selectedCommand);
            }
        }
    }
}

void MainWindow::controlDevicesDataExchangeLogTimerUpdate()
{
    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    if (ui->logSelectedOnlyCheckBox->isChecked())
    {
        if (selectedControlDeviceId.length() > 0)
        {
            ControlDevice *controlDevice = controlDevicesMap[selectedControlDeviceId];
            if (controlDevice->dataExchangeLogAvailable() > 0)
            {
                string logMsg = controlDevice->getMsgFromDataExchangeLog();
                Valter::log(logMsg);
                if (ui->logToStatusBarCheckBox->isChecked())
                {
                    statusBarText->setText(logMsg.c_str());
                }
                if (ui->autoclearLogBox->isChecked())
                {
                    if (controlDevice->dataExchangeLogAvailable() > ControlDevice::maxLogBufferSize)
                    {
                        controlDevice->clearDataExchangeLog();
                    }
                }
            }
        }
    }
    else
    {
        for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
        {
            ControlDevice *controlDevice = controlDevicesMap[iterator->first];
            if (controlDevice->dataExchangeLogAvailable() > 0)
            {
                string logMsg = controlDevice->getMsgFromDataExchangeLog();
                Valter::log(logMsg);
                statusBarText->setText(logMsg.c_str());
                if (ui->autoclearLogBox->isChecked())
                {
                    if (controlDevice->dataExchangeLogAvailable() > ControlDevice::maxLogBufferSize)
                    {
                        controlDevice->clearDataExchangeLog();
                    }
                }
            }
        }
    }
}

void MainWindow::controlDevicesTableRefreshTimerUpdate()
{
    controlDevicesTableRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_connectAllPushButton_clicked()
{
    int selectedControlDeviceRowIndex = ui->controlDeviceTableWidget->selectionModel()->currentIndex().row();
    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        if (allConnect)
        {
            if (!controlDevice->getControlDevicePort()->isOpen())
            {
                controlDevice->activate();

                IValterModule *valterModule = Valter::getInstance()->getValterModule(controlDevice->getControlDeviceId());
                valterModule->loadDefaults();
                valterModule->addActionToDelayedGUIActions(IValterModule::RELOAD_DEFAULTS);
            }
        }
        else
        {
            if (controlDevice->getControlDevicePort()->isOpen())
            {
                controlDevice->deactivate();
            }
        }
    }
    if (allConnect)
    {
        ui->connectAllPushButton->setText("Disconnect All");
        allConnect = false;
    }
    else
    {
        ui->connectAllPushButton->setText("Connect All");
        allConnect = true;
    }
    refreshControlDeviceTableWidget();
    if (selectedControlDeviceRowIndex >= 0)
    {
        ui->controlDeviceTableWidget->selectRow(selectedControlDeviceRowIndex);
    }
}

void MainWindow::on_wdResetStopButton_clicked()
{
    if (selectedControlDeviceId.length() > 0)
    {
        map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
        ControlDevice *controlDevice = controlDevicesMap[selectedControlDeviceId];
        Valter::log(Valter::format_string("Stop WDR signal is sent to %s", controlDevice->getControlDeviceId().c_str()));
        controlDevice->setResetWDTimer(false);
    }
}

void MainWindow::on_reScanControlDevicesButton_clicked()
{
    if (selectedControlDeviceId.length() > 0)
    {
        map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
        ControlDevice *controlDevice = controlDevicesMap[selectedControlDeviceId];
        controlDevice->reScanThisControlDevice();
    }
}

void MainWindow::on_wdResetOnButton_clicked()
{
    if (selectedControlDeviceId.length() > 0)
    {
        map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
        ControlDevice *controlDevice = controlDevicesMap[selectedControlDeviceId];
        Valter::log(Valter::format_string("Start WDR signal is sent to %s", controlDevice->getControlDeviceId().c_str()));
        controlDevice->setResetWDTimer(true);
    }
}

void MainWindow::on_stopAllPlatformControlP1Button_clicked()
{
    qDebug("STOP ALL at PlatformControlP1");
    Valter::getInstance()->stopAllModules();
}

void MainWindow::delayedGUIActionsProcessingTimerUpdate()
{
    map<string, IValterModule*> valterModulesMap = Valter::getInstance()->getValterModulesMap();
    typedef map<string, IValterModule*>::iterator it_type;

    for(it_type iterator = valterModulesMap.begin(); iterator != valterModulesMap.end(); iterator++)
    {
        IValterModule *valterModule = valterModulesMap[iterator->first];
        if (valterModule->areActionsInDelayedGUIActions())
        {
            delayGUIAction(valterModule);
        }
    }
}

void MainWindow::platformControlP1TabRefreshTimerUpdate()
{
    platformControlP1TabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_on5VPlatformControlP1pushButton_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("DCDC5VENABLEON");
}

void MainWindow::on_off5VPlatformControlP1pushButton_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("DCDC5VENABLEOFF");
}

void MainWindow::on_onLeftAccumulatorPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("LEFTACCUMULATORCONNECT");
}

void MainWindow::on_offLeftAccumulatorPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("LEFTACCUMULATORDISCONNECT");
}

void MainWindow::on_onRightAccumulatorPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("RIGHTACCUMULATORCONNECT");
}

void MainWindow::on_offRightAccumulatorPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("RIGHTACCUMULATORDISCONNECT");
}

void MainWindow::on_scan220VAOnCButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setScan220ACAvailable(true);
}

void MainWindow::on_scan220VAOffCButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setScan220ACAvailable(false);
}

void MainWindow::on_onMainAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (!platformControlP1->mainAccumulatorON())
    {
        QMessageBox msgBox;
        msgBox.setText("220V AC is not connected");
        msgBox.exec();
    }
}

void MainWindow::on_offMainAccumulatorRelayPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("MAINACCUMULATORRELAYOFF");
}

void MainWindow::on_onLeftAccumulatorRelayPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("LEFTACCUMULATORRELAYON");
}

void MainWindow::on_offLeftAccumulatorRelayPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("LEFTACCUMULATORRELAYOFF");
}

void MainWindow::on_onRightAccumulatorRelayPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("RIGHTACCUMULATORRELAYON");
}

void MainWindow::on_offRightAccumulatorRelayPlatformControlP1Button_clicked()
{
    IValterModule *valterModule = Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    valterModule->sendCommand("RIGHTACCUMULATORRELAYOFF");
}

void MainWindow::on_chargerButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->chargerButtonPress();
}

void MainWindow::on_setChargeOnButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setChargerMode(true);
}

void MainWindow::on_setChargeOffButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setChargerMode(false);
}

void MainWindow::on_platformMoveStopButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setPlatformEmergencyStop(true);
}

void MainWindow::on_platformMoveForwardButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //left and right forward
        if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(true))
        {
            platformControlP1->setLeftMotorActivated(true);
            platformControlP1->setRightMotorActivated(true);
        }
    }
}

void MainWindow::on_platformMoveForwardButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformBackwardForwardButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //left and right backward
        if (platformControlP1->setLeftMotorDirection(false) && platformControlP1->setRightMotorDirection(false))
        {
            platformControlP1->setLeftMotorActivated(true);
            platformControlP1->setRightMotorActivated(true);
        }
    }
}

void MainWindow::on_platformBackwardForwardButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformMoveForwardLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //left forward
        if (platformControlP1->setLeftMotorDirection(true))
        {
            platformControlP1->setLeftMotorActivated(true);
        }
    }
}

void MainWindow::on_platformMoveForwardLeftButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorActivated(false);
}

void MainWindow::on_platformMoveForwardRightButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //right forward
        if (platformControlP1->setRightMotorDirection(true))
        {
            platformControlP1->setRightMotorActivated(true);
        }
    }
}

void MainWindow::on_platformMoveForwardRightButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformMoveBackwardLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //left backward
        if (platformControlP1->setLeftMotorDirection(false))
        {
            platformControlP1->setLeftMotorActivated(true);
        }
    }
}

void MainWindow::on_platformMoveBackwardRightButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //right backward
        if (platformControlP1->setRightMotorDirection(false))
        {
            platformControlP1->setRightMotorActivated(true);
        }
    }
}

void MainWindow::on_platformMoveBackwardLeftButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorActivated(false);
}

void MainWindow::on_platformMoveBackwardRightButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformRotateLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //left backward right forward
        if (platformControlP1->setLeftMotorDirection(false) && platformControlP1->setRightMotorDirection(true))
        {
            platformControlP1->setLeftMotorActivated(true);
            platformControlP1->setRightMotorActivated(true);
        }
    }
}

void MainWindow::on_platformRotateRightButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //left forward right backward
        if (platformControlP1->setLeftMotorDirection(true) && platformControlP1->setRightMotorDirection(false))
        {
            platformControlP1->setLeftMotorActivated(true);
            platformControlP1->setRightMotorActivated(true);
        }
    }
}

void MainWindow::on_platformRotateLeftButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformRotateRightButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_turretRotateLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //rotate left (ccw)
        if (platformControlP1->setTurretMotorDirection(false))
        {
            platformControlP1->setTurretMotorActivated(true);
        }
    }
}

void MainWindow::on_turretRotateLeftButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretMotorActivated(false);
}

void MainWindow::on_turretRotateRightButton_pressed()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (platformControlP1->preparePlatformMovement())
    {
        //rotate right (cw)
        if (platformControlP1->setTurretMotorDirection(true))
        {
            platformControlP1->setTurretMotorActivated(true);
        }
    }
}

void MainWindow::on_bodyRotationStopButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretEmergencyStop(true);
}

void MainWindow::on_turretRotateRightButton_released()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretMotorActivated(false);
}

void MainWindow::on_platformControlP1LoadDefaultsButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->loadDefaults();
    loadPlatformControlP1Defaults(ui);
}

void MainWindow::on_platformControlP1MotorsPWMFrequencySetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setMotorsPWMFrequncy(ui->platformControlP1MotorsPWMFrequencySpinBox->value());
}

void MainWindow::on_leftMotorCurrentCheckBox_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorCurrentRead(ui->leftMotorCurrentCheckBox->isChecked());
}

void MainWindow::on_rightMotorCurrentCheckBox_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setRightMotorCurrentRead(ui->rightMotorCurrentCheckBox->isChecked());
}

void MainWindow::on_turretMotorCurrentCheckBox_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretMotorCurrentRead(ui->turretMotorCurrentCheckBox->isChecked());
}

void MainWindow::on_platformControlP1LeftWheelEncoderResetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->resetLeftWheelEncoder();
    platformControlP1->setLeftWheelEncoder(0);
}

void MainWindow::on_platformControlP1RightWheelEncoderResetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->resetRightWheelEncoder();
    platformControlP1->setRightWheelEncoder(0);
}

void MainWindow::on_platformControlP1LeftWheelEncoderGetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    ui->platformControlP1LeftWheelEncoderLcdNumber->display("");
    platformControlP1->setLeftWheelEncoderGetOnce(true);
    platformControlP1->sendCommand("GETLEFTMOTORCOUNTER");
}

void MainWindow::on_platformControlP1RightWheelEncoderGetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    ui->platformControlP1RightWheelEncoderLcdNumber->display("");
    platformControlP1->setRightWheelEncoderGetOnce(true);
    platformControlP1->sendCommand("GETRIGHTMOTORCOUNTER");
}

void MainWindow::on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftWheelEncoderAutoreset(checked);
}

void MainWindow::on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setRightWheelEncoderAutoreset(checked);
}

void MainWindow::on_pushButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    ui->turretPositionLcdNumber->display("");
    platformControlP1->setTurretPositionGetOnce(true);
    platformControlP1->sendCommand("GETTURRETPOSITION");
}

void MainWindow::on_platformControlP1ReadingsTable_itemClicked(QTableWidgetItem *item)
{
    setPlatfromControlP1AdditionalReadings(item);
}

void MainWindow::on_leftMotorPlatformControlP1DutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorDutyMax(value);
    ui->leftMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getLeftMotorDutyMax()).c_str());
}

void MainWindow::on_rightMotorPlatformControlP1DutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setRightMotorDutyMax(value);
    ui->rightMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getRightMotorDutyMax()).c_str());
}

void MainWindow::on_platformMovementAccelerationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setPlatformAcceleration(value);
    ui->platformControlP1WheelMotorsAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_platformMovementDecelerationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setPlatformDeceleration(value);
    ui->platformControlP1WheelMotorsDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_platformControlP1WheelMotorsDutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    static int prevDutyVal = 1;
    int maxDutyLeft = platformControlP1->getLeftMotorDutyMax();
    int maxDutyRight = platformControlP1->getRightMotorDutyMax();

    if(value > prevDutyVal)
    {
        if (maxDutyLeft < 100)
            maxDutyLeft++;
        if (maxDutyRight < 100)
            maxDutyRight++;
    }
    else
    {
        if (maxDutyLeft > 1)
            maxDutyLeft--;
        if (maxDutyRight > 1)
            maxDutyRight--;
    }
    platformControlP1->setLeftMotorDutyMax(maxDutyLeft);
    platformControlP1->setRightMotorDutyMax(maxDutyRight);

    ui->leftMotorPlatformControlP1DutySlider->setValue(platformControlP1->getLeftMotorDutyMax());
    ui->rightMotorPlatformControlP1DutySlider->setValue(platformControlP1->getRightMotorDutyMax());

    prevDutyVal = value;
}

void MainWindow::on_turretRotationDutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretMotorDutyMax(value);
    ui->turretMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getTurretMotorDutyMax()).c_str());
}

void MainWindow::on_decelerationTurretRotationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretDeceleration(value);
    ui->platformControlP1TurretMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_accelerationTurretRotationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretAcceleration(value);
    ui->platformControlP1TurretMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_platformControlP1additionalReadingsTrackingDelay_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setAdditionalReadingsDelayCur(value);
    ui->platformControlP1additionalReadingsTrackingDelayLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_mainTabWidget_tabBarDoubleClicked(int index)
{
    QWidget* pWidget = ui->mainTabWidget->widget(index);
    pWidget->installEventFilter(new GenericEventFilter(ui, ui->mainTabWidget->tabText(index), index));
    pWidget = ui->mainTabWidget->widget(index);
    pWidget->setWindowTitle(ui->mainTabWidget->tabText(index));
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
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
        platformLocationP1AccelerometerRefreshTimer->start(5);
    }
    else
    {
        platformLocationP1AccelerometerRefreshTimer->stop();
    }
}

void MainWindow::platformLocationP1AccelerometerRefreshTimerTimerUpdate()
{
    accelerometerRefreshGraphicsView();
}

void MainWindow::on_detatchSonarsFrameButton_2_clicked()
{
    QWidget* pWidget = ui->accAndMagFrame;
    pWidget->installEventFilter(new ACCAndMAGFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Accelerometer and Magnetometer");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}
