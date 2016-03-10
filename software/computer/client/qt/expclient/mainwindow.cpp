#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>
#include <mainwindowutils.h>

#include <valter.h>

MainWindow* MainWindow::pMainWindow = NULL;
bool MainWindow::instanceFlag = false;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    statusBarText = new QLabel();
    statusBar()->addWidget(statusBarText, 1);

    setWindowIcon(QIcon(":/valter_head_icon.png"));

    //controlDeviceTableWidget
    ui->controlDeviceTableWidget->setColumnCount(5);
    ui->controlDeviceTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->controlDeviceTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("Device File Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Id"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("WDR Intent"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(4, new QTableWidgetItem("System Device Path"));
    QHeaderView* controlDeviceTableWidgetHeaderView = new QHeaderView(Qt::Horizontal);
    controlDeviceTableWidgetHeaderView->setStretchLastSection(true);
    controlDeviceTableWidgetHeaderView->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->controlDeviceTableWidget->setHorizontalHeader(controlDeviceTableWidgetHeaderView);

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
    platformControlP1TabRefreshTimer->start(10);

    delayedGUIActionsProcessingTimer = new QTimer(this);
    connect(delayedGUIActionsProcessingTimer, SIGNAL(timeout()), this, SLOT(delayedGUIActionsProcessingTimerUpdate()));
    delayedGUIActionsProcessingTimer->start(250);
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

void MainWindow::refreshControlDeviceTableWidget()
{
    ui->controlDeviceTableWidget->clear();

    ui->controlDeviceTableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("Device File Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Id"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Port Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("WDR Intent"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(4, new QTableWidgetItem("System Device Path"));

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "TRUE" :"FALSE")));
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getIntentionalWDTimerResetOnAT91SAM7s() ? "ON" :"OFF"));
        ui->controlDeviceTableWidget->setItem(i, 4, new QTableWidgetItem(controlDevice->getSysDevicePath().c_str()));
        i++;
    }
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

        controlDevice->setResetWDTimer(true);

        if (controlDevice->getControlDevicePort()->isOpen())
        {
            controlDevice->deactivate();
        }
        else
        {
            controlDevice->activate();
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
    int selectedControlDeviceRowIndex = ui->controlDeviceTableWidget->selectionModel()->currentIndex().row();

    ui->controlDeviceTableWidget->clear();

    ui->controlDeviceTableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("Device File Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Id"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Port Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("WDR Intent"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(4, new QTableWidgetItem("System Device Path"));

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        if (controlDevice->getRescanningAfterPossibleReset())
        {
            string rescanningMsg = Valter::format_string("rescanning... Attempt #%d", controlDevice->getRescanNum());
            QTableWidgetItem *item = new QTableWidgetItem(rescanningMsg.c_str());
            item->setBackground(Qt::yellow);
            ui->controlDeviceTableWidget->setItem(i, 2, item);
        }
        else
        {
            if (controlDevice->getWdTimerNotResetCnt() > 0)
            {
                string noWDResetMsg = Valter::format_string("no WDRESET answer... Attempt #%d", controlDevice->getWdTimerNotResetCnt());
                QTableWidgetItem *item = new QTableWidgetItem(noWDResetMsg.c_str());
                item->setBackground(Qt::yellow);
                ui->controlDeviceTableWidget->setItem(i, 2, item);
            }
            else if (controlDevice->getFailedAfterRescanning())
            {
                controlDevice->setWdTimerNotResetCnt(0);
                QTableWidgetItem *item = new QTableWidgetItem("FAILED");
                item->setBackground(Qt::red);
                ui->controlDeviceTableWidget->setItem(i, 2, item);
            }
            else
            {
                ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "TRUE" :"FALSE")));
            }
        }
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getIntentionalWDTimerResetOnAT91SAM7s() ? "ON" :"OFF"));
        ui->controlDeviceTableWidget->setItem(i, 4, new QTableWidgetItem(controlDevice->getSysDevicePath().c_str()));
        i++;
    }

    if (selectedControlDeviceRowIndex >= 0)
    {
        ui->controlDeviceTableWidget->selectRow(selectedControlDeviceRowIndex);
    }
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
        Valter::log("WD STOP RESET signal sent to " + selectedControlDeviceId);
        map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
        ControlDevice *controlDevice = controlDevicesMap[selectedControlDeviceId];
        controlDevice->reScanThisControlDevice();
        ui->wdResetStopButton->setText("WDR Stop");
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
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (!platformControlP1->getScan220ACAvailable())
    {
        ui->power220VACAvailableRadioButton->setChecked(false);
        ui->power220VACAvailableRadioButton->setEnabled(false);
        ui->chargerVoltageLabel->setEnabled(false);
        ui->chargerVoltageADCCheckBox->setEnabled(false);
        ui->chargerVoltageLcdNumber->setEnabled(false);
        ui->charger35AhRadioButton->setEnabled(false);
        ui->charger35AhRadioButton->setChecked(false);
        ui->charger120AhRadioButton->setEnabled(false);
        ui->charger120AhRadioButton->setChecked(false);
        ui->chargingInProgressRadioButton->setEnabled(false);
        ui->chargingInProgressRadioButton->setChecked(false);
        ui->chargingCompleteRadioButton->setEnabled(false);
        ui->chargingCompleteRadioButton->setChecked(false);
        ui->chargerConnectedRadioButton->setEnabled(false);
        ui->chargerConnectedRadioButton->setChecked(false);
        ui->charger120Ah14v7RadioButton->setEnabled(false);
        ui->chargerButton->setEnabled(false);
        platformControlP1->setChargerVoltageADC(0);
    }
    else
    {
        ui->power220VACAvailableRadioButton->setEnabled(true);
        ui->chargerVoltageLabel->setEnabled(true);
        ui->chargerVoltageADCCheckBox->setEnabled(true);
        ui->chargerVoltageLcdNumber->setEnabled(true);
        ui->charger35AhRadioButton->setEnabled(true);
        ui->charger120AhRadioButton->setEnabled(true);
        ui->chargingInProgressRadioButton->setEnabled(true);
        ui->chargingCompleteRadioButton->setEnabled(true);
        ui->chargerButton->setEnabled(true);
        ui->chargerConnectedRadioButton->setEnabled(true);
        ui->charger120Ah14v7RadioButton->setEnabled(true);
    }

    ui->leftMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getLeftMotorDutyMax()).c_str());
    ui->rightMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getRightMotorDutyMax()).c_str());
    ui->turretMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getTurretMotorDutyMax()).c_str());
    ui->leftMotorCurDutyBar->setValue(platformControlP1->getLeftMotorDuty());
    ui->rightMotorCurDutyBar->setValue(platformControlP1->getRightMotorDuty());
    ui->turretMotorCurDutyBar->setValue(platformControlP1->getTurretMotorDuty());

    ui->powerSource5VRadioButton->setChecked(platformControlP1->getPower5VOnState());
    ui->leftAccumulatorConnectedRadioButton->setChecked(platformControlP1->getLeftAccumulatorConnected());
    ui->rightAccumulatorConnected->setChecked(platformControlP1->getRightAccumulatorConnected());
    ui->mainAccumulatorRelayRadioButton->setChecked(platformControlP1->getMainAccumulatorRelayOnState());
    ui->leftAccumulatorRelay->setChecked(platformControlP1->getLeftAccumulatorRelayOnState());
    ui->rightAccumulatorRelay->setChecked(platformControlP1->getRightAccumulatorRelayOnState());
    ui->power220VACAvailableRadioButton->setChecked(platformControlP1->getPower220VACAvailable());
    ui->charger35AhRadioButton->setChecked(platformControlP1->getCharger35Ah());
    ui->charger120AhRadioButton->setChecked(platformControlP1->getCharger120Ah());
    ui->chargingInProgressRadioButton->setChecked(platformControlP1->getChargingInProgress());
    ui->chargingCompleteRadioButton->setChecked(platformControlP1->getChargingComplete());
    ui->chargerConnectedRadioButton->setChecked(platformControlP1->getChargerConnected());
    if (ui->chargerVoltageADCCheckBox->isChecked())
    {
        ui->chargerVoltageLcdNumber->display(platformControlP1->getChargerVoltageADC());
    }
    else
    {
        ui->chargerVoltageLcdNumber->display(platformControlP1->getChargerVoltageVolts());
    }
    if (platformControlP1->getControlDeviceIsSet())
    {
        switch (platformControlP1->getChargerButtonPressStep())
        {
                case 0:
                    ui->chargerButtonPressStep0->setText("←");
                    ui->chargerButtonPressStep1->setText("");
                    ui->chargerButtonPressStep2->setText("");
                    ui->chargerButtonPressStep3->setText("");
                break;
                case 1:
                    ui->chargerButtonPressStep0->setText("");
                    ui->chargerButtonPressStep1->setText("←");
                    ui->chargerButtonPressStep2->setText("");
                    ui->chargerButtonPressStep3->setText("");
                break;
                case 2:
                    ui->chargerButtonPressStep0->setText("");
                    ui->chargerButtonPressStep1->setText("");
                    ui->chargerButtonPressStep2->setText("←");
                    ui->chargerButtonPressStep3->setText("");
                break;
                case 3:
                    ui->chargerButtonPressStep0->setText("");
                    ui->chargerButtonPressStep1->setText("");
                    ui->chargerButtonPressStep2->setText("");
                    ui->chargerButtonPressStep3->setText("←");
                break;
        }
    }
    else
    {
        ui->chargerButtonPressStep0->setText("");
        ui->chargerButtonPressStep1->setText("");
        ui->chargerButtonPressStep2->setText("");
        ui->chargerButtonPressStep3->setText("");
    }
}

void MainWindow::on_platformControlP1WheelMotorsDutySlider_sliderMoved(int dutyValue)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    static int prevDutyVal = 1;
    int maxDutyLeft = platformControlP1->getLeftMotorDutyMax();
    int maxDutyRight = platformControlP1->getRightMotorDutyMax();

    if(dutyValue > prevDutyVal)
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

    prevDutyVal = dutyValue;
}

void MainWindow::on_leftMotorPlatformControlP1DutySlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setLeftMotorDutyMax(value);
}

void MainWindow::on_rightMotorPlatformControlP1DutySlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setRightMotorDutyMax(value);
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

void MainWindow::on_platformMovementAccelerationSlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setPlatformAcceleration(value);
}

void MainWindow::on_platformMovementDecelerationSlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setPlatformDeceleration(value);
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

void MainWindow::on_turretRotationDutySlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretMotorDutyMax(value);
}

void MainWindow::on_decelerationTurretRotationSlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretDeceleration(value);
}

void MainWindow::on_accelerationTurretRotationSlider_sliderMoved(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->setTurretAcceleration(value);
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
