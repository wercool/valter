#include <QtDebug>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <valter.h>
#include <sys/time.h>

MainWindow* MainWindow::pMainWindow = NULL;
bool MainWindow::instanceFlag = false;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowIcon(QIcon(":/valter_head_icon.png"));

    //controlDeviceTableWidget
    ui->controlDeviceTableWidget->setColumnCount(4);
    ui->controlDeviceTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->controlDeviceTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem("Device File Name"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem("Control Device Id"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("Status"));
    QHeaderView* controlDeviceTableWidgetHeaderView = new QHeaderView(Qt::Horizontal);
    controlDeviceTableWidgetHeaderView->setStretchLastSection(true);
    controlDeviceTableWidgetHeaderView->setSectionResizeMode(QHeaderView::Stretch);
    ui->controlDeviceTableWidget->setHorizontalHeader(controlDeviceTableWidgetHeaderView);

    logLength = 0;
    allConnect = true;

    controlDevicesDataExchangeLogTimer = new QTimer(this);
    connect(controlDevicesDataExchangeLogTimer, SIGNAL(timeout()), this, SLOT(controlDevicesDataExchangeLogTimerUpdate()));
    controlDevicesDataExchangeLogTimer->start(1);
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
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(2, new QTableWidgetItem("Opened"));
    ui->controlDeviceTableWidget->setHorizontalHeaderItem(3, new QTableWidgetItem("Status"));

    map<string, ControlDevice*> controlDevicesMap = Valter::getInstance()->getControlDevicesMap();
    typedef map<string, ControlDevice*>::iterator it_type;

    ui->controlDeviceTableWidget->setRowCount(controlDevicesMap.size());

    char i = 0;
    for(it_type iterator = controlDevicesMap.begin(); iterator != controlDevicesMap.end(); iterator++)
    {
        ControlDevice *controlDevice = controlDevicesMap[iterator->first];
        ui->controlDeviceTableWidget->setItem(i, 0, new QTableWidgetItem(controlDevice->getControlDevicePort()->getPort() .c_str()));
        ui->controlDeviceTableWidget->setItem(i, 1, new QTableWidgetItem(controlDevice->getControlDeviceId().c_str()));
        ui->controlDeviceTableWidget->setItem(i, 2, new QTableWidgetItem((controlDevice->getControlDevicePort()->isOpen() ? "true" :"false")));
        ui->controlDeviceTableWidget->setItem(i, 3, new QTableWidgetItem(controlDevice->getStatus().c_str()));
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
}


void MainWindow::on_scanControlDevicesBtn_clicked()
{
    ui->selectedControlDeviceListWidget->clear();
    ui->commandEdit->clear();
    selectedControlDeviceId = "";


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

        if (controlDevice->getControlDevicePort()->isOpen())
        {
            controlDevice->setStatus(ControlDevice::StatusReady);
            controlDevice->getControlDevicePort()->close();
        }
        else
        {
            controlDevice->getControlDevicePort()->open();
            controlDevice->setStatus(ControlDevice::StatusActive);
            controlDevice->spawnControlDeviceThreadWorker();
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
                Valter::log(controlDevice->getMsgFromDataExchangeLog());
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
                Valter::log(controlDevice->getMsgFromDataExchangeLog());
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
                controlDevice->getControlDevicePort()->open();
                controlDevice->setStatus(ControlDevice::StatusActive);
                controlDevice->spawnControlDeviceThreadWorker();
            }
        }
        else
        {
            if (controlDevice->getControlDevicePort()->isOpen())
            {
                controlDevice->setStatus(ControlDevice::StatusReady);
                controlDevice->getControlDevicePort()->close();
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
