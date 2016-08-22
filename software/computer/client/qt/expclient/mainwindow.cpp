#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/controldevicesGUI.h>
#include <gui/platformcontrolp1GUI.h>
#include <gui/platformcontrolp2GUI.h>
#include <gui/platformlocationp1GUI.h>
#include <gui/platformmanipulatorandirbumperGUI.h>
#include <gui/link1endpointviewitem.h>
#include <gui/link2endpointviewitem.h>
#include <gui/link3endpointviewitem.h>
#include <gui/bodycontrolp1GUI.h>
#include <gui/armcontrolleftGUI.h>
#include <gui/armcontrolrightGUI.h>
#include <gui/taskstabgui.h>

#include "mainwindow/mainwindow.control.devices.utils.cpp"
#include "mainwindow/mainwindow.platformcontrolp1.cpp"
#include "mainwindow/mainwindow.platformlocationp1.cpp"
#include "mainwindow/mainwindow.platformcontrolp2.cpp"
#include "mainwindow/mainwindow.manipulatorandirbumper.cpp"
#include "mainwindow/mainwindow.bodycontrolp1.cpp"
#include "mainwindow/mainwindow.armcontrolleft.cpp"
#include "mainwindow/mainwindow.armcontrolright.cpp"


MainWindow* MainWindow::pMainWindow = NULL;
bool MainWindow::instanceFlag = false;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    statusBarText = new QLabel();
    statusBar()->addWidget(statusBarText, 1);

    valter3d = 0;

    logLength = 0;
    allConnect = true;

    delayedGUIActionsProcessingTimer = new QTimer(this);
    connect(delayedGUIActionsProcessingTimer, SIGNAL(timeout()), this, SLOT(delayedGUIActionsProcessingTimerUpdate()));
    delayedGUIActionsProcessingTimer->start(250);

    ui->mainTabWidget->setCurrentIndex(8);

    //controlDeviceTableWidget
    initControlDevices(ui);

    //PLATFORM-CONTROL-P1
    initPlatfromControlP1(ui);

    //PLATFORM-LOCATION-P1
    initPlatfromLocationP1(ui);

    //PLATFROM-CONTROL-P2
    initPlatfromControlP2(ui);

    //PLATFORM-MANIPULATOR-AND-IR-BUMPER
    initPlatformManipulatorAndIRBumper(ui);

    //BODY-CONTROL-P1
    initBodyControlP1(ui);

    //ARM-CONTROL-RIGHT
    initArmControlRight(ui);

    //ARM-CONTROL-LEFT
    initArmControlLeft(ui);

    ui->centralCommandHostIPLineEdit->setText(TCPInterface::getLocalHostIP().c_str());

    connect(ui->stopAllButton1, SIGNAL(clicked()), this, SLOT(on_stopAllButton1_clicked()));
    connect(ui->stopAllButton2, SIGNAL(clicked()), this, SLOT(on_stopAllButton2_clicked()));
    connect(ui->stopAllButton3, SIGNAL(clicked()), this, SLOT(on_stopAllButton3_clicked()));
    connect(ui->stopAllButton4, SIGNAL(clicked()), this, SLOT(on_stopAllButton4_clicked()));
    connect(ui->stopAllButton5, SIGNAL(clicked()), this, SLOT(on_stopAllButton5_clicked()));
    connect(ui->stopAllButton6, SIGNAL(clicked()), this, SLOT(on_stopAllButton6_clicked()));
    connect(ui->stopAllButton7, SIGNAL(clicked()), this, SLOT(on_stopAllButton7_clicked()));


    tasksTabRefreshTimer = new QTimer(this);
    connect(tasksTabRefreshTimer, SIGNAL(timeout()), this, SLOT(tasksTabRefreshTimerUpdate()));
    tasksTabRefreshTimer->start(100);

    ui->tasksTableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
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

void MainWindow::on_stopAllButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_stopAllButton_clicked");
    }
    platformControlP1->sendCommand("DCDC5VENABLEOFF");
}

void MainWindow::on_stopAllButton1_clicked()
{
    on_stopAllButton_clicked();
}
void MainWindow::on_stopAllButton2_clicked()
{
    on_stopAllButton_clicked();
}
void MainWindow::on_stopAllButton3_clicked()
{
    on_stopAllButton_clicked();
}
void MainWindow::on_stopAllButton4_clicked()
{
    on_stopAllButton_clicked();
}
void MainWindow::on_stopAllButton5_clicked()
{
    on_stopAllButton_clicked();
}
void MainWindow::on_stopAllButton6_clicked()
{
    on_stopAllButton_clicked();
}
void MainWindow::on_stopAllButton7_clicked()
{
    on_stopAllButton_clicked();
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
    //from PlatformControlP2
    if (valterModule->getControlDevice()->getControlDeviceId().compare(PlatformControlP2::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadPlatformControlP2Defaults(ui);
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
    //from PlatformManipulatorAndIRBumper
    if (valterModule->getControlDevice()->getControlDeviceId().compare(PlatformManipulatorAndIRBumper::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadPlatformManipulatorAndIRBumperDefaults(ui);
            break;
        }
    }
    //from BodyControlP1
    if (valterModule->getControlDevice()->getControlDeviceId().compare(BodyControlP1::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadBodyControlP1Defaults(ui);
            break;
        }
    }
    //from ArmControlRight
    if (valterModule->getControlDevice()->getControlDeviceId().compare(ArmControlRight::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadArmControlRightDefaults(ui);
            break;
        }
    }
    //from ArmControlLeft
    if (valterModule->getControlDevice()->getControlDeviceId().compare(ArmControlLeft::getControlDeviceId()) == 0)
    {
        switch (action)
        {
            case IValterModule::RELOAD_DEFAULTS:
                valterModule->loadDefaults();
                loadArmControlLeftDefaults(ui);
            break;
        }
    }
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

//------------------------------------------------Utilites
//Valter3D
void MainWindow::on_valter3dOpenButton_clicked()
{
    if (!valter3d)
    {
        QQuickView *pValter3dView = new QQuickView();
        pValter3dView->setSource(QUrl("qrc:/valter3d/valter3dView.qml"));
        valter3d = (Valter3d*) Valter3d::createWindowContainer(pValter3dView);
        valter3d->installEventFilter(new Valter3DEventFilter(this));
        valter3d->valter3dView = pValter3dView;
        valter3d->setWindowTitle("Valter 3D");
    }
    valter3d->show();
}

void MainWindow::on_horizontalScrollBar_9_valueChanged(int value)
{
    if (valter3d != 0)
    {
        valter3d->setValterTrunkRotationY((double)value / 10);
    }
}

void MainWindow::on_horizontalScrollBar_10_valueChanged(int value)
{
    if (valter3d != 0)
    {
        valter3d->setValterBodyRotationZ(-1 * (double)value / 10);
    }
}

//TCP Interface Tab
void MainWindow::on_updateCentralCommandHostConnectionInfoOnAllSlavesButton_clicked()
{
    refreshControlDeviceTableTimer->start(1000);

    string centralCommandHostIp = ui->centralCommandHostIPLineEdit->text().toStdString();

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->getTcpInterface()->setCommandHostIP(ui->platformControlP1CommandHostLineEdit->text().toStdString());
        platformControlP1->getTcpInterface()->setCommandHostPort(atoi(ui->platformControlP1CommandPortLineEdit->text().toStdString().c_str()));
        ui->platformControlP1RedrawGUICheckBox->setChecked(true);
        platformControlP1->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), platformControlP1->getTcpInterface()->getPort()));
        platformControlP1->loadDefaults();
        loadPlatformControlP1Defaults(ui);
    }

    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->getTcpInterface()->setCommandHostIP(ui->platformControlP2CommandHostLineEdit->text().toStdString());
        platformControlP2->getTcpInterface()->setCommandHostPort(atoi(ui->platformControlP2CommandPortLineEdit->text().toStdString().c_str()));
        ui->platfromControlP2RedrawGUICheckBox->setChecked(true);
        platformControlP2->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), platformControlP2->getTcpInterface()->getPort()));
        platformControlP2->loadDefaults();
        loadPlatformControlP2Defaults(ui);
    }

    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    if (ui->platformLocationP1RemoteControlCheckbox->isChecked())
    {
        platformLocationP1->getTcpInterface()->setCommandHostIP(ui->platformLocationP1CommandHostLineEdit->text().toStdString());
        platformLocationP1->getTcpInterface()->setCommandHostPort(atoi(ui->platformLocationP1CommandPortLineEdit->text().toStdString().c_str()));
        ui->platformLocationP1RedrawGUICheckBox->setChecked(true);
        platformLocationP1->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), platformLocationP1->getTcpInterface()->getPort()));
        platformLocationP1->loadDefaults();
        loadPlatformLocationP1Defaults(ui);
    }

    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->getTcpInterface()->setCommandHostIP(ui->platformManipulatorAndIRBumperCommandHostLineEdit->text().toStdString());
        platformManipulatorAndIRBumper->getTcpInterface()->setCommandHostPort(atoi(ui->platformManipulatorAndIRBumperCommandPortLineEdit->text().toStdString().c_str()));
        ui->platformManipulatorAndIRBumperRedrawGUICheckBox->setChecked(true);
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), platformManipulatorAndIRBumper->getTcpInterface()->getPort()));
        platformManipulatorAndIRBumper->loadDefaults();
        loadPlatformManipulatorAndIRBumperDefaults(ui);
    }

    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->getTcpInterface()->setCommandHostIP(ui->bodyControlP1CommandHostLineEdit->text().toStdString());
        bodyControlP1->getTcpInterface()->setCommandHostPort(atoi(ui->bodyControlP1CommandPortLineEdit->text().toStdString().c_str()));
        ui->bodyControlP1CheckBox->setChecked(true);
        bodyControlP1->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), bodyControlP1->getTcpInterface()->getPort()));
        bodyControlP1->loadDefaults();
        loadBodyControlP1Defaults(ui);
    }

    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->getTcpInterface()->setCommandHostIP(ui->armControlRightCommandHostLineEdit->text().toStdString());
        armControlRight->getTcpInterface()->setCommandHostPort(atoi(ui->armControlRightCommandPortLineEdit->text().toStdString().c_str()));
        ui->armControlRightRedrawGUICheckBox->setChecked(true);
        armControlRight->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), armControlRight->getTcpInterface()->getPort()));
        armControlRight->loadDefaults();
        loadArmControlRightDefaults(ui);
    }

    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->getTcpInterface()->setCommandHostIP(ui->armControlLeftCommandHostLineEdit->text().toStdString());
        armControlLeft->getTcpInterface()->setCommandHostPort(atoi(ui->armControlLeftCommandPortLineEdit->text().toStdString().c_str()));
        ui->armControlLeftRedrawGUICheckBox->setChecked(true);
        armControlLeft->sendTCPCommand(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), armControlLeft->getTcpInterface()->getPort()));
        armControlLeft->loadDefaults();
        loadArmControlLeftDefaults(ui);
    }

    TaskManager::getInstance()->sendScript(Valter::format_string("setCentralCommandHostInfo@%s@%d", centralCommandHostIp.c_str(), 55555));
}

void MainWindow::on_tcpInterfaceRemoteControlDevicesHostsUpdateSettingsButton_clicked()
{
    refreshControlDeviceTableTimer->start(1000);

    on_updateCentralCommandHostConnectionInfoOnAllSlavesButton_clicked();
}


void MainWindow::on_tcpInterfaceRemoteControlDevicesHostsDisconnectAllButton_clicked()
{
    refreshControlDeviceTableTimer->stop();

    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");

    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");

    PlatformLocationP1 *platformLocationP1 = PlatformLocationP1::getInstance();
    platformLocationP1->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");

    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");

    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");

    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");

    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->getTcpInterface()->sendCommandMessage("stopCDTtoCentralCommandHost");
}

void MainWindow::on_tcpInterfaceRemoteControlCheckAllAsRemoteControlledButton_clicked()
{
    ui->platformControlP1RemoteControlCheckbox->setChecked(true);
    ui->platformControlP2RemoteControlCheckbox->setChecked(true);
    ui->platformLocationP1RemoteControlCheckbox->setChecked(true);
    ui->platformManipulatorAndIRBumperRemoteControlCheckbox->setChecked(true);
    ui->bodyControlP1RemoteControlCheckbox->setChecked(true);
    ui->armControlRightRemoteControlCheckbox->setChecked(true);
    ui->armControlLeftRemoteControlCheckbox->setChecked(true);
}


//Tasks


void MainWindow::on_executeScriptButton_clicked()
{
    if (ui->executeTasksScriptLocallyCheckBox->isChecked())
    {
        TaskManager::getInstance()->processScript(ui->taskScriptTextEdit->toPlainText().toStdString());
    }
    else
    {
        TaskManager::getInstance()->sendScript(ui->taskScriptTextEdit->toPlainText().toStdString());
    }
    if (ui->autoClearRTMMCheckBox->isChecked())
    {
        on_clearRemoteTaskManagerTasksMapButton_clicked();
    }
    ui->tasksRTMMTableRefreshCheckBox->setChecked(true);
}

void MainWindow::on_clearTasksQueueButton_clicked()
{
    if (ui->executeTasksScriptLocallyCheckBox->isChecked())
    {
        TaskManager::getInstance()->clearQueue();
    }
    else
    {
        vector<string> remoteControlDeviceTCPInterfacesIpAdresses = Valter::getInstance()->getRemoteControlDeviceTCPInterfacesIpAddressesVector();
        for(std::vector<string>::size_type i = 0; i != remoteControlDeviceTCPInterfacesIpAdresses.size(); i++)
        {
            TaskManager::getInstance()->sendScriptToRemoteTaskManager("CLEARQUEUE", remoteControlDeviceTCPInterfacesIpAdresses[i]);
        }
    }
}

void MainWindow::on_forcefullyCompleteTaskButton_clicked()
{
    if (ui->executeTasksScriptLocallyCheckBox->isChecked())
    {
        TaskManager::getInstance()->setStopTopTask(true);
    }
    else
    {
        TaskManager::getInstance()->sendScript("STOPTOPTASK");
    }
}

void MainWindow::tasksTabRefreshTimerUpdate()
{
    tasksTabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_clearRemoteTaskManagerTasksMapButton_clicked()
{
    TaskManager::getInstance()->clearRTMM();
}

void MainWindow::on_stopSelectedTaskButton_toggled(bool checked)
{
    if (checked)
    {
        ui->stopSelectedTaskButton->setStyleSheet("background-color: rgb(255, 247, 0)");
        ui->tasksRTMMTableRefreshCheckBox->setChecked(false);
    }
    else
    {
        ui->stopSelectedTaskButton->setStyleSheet("");
    }
}

void MainWindow::on_tasksTableWidget_itemClicked(QTableWidgetItem *item)
{
    if (ui->stopSelectedTaskButton->isChecked())
    {
        long selectedTaskId = atol(ui->tasksTableWidget->item(item->row(), 0)->text().toStdString().c_str());
        TaskManager::getInstance()->sendScript(Valter::format_string("STOPTASK_%lu", selectedTaskId));
        qDebug("stop task %lu request sent", selectedTaskId);

        ui->tasksRTMMTableRefreshCheckBox->setChecked(true);
        ui->stopSelectedTaskButton->setStyleSheet("");
        ui->stopSelectedTaskButton->setChecked(false);
    }
}
