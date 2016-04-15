#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/platformcontrolp1GUI.h>
#include <gui/platformcontrolp2GUI.h>
#include <gui/platformlocationp1GUI.h>
#include <gui/platformmanipulatorandirbumperGUI.h>
#include <gui/link1endpointviewitem.h>
#include <gui/link2endpointviewitem.h>
#include <gui/link3endpointviewitem.h>

#include "mainwindow/mainwindow.control.devices.utils.cpp"
#include "mainwindow/mainwindow.platformcontrolp1.cpp"
#include "mainwindow/mainwindow.platformlocationp1.cpp"
#include "mainwindow/mainwindow.platformcontrolp2.cpp"
#include "mainwindow/mainwindow.manipulatorandirbumper.cpp"


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
