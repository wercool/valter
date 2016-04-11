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


MainWindow* MainWindow::pMainWindow = NULL;
bool MainWindow::instanceFlag = false;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    statusBarText = new QLabel();
    statusBar()->addWidget(statusBarText, 1);

    valter3d = 0;

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


    //PLATFROM-CONTROL-P2
    platformControlP2EncodersRefreshTimer = new QTimer(this);
    connect(platformControlP2EncodersRefreshTimer, SIGNAL(timeout()), this, SLOT(platformControlP2EncodersRefreshTimerUpdate()));

    irScanningGraphicsViewScene = new QGraphicsScene;
    irScanningGraphicsViewScene->setSceneRect(0, 0, 586, 296);
    ui->irScanningGraphicsView->setScene(irScanningGraphicsViewScene);

    platformControlP2IRScannerRefreshTimer = new QTimer(this);
    connect(platformControlP2IRScannerRefreshTimer, SIGNAL(timeout()), this, SLOT(platformControlP2IRScannerRefreshTimerUpdate()));

    IRScannerVector = new QGraphicsLineItem;
    IRScannerVector->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
    IRScannerVector->setLine(293, 285, 293, 50);
    irScanningGraphicsViewScene->addItem(IRScannerVector);

    //PLATFORM-MANIPULATOR-AND-IR-BUMPER
    platformManipulatorAndIRBumperRefreshTimer = new QTimer(this);
    connect(platformManipulatorAndIRBumperRefreshTimer, SIGNAL(timeout()), this, SLOT(platformManipulatorAndIRBumperRefreshTimerUpdate()));

    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene = new QGraphicsScene;
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->setSceneRect(0, 0, 246, 246);
    ui->platformManipulatorAndIRBumperLink1Link2PositionGraphicsView->setScene(platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene);
    ui->platformManipulatorAndIRBumperLink1Link2PositionGraphicsView->setRenderHint(QPainter::Antialiasing);

    double man_l1 = PlatformManipulatorAndIRBumper::man_l1;
    double man_l2 = PlatformManipulatorAndIRBumper::man_l2;
    double man_l1_l2 = PlatformManipulatorAndIRBumper::man_l1_l2;
    double man_l2_l3 = PlatformManipulatorAndIRBumper::man_l2_l3;
    double man_l3 = PlatformManipulatorAndIRBumper::man_l3;
    double rootX = PlatformManipulatorAndIRBumper::rootX;
    double rootY = PlatformManipulatorAndIRBumper::rootY;

    //link1
    platfromManipulatorLink1 = new QGraphicsLineItem;
    platfromManipulatorLink1->setPen(QPen(Qt::gray, 6.0, Qt::SolidLine));
    platfromManipulatorLink1->setLine(rootX, rootY, rootX, rootY + man_l1);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink1);

    platfromManipulatorLink1Initial = new QGraphicsLineItem;
    platfromManipulatorLink1Initial->setPen(QPen(Qt::gray, 0.5, Qt::SolidLine));
    platfromManipulatorLink1Initial->setLine(platfromManipulatorLink1->line());
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink1Initial);

    platfromManipulatorLink1Link2Console = new QGraphicsLineItem;
    platfromManipulatorLink1Link2Console->setPen(QPen(Qt::gray, 6.0, Qt::SolidLine));
    platfromManipulatorLink1Link2Console->setLine(rootX, rootY + man_l1, rootX - man_l1_l2, rootY + man_l1);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink1Link2Console);

    platfromManipulatorLink1Helper = new QGraphicsLineItem;
    platfromManipulatorLink1Helper->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
    platfromManipulatorLink1Helper->setLine(platfromManipulatorLink1->line());
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink1Helper);


    //link2
    platfromManipulatorLink2 = new QGraphicsLineItem;
    platfromManipulatorLink2->setPen(QPen(Qt::gray, 6.0, Qt::SolidLine));
    platfromManipulatorLink2->setLine(rootX - man_l1_l2, rootY + man_l1, rootX - man_l1_l2, rootY + man_l1 - man_l2);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink2);

    platfromManipulatorLink2Initial = new QGraphicsLineItem;
    platfromManipulatorLink2Initial->setPen(QPen(Qt::gray, 0.5, Qt::SolidLine));
    platfromManipulatorLink2Initial->setLine(platfromManipulatorLink2->line());
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink2Initial);

    platfromManipulatorLink2Link3Console = new QGraphicsLineItem;
    platfromManipulatorLink2Link3Console->setPen(QPen(Qt::gray, 6.0, Qt::SolidLine));
    platfromManipulatorLink2Link3Console->setLine(rootX - man_l1_l2, rootY + man_l1 - man_l2, rootX - man_l1_l2 + man_l2_l3, rootY + man_l1 - man_l2);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink2Link3Console);

    platfromManipulatorLink2Helper = new QGraphicsLineItem;
    platfromManipulatorLink2Helper->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
    platfromManipulatorLink2Helper->setLine(platfromManipulatorLink2->line());
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink2Helper);

    //link3
    platfromManipulatorLink3 = new QGraphicsLineItem;
    platfromManipulatorLink3->setPen(QPen(Qt::gray, 6.0, Qt::SolidLine));
    platfromManipulatorLink3->setLine(rootX - man_l1_l2 + man_l2_l3, rootY + man_l1 - man_l2, rootX - man_l1_l2 + man_l2_l3, rootY + man_l1 - man_l2 - man_l3);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink3);

    platfromManipulatorLink3Initial = new QGraphicsLineItem;
    platfromManipulatorLink3Initial->setPen(QPen(Qt::gray, 0.5, Qt::SolidLine));
    platfromManipulatorLink3Initial->setLine(platfromManipulatorLink3->line());
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink3Initial);

    platfromManipulatorLink3Helper = new QGraphicsLineItem;
    platfromManipulatorLink3Helper->setPen(QPen(Qt::black, 1.0, Qt::DashLine));
    platfromManipulatorLink3Helper->setLine(platfromManipulatorLink3->line());
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(platfromManipulatorLink3Helper);

    QBrush brush;

    link1RootPoint = new QGraphicsEllipseItem;
    link1RootPoint->setRect(rootX - 6, rootY - 6, 12, 12);
    brush = link1RootPoint->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::lightGray);
    link1RootPoint->setBrush(brush);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(link1RootPoint);

    link1link2Point = new QGraphicsEllipseItem;
    link1link2Point->setPos(rootX - man_l1_l2 - 6, rootY + man_l1 - 6);
    link1link2Point->setRect(0, 0, 12, 12);
    brush = link1link2Point->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::gray);
    link1link2Point->setBrush(brush);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(link1link2Point);

    link2link3Point = new QGraphicsEllipseItem;
    link2link3Point->setPos(rootX - man_l1_l2 + man_l2_l3 - 6, rootY + man_l1 - man_l2 - 6);
    link2link3Point->setRect(0, 0, 12, 12);
    brush = link2link3Point->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::lightGray);
    link2link3Point->setBrush(brush);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(link2link3Point);

    link1EndPoint = new Link1EndPointViewItem;
    link1EndPoint->setPos(rootX - 6, rootY + man_l1 - 6);
    link1EndPoint->setRect(0, 0, 12, 12);
    brush = link1EndPoint->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::darkGreen);
    link1EndPoint->setBrush(brush);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(link1EndPoint);

    link2EndPoint = new Link2EndPointViewItem;
    link2EndPoint->setPos(rootX - man_l1_l2 - 6, rootY + man_l1 - man_l2 - 6);
    link2EndPoint->setRect(0, 0, 12, 12);
    brush = link2EndPoint->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::darkGreen);
    link2EndPoint->setBrush(brush);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(link2EndPoint);

    link3EndPoint = new Link3EndPointViewItem;
    link3EndPoint->setPos(rootX - man_l1_l2 + man_l2_l3 - 6, rootY + man_l1 - man_l2 - man_l3 - 6);
    link3EndPoint->setRect(0, 0, 12, 12);
    brush = link3EndPoint->brush();
    brush.setStyle(Qt::SolidPattern);
    brush.setColor(Qt::darkGreen);
    link3EndPoint->setBrush(brush);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(link3EndPoint);

    QFont font = QFont();
    font.setPixelSize(10);
    alpha = new QGraphicsTextItem;
    alpha->setPos(rootX + 5, rootY - 5);
    alpha->setPlainText("α:0.00");
    alpha->setFont(font);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(alpha);

    beta = new QGraphicsTextItem;
    beta->setPos(rootX - man_l1_l2 - 20, rootY + man_l1);
    beta->setPlainText("β:0.00");
    beta->setFont(font);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(beta);

    gamma = new QGraphicsTextItem;
    gamma->setPos(rootX - man_l1_l2 + man_l2_l3 + 5, rootY + man_l1 - man_l2 - 15);
    gamma->setPlainText("γ:0.00");
    gamma->setFont(font);
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->addItem(gamma);
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggle5VSource(true);
}

void MainWindow::on_off5VPlatformControlP1pushButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggle5VSource(false);
}

void MainWindow::on_onLeftAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleLeftAccumulator(true);
}

void MainWindow::on_offLeftAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleLeftAccumulator(false);
}

void MainWindow::on_onRightAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleRightAccumulator(true);
}

void MainWindow::on_offRightAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleRightAccumulator(false);
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleMainAccumulatorRelayState(false);
}

void MainWindow::on_onLeftAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleLeftAccumulatorRelay(true);
}

void MainWindow::on_offLeftAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleLeftAccumulatorRelay(false);
}

void MainWindow::on_onRightAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleRightAccumulatorRelay(true);
}

void MainWindow::on_offRightAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    platformControlP1->toggleRightAccumulatorRelay(false);
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
}

void MainWindow::on_platformControlP1RightWheelEncoderResetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    platformControlP1->resetRightWheelEncoder();
}

void MainWindow::on_platformControlP1LeftWheelEncoderGetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    ui->platformControlP1LeftWheelEncoderLcdNumber->display("");
    platformControlP1->setLeftWheelEncoderGetOnce(true);
}

void MainWindow::on_platformControlP1RightWheelEncoderGetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    ui->platformControlP1RightWheelEncoderLcdNumber->display("");
    platformControlP1->setRightWheelEncoderGetOnce(true);
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

//PLATFROM-CONTROL-P2
void MainWindow::on_chargerMotorDutyScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorDutyPresetCur(value);
    ui->chargerMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_chargerMotorPushDurationScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorPushDurationPresetCur(value);
    ui->chargerMotorPushDurationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_detachIRScanningFrameButton_clicked()
{
    QWidget* pWidget = ui->irScanningFrame;
    pWidget->installEventFilter(new IRScanningFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Bottom IR Scanner");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}

void MainWindow::on_chargerLedsButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerLeds(checked);
}

void MainWindow::on_loadDefaultsPlatfromControlP2Button_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->loadDefaults();
    loadPlatformControlP2Defaults(ui);
}

void MainWindow::on_platformControlP2LeftWheelEncoderCheckBox_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setReadLeftEncoder(checked);
    platformControlP2->setEncodersWorkerActivated(checked);
    if (platformControlP2->getReadLeftEncoder() || platformControlP2->getReadRightEncoder())
    {
        platformControlP2EncodersRefreshTimer->start(200);
    }
    else
    {
        platformControlP2EncodersRefreshTimer->stop();
    }
}

void MainWindow::on_platformControlP2RightWheelEncoderCheckBox_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setReadRightEncoder(checked);
    platformControlP2->setEncodersWorkerActivated(checked);
    if (platformControlP2->getReadLeftEncoder() || platformControlP2->getReadRightEncoder())
    {
        platformControlP2EncodersRefreshTimer->start(200);
    }
    else
    {
        platformControlP2EncodersRefreshTimer->stop();
    }
}

void MainWindow::on_platformControlP2LeftWheelEncoderResetButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->resetLeftEncoder();
}

void MainWindow::on_platformControlP2RightWheelEncoderResetButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->resetRightEncoder();
}

void MainWindow::on_beepDurationScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setBeepDuration(value);
    ui->beepDurationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::platformControlP2EncodersRefreshTimerUpdate()
{
    platformControlP2TabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_platformControlP2LeftWheelEncoderGetButton_clicked()
{
    platformControlP2EncodersRefreshTimer->start(200);
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->getLeftEncoderOnce(true);
}

void MainWindow::on_platformControlP2RightWheelEncoderGetButton_clicked()
{
    platformControlP2EncodersRefreshTimer->start(200);
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->getRightEncoderOnce(true);
}

void MainWindow::on_irScanningButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setIrScanningWorkerActivated(checked);
    if (checked)
    {
        platformControlP2IRScannerRefreshTimer->start(50);
    }
    else
    {
        platformControlP2IRScannerRefreshTimer->stop();
        typedef map<int, QGraphicsEllipseItem*>::iterator it_type;
        for(it_type iterator = IRScannerDots.begin(); iterator != IRScannerDots.end(); iterator++)
        {
            QGraphicsEllipseItem* dot = IRScannerDots[iterator->first];
            if (dot)
            {
                dot->setPen(QPen(Qt::lightGray));
            }
        }
    }
}

void MainWindow::platformControlP2IRScannerRefreshTimerUpdate()
{
    platformControlP2IRScannerRefresh(ui);
}

void MainWindow::on_irScannerAngleScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setIRScannerAngle(value);
    ui->irScannerAngleLabel->setText(Valter::format_string("[%d]", platformControlP2->getIRScannerAngle()).c_str());
}

void MainWindow::on_irScannerAngleScrollBar_sliderPressed()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (!platformControlP2->getIrScanningWorkerActivated())
    {
        platformControlP2->setIRScannerIntentionalAngleSet(true);
        platformControlP2IRScannerRefreshTimer->start(50);
    }
}

void MainWindow::on_irScannerAngleScrollBar_sliderReleased()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (!platformControlP2->getIrScanningWorkerActivated())
    {
        platformControlP2->setIRScannerIntentionalAngleSet(false);
        platformControlP2IRScannerRefreshTimer->stop();
    }
}

void MainWindow::on_chargerMotorPushCCWButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorDirection(false);
    platformControlP2->pushChargerMotor();
}

void MainWindow::on_chargerMotorPushCWButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorDirection(true);
    platformControlP2->pushChargerMotor();
}

void MainWindow::on_chargerMotorRotateCCWButton_pressed()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorDirection(false);
    platformControlP2->setChargerMotorRotateWorkerActivated(true);
}

void MainWindow::on_chargerMotorRotateCWButton_pressed()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorDirection(true);
    platformControlP2->setChargerMotorRotateWorkerActivated(true);
}

void MainWindow::on_chargerMotorRotateCCWButton_released()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorRotateWorkerActivated(false);
}

void MainWindow::on_chargerMotorRotateCWButton_released()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setChargerMotorRotateWorkerActivated(false);
}

void MainWindow::on_beepButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->alarmBeep();
}

void MainWindow::on_alarmButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->ALARMOnOff(checked);
}

void MainWindow::on_bottomFronLedsButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setBottomFrontLeds(checked);
}

void MainWindow::on_bottomRearLedsButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    platformControlP2->setBottomRearLeds(checked);
}

void MainWindow::platformManipulatorAndIRBumperRefreshTimerUpdate()
{
    platformManipulatorAndIRBumperRefreshTimerUpdateWorker(ui);
}

//PLATFORM-MANIPULATOR-AND-IR-BUMPER


void MainWindow::on_platformManipulatorAndIRBumperRedrawGUICheckBox_toggled(bool checked)
{
    if (checked)
    {
        platformManipulatorAndIRBumperRefreshTimer->start(50);
    }
    else
    {
        platformManipulatorAndIRBumperRefreshTimer->stop();
    }
}

void MainWindow::on_platformManipulatorExecuteAngleSettingsButton_clicked()
{
    Link1EndPointViewItem* plink1EndPoint = (Link1EndPointViewItem*)link1EndPoint;
    Link2EndPointViewItem* plink2EndPoint = (Link2EndPointViewItem*)link2EndPoint;
    Link3EndPointViewItem* plink3EndPoint = (Link3EndPointViewItem*)link3EndPoint;

    plink1EndPoint->setAngle(ui->platformManipulatorAngleASpinBox->value() * M_PI / 180, false);
    plink2EndPoint->setAngle(ui->platformManipulatorAngleBSpinBox->value() * M_PI / 180, false);
    plink3EndPoint->setAngle(ui->platformManipulatorAngleGSpinBox->value() * M_PI / 180, false);
}

void MainWindow::on_detachManipulatorFrameButton_clicked()
{
    QWidget* pWidget = ui->manipualtorFrame;
    pWidget->installEventFilter(new PlatfromManipulatorFrameEventFilter(ui));
    pWidget->setWindowTitle("Platform Manipulator");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}

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
        valter3d->setValterGroupRotationY((double)value / 100);
    }
}

void MainWindow::on_manipulatorLiknk1AscentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (platformManipulatorAndIRBumper->prepareManLink1Movement())
    {
        //ascent
        if (platformManipulatorAndIRBumper->setLink1MovementDirection(true))
        {
            platformManipulatorAndIRBumper->setLink1MotorActivated(true);
        }
    }
}

void MainWindow::on_manipulatorLiknk1DescentButton_clicked()
{

}
