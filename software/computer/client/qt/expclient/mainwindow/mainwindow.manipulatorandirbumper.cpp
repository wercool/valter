#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/platformmanipulatorandirbumperGUI.h>
#include <gui/link1endpointviewitem.h>
#include <gui/link2endpointviewitem.h>
#include <gui/link3endpointviewitem.h>

//PLATFORM-MANIPULATOR-AND-IR-BUMPER

void MainWindow::initPlatformManipulatorAndIRBumper(Ui::MainWindow *ui)
{
    platformManipulatorAndIRBumperRefreshTimer = new QTimer(this);
    connect(platformManipulatorAndIRBumperRefreshTimer, SIGNAL(timeout()), this, SLOT(platformManipulatorAndIRBumperRefreshTimerUpdate()));

    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene = new QGraphicsScene;
    platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene->setSceneRect(0, 0, 246, 246);
    ui->platformManipulatorAndIRBumperLink1Link2PositionGraphicsView->setScene(platformManipulatorAndIRBumperLink1Link2PositionGraphicsViewScene);
    ui->platformManipulatorAndIRBumperLink1Link2PositionGraphicsView->setRenderHint(QPainter::Antialiasing);

    ui->manipulatorLiknk1MotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->manipulatorLiknk1DecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->manipulatorLiknk1AccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->manipulatorLiknk2MotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->manipulatorLiknk2DecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->manipulatorLiknk2AccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->manGripperRotationMotorDutyScroller->installEventFilter(new WheelEventFilter());

    ui->irBumperFrequencySpin->installEventFilter(new WheelEventFilter());
    ui->irBumperDutySpin->installEventFilter(new WheelEventFilter());

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

    platformManipulatorAndIRBumperRefreshTimer->start(50);
}

void MainWindow::platformManipulatorAndIRBumperRefreshTimerUpdate()
{
    platformManipulatorAndIRBumperRefreshTimerUpdateWorker(ui);
}

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
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_platformManipulatorExecuteAngleSettingsButton_clicked");
    }

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

void MainWindow::on_powerOnOff24VPlatfromManipulatorAndIRBumperButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_powerOnOff24VPlatfromManipulatorAndIRBumperButton_clicked");
    }
    platformManipulatorAndIRBumper->setPower24VOnOff(!platformManipulatorAndIRBumper->getPower24VOnOff());
}

void MainWindow::on_manipulatorLiknk1AscentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk1AscentButton_pressed");
    }
    if (platformManipulatorAndIRBumper->prepareManLink1Movement())
    {
        //ascent
        if (platformManipulatorAndIRBumper->setLink1MovementDirection(true))
        {
            platformManipulatorAndIRBumper->setLink1MotorActivated(true);
        }
    }
}

void MainWindow::on_manipulatorLiknk1AscentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk1AscentButton_released");
    }
    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
}

void MainWindow::on_manipulatorLiknk1DescentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk1DescentButton_pressed");
    }
    if (platformManipulatorAndIRBumper->prepareManLink1Movement())
    {
        //descent
        if (platformManipulatorAndIRBumper->setLink1MovementDirection(false))
        {
            platformManipulatorAndIRBumper->setLink1MotorActivated(true);
        }
    }
}

void MainWindow::on_manipulatorLiknk1DescentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk1DescentButton_released");
    }
    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
}

void MainWindow::on_platformManipulatorAndIRBumperLoadDefaultsButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_platformManipulatorAndIRBumperLoadDefaultsButton_clicked");
    }
    platformManipulatorAndIRBumper->loadDefaults();
    loadPlatformManipulatorAndIRBumperDefaults(ui);
}


void MainWindow::on_platformManipulatorAndIRBumperRedrawGUICheckBox_clicked(bool checked)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_platformManipulatorAndIRBumperRedrawGUICheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_manipulatorLiknk1MotorDutyScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorLiknk1MotorDutyScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setLink1MotorDutyMax(value);
    ui->manipulatorLiknk1MotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk1DecelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorLiknk1DecelerationScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setLink1MotorDeceleration(value);
    ui->manipulatorLiknk1DecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk1AccelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorLiknk1AccelerationScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setLink1MotorAcceleration(value);
    ui->manipulatorLiknk1AccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2MotorDutyScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorLiknk2MotorDutyScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setLink2MotorDutyMax(value);
    ui->manipulatorLiknk2MotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2DecelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorLiknk2DecelerationScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setLink2MotorDeceleration(value);
    ui->manipulatorLiknk2DecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2AccelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorLiknk2AccelerationScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setLink2MotorAcceleration(value);
    ui->manipulatorLiknk2AccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2AscentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk2AscentButton_pressed");
    }
    if (platformManipulatorAndIRBumper->prepareManLink2Movement())
    {
        //ascent
        if (platformManipulatorAndIRBumper->setLink2MovementDirection(true))
        {
            platformManipulatorAndIRBumper->setLink2MotorActivated(true);
        }
    }
}

void MainWindow::on_manipulatorLiknk2AscentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk2AscentButton_released");
    }
    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
}

void MainWindow::on_manipulatorLiknk2DescentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk2DescentButton_pressed");
    }
    if (platformManipulatorAndIRBumper->prepareManLink2Movement())
    {
        //descent
        if (platformManipulatorAndIRBumper->setLink2MovementDirection(false))
        {
            platformManipulatorAndIRBumper->setLink2MotorActivated(true);
        }
    }
}

void MainWindow::on_manipulatorLiknk2DescentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manipulatorLiknk2DescentButton_released");
    }
    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
}

void MainWindow::on_gripperTiltAscentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_gripperTiltAscentButton_pressed");
    }
    platformManipulatorAndIRBumper->manLink3TiltUp();
}

void MainWindow::on_gripperTiltAscentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_gripperTiltAscentButton_released");
    }
    platformManipulatorAndIRBumper->manLink3Stop();
}

void MainWindow::on_gripperTiltDescentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_gripperTiltDescentButton_pressed");
    }
    platformManipulatorAndIRBumper->manLink3TiltDown();
}

void MainWindow::on_gripperTiltDescentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_gripperTiltDescentButton_released");
    }
    platformManipulatorAndIRBumper->manLink3Stop();
}

void MainWindow::on_manGripperOpenButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperOpenButton_pressed");
    }
    platformManipulatorAndIRBumper->manGripperOpen();
}

void MainWindow::on_manGripperOpenButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperOpenButton_released");
    }
    platformManipulatorAndIRBumper->manGripperStop();
}

void MainWindow::on_manGripperCloseButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperCloseButton_pressed");
    }
    platformManipulatorAndIRBumper->manGripperClose();
}

void MainWindow::on_manGripperCloseButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperCloseButton_released");
    }
    platformManipulatorAndIRBumper->manGripperStop();
}

void MainWindow::on_manGripperRotationMotorDutyScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manGripperRotationMotorDutyScroller_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setManGripperRotationMotorDuty(value);
    ui->manGripperRotationMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manGripperRotateCCW_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperRotateCCW_pressed");
    }
    platformManipulatorAndIRBumper->manGripperRotateCCW();
}

void MainWindow::on_manGripperRotateCCW_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperRotateCCW_released");
    }
    platformManipulatorAndIRBumper->manGripperRotateStop();
}

void MainWindow::on_manGripperRotateCW_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperRotateCW_pressed");
    }
    platformManipulatorAndIRBumper->manGripperRotateCW();
}

void MainWindow::on_manGripperRotateCW_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_manGripperRotateCW_released");
    }
    platformManipulatorAndIRBumper->manGripperRotateStop();
}

void MainWindow::on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked(QTableWidgetItem *item)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
    setPlatformManipulatorReadingsPresets(item);
}

void MainWindow::on_irBumperEnableButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_irBumperEnableButton_clicked");
    }
    platformManipulatorAndIRBumper->setIrBumperEnabled(true);
}

void MainWindow::on_irBumperDisableButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_irBumperDisableButton_clicked");
    }
    platformManipulatorAndIRBumper->setIrBumperEnabled(false);
}

void MainWindow::on_irBumperInitButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_irBumperInitButton_clicked");
    }
    platformManipulatorAndIRBumper->setIrBumperInitialized(true);
}

void MainWindow::on_irBumperFrequencySpin_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_irBumperFrequencySpin_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setIrBumperFrequency(value);
}

void MainWindow::on_irBumperDutySpin_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_irBumperDutySpin_valueChanged@%d", value));
    }
    platformManipulatorAndIRBumper->setIrBumperDuty(value);
}

void MainWindow::on_irBumperTrackAllButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_irBumperTrackAllButton_clicked");
    }
    for (int i = 0 ; i < 16; i++)
    {
        platformManipulatorAndIRBumper->setIRBumperTrack(i, true);
        ((QTableWidgetItem*)ui->irBumperReadingsTable->item(i, 0))->setCheckState((platformManipulatorAndIRBumper->getIRBumperTrack(i)) ? Qt::Checked : Qt::Unchecked);
    }
}

void MainWindow::on_irBumperTrackNoneButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_irBumperTrackNoneButton_clicked");
    }
    for (int i = 0 ; i < 16; i++)
    {
        platformManipulatorAndIRBumper->setIRBumperTrack(i, false);
        ((QTableWidgetItem*)ui->irBumperReadingsTable->item(i, 0))->setCheckState((platformManipulatorAndIRBumper->getIRBumperTrack(i)) ? Qt::Checked : Qt::Unchecked);
    }
}

void MainWindow::on_platformManipulatorAngleASpinBox_valueChanged(double value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_platformManipulatorAngleASpinBox_valueChanged@%f", value));
    }
}

void MainWindow::on_platformManipulatorAngleBSpinBox_valueChanged(double value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_platformManipulatorAngleBSpinBox_valueChanged@%f", value));
    }
}

void MainWindow::on_platformManipulatorAngleGSpinBox_valueChanged(double value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_platformManipulatorAngleGSpinBox_valueChanged@%f", value));
    }
}

void MainWindow::on_irBumperReadingsTable_itemClicked(QTableWidgetItem *item)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand(Valter::format_string("on_irBumperReadingsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
}

void MainWindow::on_platformManipulatorAndIRBumperTrackAllReadingsButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1PositionTrack(true);
    platformManipulatorAndIRBumper->setLink2PositionTrack(true);
    platformManipulatorAndIRBumper->setLink1CurrentTrack(true);
    platformManipulatorAndIRBumper->setLink2CurrentTrack(true);
    platformManipulatorAndIRBumper->setGripperTiltTrack(true);
    platformManipulatorAndIRBumper->setGripperRotationTrack(true);
    platformManipulatorAndIRBumper->setGripperPositionTrack(true);
    platformManipulatorAndIRBumper->setGripperForceSensor1Track(true);
    platformManipulatorAndIRBumper->setGripperForceSensor2Track(true);
    platformManipulatorAndIRBumper->setGripperForceSensor3Track(true);
    platformManipulatorAndIRBumper->setGripperObjectDetectorTrack(true);
    platformManipulatorAndIRBumper->setGripperTiltMotorCurrentTrack(true);
    platformManipulatorAndIRBumper->setGripperOpenCloseMotorCurrentTrack(true);
    platformManipulatorAndIRBumper->setGripperRotationMotorCurrentTrack(true);

    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_platformManipulatorAndIRBumperTrackAllReadingsButton_clicked");
    }
    for (int i = 0 ; i < 14; i++)
    {
        QTableWidgetItem* item = ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(i, 0);
        item->setCheckState(Qt::Checked);
        setPlatformManipulatorReadingsPresets(item);
    }
    ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->viewport()->update();
}

void MainWindow::on_platformManipulatorAndIRBumperTrackNoneReadingsButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1PositionTrack(false);
    platformManipulatorAndIRBumper->setLink2PositionTrack(false);
    platformManipulatorAndIRBumper->setLink1CurrentTrack(false);
    platformManipulatorAndIRBumper->setLink2CurrentTrack(false);
    platformManipulatorAndIRBumper->setGripperTiltTrack(false);
    platformManipulatorAndIRBumper->setGripperRotationTrack(false);
    platformManipulatorAndIRBumper->setGripperPositionTrack(false);
    platformManipulatorAndIRBumper->setGripperForceSensor1Track(false);
    platformManipulatorAndIRBumper->setGripperForceSensor2Track(false);
    platformManipulatorAndIRBumper->setGripperForceSensor3Track(false);
    platformManipulatorAndIRBumper->setGripperObjectDetectorTrack(false);
    platformManipulatorAndIRBumper->setGripperTiltMotorCurrentTrack(false);
    platformManipulatorAndIRBumper->setGripperOpenCloseMotorCurrentTrack(false);
    platformManipulatorAndIRBumper->setGripperRotationMotorCurrentTrack(false);

    if (ui->platformManipulatorAndIRBumperRemoteControlCheckbox->isChecked())
    {
        platformManipulatorAndIRBumper->sendTCPCommand("on_platformManipulatorAndIRBumperTrackNoneReadingsButton_clicked");
    }
    for (int i = 0 ; i < 13; i++)
    {
        QTableWidgetItem* item = ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->item(i, 0);
        item->setCheckState(Qt::Unchecked);
        setPlatformManipulatorReadingsPresets(item);
    }
    ui->manipulatorAndIRBumperManipulatorReadingsTableWidget->viewport()->update();
}
