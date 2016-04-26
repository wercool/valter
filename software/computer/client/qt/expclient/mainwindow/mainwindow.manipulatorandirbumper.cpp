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
    platformManipulatorAndIRBumper->setPower24VOnOff(!platformManipulatorAndIRBumper->getPower24VOnOff());
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

void MainWindow::on_manipulatorLiknk1AscentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
}

void MainWindow::on_manipulatorLiknk1DescentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
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
    platformManipulatorAndIRBumper->setLink1MotorActivated(false);
}

void MainWindow::on_platformManipulatorAndIRBumperLoadDefaultsButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->loadDefaults();
    loadPlatformManipulatorAndIRBumperDefaults(ui);
}

void MainWindow::on_manipulatorLiknk1MotorDutyScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1MotorDutyMax(value);
    ui->manipulatorLiknk1MotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk1DecelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1MotorDeceleration(value);
    ui->manipulatorLiknk1DecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk1AccelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink1MotorAcceleration(value);
    ui->manipulatorLiknk1AccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2MotorDutyScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink2MotorDutyMax(value);
    ui->manipulatorLiknk2MotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2DecelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink2MotorDeceleration(value);
    ui->manipulatorLiknk2DecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2AccelerationScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setLink2MotorAcceleration(value);
    ui->manipulatorLiknk2AccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manipulatorLiknk2AscentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
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
    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
}

void MainWindow::on_manipulatorLiknk2DescentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
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
    platformManipulatorAndIRBumper->setLink2MotorActivated(false);
}

void MainWindow::on_gripperTiltAscentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manLink3TiltUp();
}

void MainWindow::on_gripperTiltAscentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manLink3Stop();
}

void MainWindow::on_gripperTiltDescentButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manLink3TiltDown();
}

void MainWindow::on_gripperTiltDescentButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manLink3Stop();
}

void MainWindow::on_manGripperOpenButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperOpen();
}

void MainWindow::on_manGripperOpenButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperStop();
}

void MainWindow::on_manGripperCloseButton_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperClose();
}

void MainWindow::on_manGripperCloseButton_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperStop();
}

void MainWindow::on_manGripperRotationMotorDutyScroller_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setManGripperRotationMotorDuty(value);
    ui->manGripperRotationMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_manGripperRotateCCW_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperRotateCCW();
}

void MainWindow::on_manGripperRotateCCW_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperRotateStop();
}

void MainWindow::on_manGripperRotateCW_pressed()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperRotateCW();
}

void MainWindow::on_manGripperRotateCW_released()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->manGripperRotateStop();
}

void MainWindow::on_manipulatorAndIRBumperManipulatorReadingsTableWidget_itemClicked(QTableWidgetItem *item)
{
    setPlatformManipulatorReadingsPresets(item);
}

void MainWindow::on_irBumperEnableButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setIrBumperEnabled(true);
}

void MainWindow::on_irBumperDisableButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setIrBumperEnabled(false);
}

void MainWindow::on_irBumperInitButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setIrBumperInitialized(true);
}

void MainWindow::on_irBumperFrequencySpin_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setIrBumperFrequency(value);
}

void MainWindow::on_irBumperDutySpin_valueChanged(int value)
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    platformManipulatorAndIRBumper->setIrBumperDuty(value);
}

void MainWindow::on_irBumperTrackAllButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    for (int i = 0 ; i < 16; i++)
    {
        platformManipulatorAndIRBumper->setIRBumperTrack(i, true);
        ((QTableWidgetItem*)ui->irBumperReadingsTable->item(i, 0))->setCheckState((platformManipulatorAndIRBumper->getIRBumperTrack(i)) ? Qt::Checked : Qt::Unchecked);
    }
}

void MainWindow::on_irBumperTrackNoneButton_clicked()
{
    PlatformManipulatorAndIRBumper *platformManipulatorAndIRBumper = PlatformManipulatorAndIRBumper::getInstance();
    for (int i = 0 ; i < 16; i++)
    {
        platformManipulatorAndIRBumper->setIRBumperTrack(i, false);
        ((QTableWidgetItem*)ui->irBumperReadingsTable->item(i, 0))->setCheckState((platformManipulatorAndIRBumper->getIRBumperTrack(i)) ? Qt::Checked : Qt::Unchecked);
    }
}
