#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/platformcontrolp2GUI.h>

void MainWindow::initPlatfromControlP2(Ui::MainWindow *ui)
{
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

    ui->chargerMotorDutyScrollBar->installEventFilter(new WheelEventFilter());
    ui->chargerMotorPushDurationScrollBar->installEventFilter(new WheelEventFilter());
    ui->irScannerAngleScrollBar->installEventFilter(new WheelEventFilter());
    ui->beepDurationScrollBar->installEventFilter(new WheelEventFilter());
    ui->beepDurationScrollBar->installEventFilter(new WheelEventFilter());

}

//PLATFROM-CONTROL-P2
void MainWindow::on_chargerMotorDutyScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_chargerMotorDutyScrollBar_valueChanged@%d", value));
    }
    platformControlP2->setChargerMotorDutyPresetCur(value);
    ui->chargerMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_chargerMotorPushDurationScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_chargerMotorPushDurationScrollBar_valueChanged@%d", value));
    }
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
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_chargerLedsButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    platformControlP2->setChargerLeds(checked);
}

void MainWindow::on_loadDefaultsPlatfromControlP2Button_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_loadDefaultsPlatfromControlP2Button_clicked");
    }
    platformControlP2->loadDefaults();
    loadPlatformControlP2Defaults(ui);
}


void MainWindow::on_platformControlP2LeftWheelEncoderCheckBox_clicked(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_platformControlP2LeftWheelEncoderCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}


void MainWindow::on_platformControlP2RightWheelEncoderCheckBox_clicked(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_platformControlP2RightWheelEncoderCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_platformControlP2LeftWheelEncoderAutoresetCheckBox_clicked(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_platformControlP2LeftWheelEncoderAutoresetCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_platformControlP2RightWheelEncoderAutoresetCheckBox_clicked(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_platformControlP2RightWheelEncoderAutoresetCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
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
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_platformControlP2LeftWheelEncoderResetButton_clicked");
    }
    platformControlP2->resetLeftEncoder();
}

void MainWindow::on_platformControlP2RightWheelEncoderResetButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_platformControlP2RightWheelEncoderResetButton_clicked");
    }
    platformControlP2->resetRightEncoder();
}

void MainWindow::on_beepDurationScrollBar_valueChanged(int value)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_beepDurationScrollBar_valueChanged@%d", value));
    }
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
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_platformControlP2LeftWheelEncoderGetButton_clicked");
    }
    platformControlP2->getLeftEncoderOnce(true);
}

void MainWindow::on_platformControlP2RightWheelEncoderGetButton_clicked()
{
    platformControlP2EncodersRefreshTimer->start(200);
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_platformControlP2RightWheelEncoderGetButton_clicked");
    }
    platformControlP2->getRightEncoderOnce(true);
}

void MainWindow::on_irScanningButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_irScanningButton_toggled@%s", (!checked) ? "true" : "false"));
    }
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
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_irScannerAngleScrollBar_valueChanged@%d", value));
    }
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
        platformControlP2->disableIRScannerServo();
        platformControlP2IRScannerRefreshTimer->stop();
    }
}

void MainWindow::on_chargerMotorPushCCWButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_chargerMotorPushCCWButton_clicked");
    }
    platformControlP2->setChargerMotorDirection(true);
    platformControlP2->pushChargerMotor();
}

void MainWindow::on_chargerMotorPushCWButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_chargerMotorPushCWButton_clicked");
    }
    platformControlP2->setChargerMotorDirection(false);
    platformControlP2->pushChargerMotor();
}

void MainWindow::on_chargerMotorRotateCCWButton_pressed()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_chargerMotorRotateCCWButton_pressed");
    }
    platformControlP2->setChargerMotorDirection(true);
    platformControlP2->setChargerMotorRotateWorkerActivated(true);
}

void MainWindow::on_chargerMotorRotateCWButton_pressed()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_chargerMotorRotateCWButton_pressed");
    }
    platformControlP2->setChargerMotorDirection(false);
    platformControlP2->setChargerMotorRotateWorkerActivated(true);
}

void MainWindow::on_chargerMotorRotateCCWButton_released()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_chargerMotorRotateCCWButton_released");
    }
    platformControlP2->setChargerMotorRotateWorkerActivated(false);
}

void MainWindow::on_chargerMotorRotateCWButton_released()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_chargerMotorRotateCWButton_released");
    }
    platformControlP2->setChargerMotorRotateWorkerActivated(false);
}

void MainWindow::on_beepButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_beepButton_clicked");
    }
    platformControlP2->alarmBeep();
}

void MainWindow::on_alarmButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_alarmButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    platformControlP2->ALARMOnOff(checked);
}

void MainWindow::on_bottomFronLedsButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_bottomFronLedsButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    platformControlP2->setBottomFrontLeds(checked);
}

void MainWindow::on_bottomRearLedsButton_toggled(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_bottomRearLedsButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    platformControlP2->setBottomRearLeds(checked);
}

void MainWindow::on_platformControlP2ResetIRScannerServoButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_platformControlP2ResetIRScannerServoButton_clicked");
    }
    platformControlP2->resetIRScannerServo();
}

void MainWindow::on_platformControlP2GetBottomIRReadingButton_clicked()
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand("on_platformControlP2GetBottomIRReadingButton_clicked");
    }
    platformControlP2->requestIrScannerReadingADC();
}

void MainWindow::on_platfromControlP2IRScannerADCCheckBox_clicked(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_platfromControlP2IRScannerADCCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}


void MainWindow::on_platfromControlP2RedrawGUICheckBox_clicked(bool checked)
{
    PlatformControlP2 *platformControlP2 = PlatformControlP2::getInstance();
    if (ui->platformControlP2RemoteControlCheckbox->isChecked())
    {
        platformControlP2->sendTCPCommand(Valter::format_string("on_platfromControlP2RedrawGUICheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}
