#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/platformcontrolp1GUI.h>

void MainWindow::initPlatfromControlP1(Ui::MainWindow *ui)
{
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

    platformControlP1TabRefreshTimer = new QTimer(this);
    connect(platformControlP1TabRefreshTimer, SIGNAL(timeout()), this, SLOT(platformControlP1TabRefreshTimerUpdate()));
    platformControlP1TabRefreshTimer->start(50);
}

void MainWindow::platformControlP1TabRefreshTimerUpdate()
{
    platformControlP1TabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_on5VPlatformControlP1pushButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_on5VPlatformControlP1pushButton_clicked");
    }
    platformControlP1->toggle5VSource(true);
}

void MainWindow::on_off5VPlatformControlP1pushButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_off5VPlatformControlP1pushButton_clicked");
    }
    platformControlP1->toggle5VSource(false);
}

void MainWindow::on_onLeftAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_onLeftAccumulatorPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleLeftAccumulator(true);
}

void MainWindow::on_offLeftAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_offLeftAccumulatorPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleLeftAccumulator(false);
}

void MainWindow::on_onRightAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_onRightAccumulatorPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleRightAccumulator(true);
}

void MainWindow::on_offRightAccumulatorPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_offRightAccumulatorPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleRightAccumulator(false);
}

void MainWindow::on_scan220VAOnCButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_scan220VAOnCButton_clicked");
    }
    platformControlP1->setScan220ACAvailable(true);
}

void MainWindow::on_scan220VAOffCButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_scan220VAOffCButton_clicked");
    }
    platformControlP1->setScan220ACAvailable(false);
}

void MainWindow::on_chargerVoltageADCCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_chargerVoltageADCCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_onMainAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (!platformControlP1->mainAccumulatorON())
    {
        if (ui->platformControlP1RemoteControlCheckbox->isChecked())
        {
            platformControlP1->sendTCPCommand("on_onMainAccumulatorRelayPlatformControlP1Button_clicked");
        }

        QMessageBox msgBox;
        msgBox.setText("220V AC is not connected");
        msgBox.exec();
    }
}

void MainWindow::on_offMainAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_offMainAccumulatorRelayPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleMainAccumulatorRelayState(false);
}

void MainWindow::on_onLeftAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_onLeftAccumulatorRelayPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleLeftAccumulatorRelay(true);
}

void MainWindow::on_offLeftAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_offLeftAccumulatorRelayPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleLeftAccumulatorRelay(false);
}

void MainWindow::on_onRightAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_onRightAccumulatorRelayPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleRightAccumulatorRelay(true);
}

void MainWindow::on_offRightAccumulatorRelayPlatformControlP1Button_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_offRightAccumulatorRelayPlatformControlP1Button_clicked");
    }
    platformControlP1->toggleRightAccumulatorRelay(false);
}

void MainWindow::on_chargerButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_chargerButton_clicked");
    }
    platformControlP1->chargerButtonPress();
}

void MainWindow::on_setChargeOnButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_setChargeOnButton_clicked");
    }
    platformControlP1->setChargerMode(true);
}

void MainWindow::on_setChargeOffButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_setChargeOffButton_clicked");
    }
    platformControlP1->setChargerMode(false);
}

void MainWindow::on_platformMoveStopButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveStopButton_clicked");
    }
    platformControlP1->setPlatformEmergencyStop(true);
}

void MainWindow::on_platformMoveForwardButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveForwardButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
         platformControlP1->sendTCPCommand("on_platformMoveForwardButton_released");
    }
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformBackwardForwardButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformBackwardForwardButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformBackwardForwardButton_released");
    }
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformMoveForwardLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveForwardLeftButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveForwardLeftButton_released");
    }
    platformControlP1->setLeftMotorActivated(false);
}

void MainWindow::on_platformMoveForwardRightButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveForwardRightButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveForwardRightButton_released");
    }
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformMoveBackwardLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveBackwardLeftButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveBackwardRightButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveBackwardLeftButton_released");
    }
    platformControlP1->setLeftMotorActivated(false);
}

void MainWindow::on_platformMoveBackwardRightButton_released()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformMoveBackwardRightButton_released");
    }
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformRotateLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformRotateLeftButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformRotateRightButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformRotateLeftButton_released");
    }
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_platformRotateRightButton_released()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformRotateRightButton_released");
    }
    platformControlP1->setLeftMotorActivated(false);
    platformControlP1->setRightMotorActivated(false);
}

void MainWindow::on_turretRotateLeftButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_turretRotateLeftButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_turretRotateLeftButton_released");
    }
    platformControlP1->setTurretMotorActivated(false);
}

void MainWindow::on_turretRotateRightButton_pressed()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_turretRotateRightButton_pressed");
    }
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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_bodyRotationStopButton_clicked");
    }
    platformControlP1->setTurretEmergencyStop(true);
}

void MainWindow::on_turretRotateRightButton_released()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_turretRotateRightButton_released");
    }
    platformControlP1->setTurretMotorActivated(false);
}

void MainWindow::on_platformControlP1LoadDefaultsButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1LoadDefaultsButton_clicked");
    }
    platformControlP1->loadDefaults();
    loadPlatformControlP1Defaults(ui);
}

void MainWindow::on_platformControlP1MotorsPWMFrequencySetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1MotorsPWMFrequencySetButton_clicked");
    }
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

void MainWindow::on_platformControlP1LeftWheelEncoderResetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1LeftWheelEncoderResetButton_clicked");
    }
    platformControlP1->resetLeftWheelEncoder();
}

void MainWindow::on_platformControlP1RightWheelEncoderResetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1RightWheelEncoderResetButton_clicked");
    }
    platformControlP1->resetRightWheelEncoder();
}

void MainWindow::on_platformControlP1LeftWheelEncoderGetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1LeftWheelEncoderGetButton_clicked");
    }
    ui->platformControlP1LeftWheelEncoderLcdNumber->display("");
    platformControlP1->setLeftWheelEncoderGetOnce(true);
}

void MainWindow::on_platformControlP1RightWheelEncoderGetButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1RightWheelEncoderGetButton_clicked");
    }
    ui->platformControlP1RightWheelEncoderLcdNumber->display("");
    platformControlP1->setRightWheelEncoderGetOnce(true);
}

void MainWindow::on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1LeftWheelEncoderAutoresetCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
    platformControlP1->setLeftWheelEncoderAutoreset(checked);
}

void MainWindow::on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1RightWheelEncoderAutoresetCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
    platformControlP1->setRightWheelEncoderAutoreset(checked);
}

void MainWindow::on_getBodyRotationPositionButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_getBodyRotationPositionButton_clicked");
    }
    ui->turretPositionLcdNumber->display("");
    platformControlP1->setTurretPositionGetOnce(true);
}

void MainWindow::on_platformControlP1ReadingsTable_itemClicked(QTableWidgetItem *item)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1ReadingsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
    setPlatfromControlP1AdditionalReadings(item);
}

void MainWindow::on_leftMotorPlatformControlP1DutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_leftMotorPlatformControlP1DutySlider_valueChanged@%d", value));
    }
    platformControlP1->setLeftMotorDutyMax(value);
    ui->leftMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getLeftMotorDutyMax()).c_str());
}

void MainWindow::on_rightMotorPlatformControlP1DutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_rightMotorPlatformControlP1DutySlider_valueChanged@%d", value));
    }
    platformControlP1->setRightMotorDutyMax(value);
    ui->rightMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getRightMotorDutyMax()).c_str());
}

void MainWindow::on_platformMovementAccelerationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformMovementAccelerationSlider_valueChanged@%d", value));
    }
    platformControlP1->setPlatformAcceleration(value);
    ui->platformControlP1WheelMotorsAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_platformMovementDecelerationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformMovementDecelerationSlider_valueChanged@%d", value));
    }
    platformControlP1->setPlatformDeceleration(value);
    ui->platformControlP1WheelMotorsDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_platformControlP1WheelMotorsDutySlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1WheelMotorsDutySlider_valueChanged@%d", value));
    }

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
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_turretRotationDutySlider_valueChanged@%d", value));
    }
    platformControlP1->setTurretMotorDutyMax(value);
    ui->turretMotorMaxDutyLabel->setText(Valter::format_string("[%d]", platformControlP1->getTurretMotorDutyMax()).c_str());
}

void MainWindow::on_decelerationTurretRotationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_decelerationTurretRotationSlider_valueChanged@%d", value));
    }
    platformControlP1->setTurretDeceleration(value);
    ui->platformControlP1TurretMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_accelerationTurretRotationSlider_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_accelerationTurretRotationSlider_valueChanged@%d", value));
    }
    platformControlP1->setTurretAcceleration(value);
    ui->platformControlP1TurretMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_platformControlP1additionalReadingsTrackingDelay_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1additionalReadingsTrackingDelay_valueChanged@%d", value));
    }
    platformControlP1->setAdditionalReadingsDelayCur(value);
    ui->platformControlP1additionalReadingsTrackingDelayLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_valterPlatformMovementControlsDetachButton_clicked()
{
    QWidget* pWidget = ui->valterPlatformMovementControlsFrame;
    pWidget->resize(pWidget->width() + 40, pWidget->height() + 40);
    pWidget->layout()->setMargin(20);
    pWidget->installEventFilter(new ValterPlatformMovementControlsEventFilter(ui));
    pWidget->setWindowTitle("Platform Movement");
    pWidget->setParent(pMainWindow->getInstance(), Qt::Window);
    pWidget->show();
}

void MainWindow::on_wheelMotorsCurrentADCCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_wheelMotorsCurrentADCCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_leftMotorCurrentCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_leftMotorCurrentCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_rightMotorCurrentCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_rightMotorCurrentCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_platformControlP1MotorsPWMFrequencySpinBox_valueChanged(int value)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1MotorsPWMFrequencySpinBox_valueChanged@%d", value));
    }
}

void MainWindow::on_platformControlP1LeftWheelEncoderCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1LeftWheelEncoderCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_platformControlP1RightWheelEncoderCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1RightWheelEncoderCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_turretMotorCurrentCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_turretMotorCurrentCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
    platformControlP1->setTurretMotorCurrentRead(checked);
}

void MainWindow::on_turretMotorCurrentADCCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_turretMotorCurrentADCCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_turretPositionReadCheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_turretPositionReadCheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_platformControlP1RedrawGUICheckBox_clicked(bool checked)
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand(Valter::format_string("on_platformControlP1RedrawGUICheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_platformControlP1UntrackAllAdditionalReadingsButton_clicked()
{
    PlatformControlP1 *platformControlP1 = PlatformControlP1::getInstance();
    if (ui->platformControlP1RemoteControlCheckbox->isChecked())
    {
        platformControlP1->sendTCPCommand("on_platformControlP1UntrackAllAdditionalReadingsButton_clicked");
    }
    for (int i = 0; i < ui->platformControlP1ReadingsTable->rowCount() - 1; i++)
    {
        QTableWidgetItem* item = ui->platformControlP1ReadingsTable->item(i, 3);
        item->setCheckState(Qt::Unchecked);
    }
    ui->platformControlP1ReadingsTable->viewport()->update();
}
