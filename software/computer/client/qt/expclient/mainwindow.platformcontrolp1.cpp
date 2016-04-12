#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/platformcontrolp1GUI.h>


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
