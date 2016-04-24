#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/armcontrolrightGUI.h>

void MainWindow::initArmControlRight(Ui::MainWindow *ui)
{
    armControlRightTabRefreshTimer = new QTimer(this);
    connect(armControlRightTabRefreshTimer, SIGNAL(timeout()), this, SLOT(armControlRightTabRefreshTimerUpdate()));
    armControlRightTabRefreshTimer->start(50);

    ui->rightForearmMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->rightForearmMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->rightForearmAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->rightArmMotorDutyScroller_2->installEventFilter(new WheelEventFilter());
    ui->rightArmMotorDecelerationScroller_2->installEventFilter(new WheelEventFilter());
    ui->rightArmMotorAccelerationScroller_2->installEventFilter(new WheelEventFilter());
    ui->rightLimbMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->rightLimbMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->rightLimbMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->rightForearmRollStepDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->rightForearmRollStepSwitchDelaySpinBox->installEventFilter(new WheelEventFilter());
}

void MainWindow::armControlRightTabRefreshTimerUpdate()
{
    armControlRightTabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_rightForearmMotorDutyScroller_valueChanged(int value)
{
    ui->rightForearmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightForearmMotorDutyMax(value);
}

void MainWindow::on_rightForearmMotorDecelerationScroller_valueChanged(int value)
{
    ui->rightForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightForearmMotorDeceleration(value);
}

void MainWindow::on_rightForearmAccelerationScroller_valueChanged(int value)
{
    ui->rightForearmAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightForearmMotorAcceleration(value);
}

void MainWindow::on_rightArmMotorDutyScroller_2_valueChanged(int value)
{
    ui->rightArmMotorDutyLabel_2->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightArmMotorDutyMax(value);
}

void MainWindow::on_rightArmMotorDecelerationScroller_2_valueChanged(int value)
{
    ui->rightArmMotorDecelerationLabel_2->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightArmMotorDeceleration(value);
}

void MainWindow::on_rightArmMotorAccelerationScroller_2_valueChanged(int value)
{
    ui->rightArmMotorAccelerationLabel_2->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightArmMotorAcceleration(value);
}

void MainWindow::on_rightLimbMotorDutyScroller_valueChanged(int value)
{
    ui->rightLimbMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightLimbMotorDutyMax(value);
}

void MainWindow::on_rightLimbMotorDecelerationScroller_valueChanged(int value)
{
    ui->rightLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightLimbMotorDeceleration(value);
}

void MainWindow::on_rightLimbMotorAccelerationScroller_valueChanged(int value)
{
    ui->rightLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightLimbMotorAcceleration(value);
}

void MainWindow::on_rightForearmMoveUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->prepareRightForearmMovement())
    {
        //up
        if (armControlRight->setRightForearmMotorMovementDirection(false))
        {
            armControlRight->setRightForearmMotorActivated(true);
        }
    }
}

void MainWindow::on_rightForearmMoveUpButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightForearmMotorActivated(false);
}

void MainWindow::on_rightForearmMoveDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->prepareRightForearmMovement())
    {
        //down
        if (armControlRight->setRightForearmMotorMovementDirection(true))
        {
            armControlRight->setRightForearmMotorActivated(true);
        }
    }
}

void MainWindow::on_rightForearmMoveDownButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightForearmMotorActivated(false);
}

void MainWindow::on_rightArmMoveUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->prepareRightArmMovement())
    {
        //up
        if (armControlRight->setRightArmMotorMovementDirection(false))
        {
            armControlRight->setRightArmMotorActivated(true);
        }
    }
}

void MainWindow::on_rightArmMoveUpButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightArmMotorActivated(false);
}

void MainWindow::on_rightArmMoveDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->prepareRightArmMovement())
    {
        //down
        if (armControlRight->setRightArmMotorMovementDirection(true))
        {
            armControlRight->setRightArmMotorActivated(true);
        }
    }
}

void MainWindow::on_rightArmMoveDownButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightArmMotorActivated(false);
}

void MainWindow::on_rightLimbMoveUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->prepareRightLimbMovement())
    {
        //up
        if (armControlRight->setRightLimbMotorMovementDirection(false))
        {
            armControlRight->setRightLimbMotorActivated(true);
        }
    }
}

void MainWindow::on_rightLimbMoveUpButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightLimbMotorActivated(false);
}

void MainWindow::on_rightLimbMoveDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->prepareRightLimbMovement())
    {
        //down
        if (armControlRight->setRightLimbMotorMovementDirection(true))
        {
            armControlRight->setRightLimbMotorActivated(true);
        }
    }
}

void MainWindow::on_rightLimbMoveDownButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setRightLimbMotorActivated(false);
}

void MainWindow::on_rightHandYawCCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setHandYawDirection(true);
    armControlRight->handYaw(true);
}

void MainWindow::on_rightHandYawCCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->handYaw(false);
}

void MainWindow::on_rightHandYawCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setHandYawDirection(false);
    armControlRight->handYaw(true);
}

void MainWindow::on_rightHandYawCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->handYaw(false);
}

void MainWindow::on_rightHandPitchUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setHandPitchDirection(true);
    armControlRight->handPitch(true);
}

void MainWindow::on_rightHandPitchUpButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->handPitch(false);
}

void MainWindow::on_rightHandPitchDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setHandPitchDirection(false);
    armControlRight->handPitch(true);
}

void MainWindow::on_rightHandPitchDownButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->handPitch(false);
}

void MainWindow::on_rightForearmRollMotorOnOffButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->getForearmRollMotorState())
    {
        armControlRight->setForearmRollMotorOnOff(false);
    }
    else
    {
        armControlRight->setForearmRollMotorOnOff(true);
    }
}

void MainWindow::on_rightForearmRollCCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->getForearmRollMotorState())
    {
        armControlRight->setForearmRollDirection(false); //CCW
        armControlRight->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_rightForearmRollCCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmRollMotorActivated(false);
}

void MainWindow::on_rightForearmRollCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (armControlRight->getForearmRollMotorState())
    {
        armControlRight->setForearmRollDirection(true); //CW
        armControlRight->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_rightForearmRollCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmRollMotorActivated(false);
}

void MainWindow::on_rightForearmRollStepDelaySpinBox_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmRollStepDelay(value);
}

void MainWindow::on_rightForearmRollStepSwitchDelaySpinBox_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmRollStepSwitchDelay(value);
}

void MainWindow::on_rightForearmResetPositionButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmRollResettingStepPosition(!armControlRight->getForearmRollResettingStepPosition());
}

void MainWindow::on_rightArmLedsOnOffButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->turnArmLedsOnOff();
}

void MainWindow::on_armControlRightLoadDefaultsButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->loadDefaults();
    loadArmControlRightDefaults(ui);
}

void MainWindow::on_rightArmReadingsTable_itemClicked(QTableWidgetItem *item)
{
    setRightArmReadingsPresets(item);
}


void MainWindow::on_rightHandSensorsTable_itemClicked(QTableWidgetItem *item)
{
    setRightHandForceSensorsReadingsPresets(item);
}

void MainWindow::on_rightForearmYawCCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmYawDirection(false);
    armControlRight->forearmYaw(true);
}

void MainWindow::on_rightForearmYawCCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->forearmYaw(false);
}

void MainWindow::on_rightForearmYawCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->setForearmYawDirection(true);
    armControlRight->forearmYaw(true);
}

void MainWindow::on_rightForearmYawCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->forearmYaw(false);
}
