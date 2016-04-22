#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include<gui/guihelpers.h>

#include <valter.h>

#include <gui/armcontrolleftGUI.h>

void MainWindow::initArmControlLeft(Ui::MainWindow *ui)
{
    armControlLeftTabRefreshTimer = new QTimer(this);
    connect(armControlLeftTabRefreshTimer, SIGNAL(timeout()), this, SLOT(armControlLeftTabRefreshTimerUpdate()));
    armControlLeftTabRefreshTimer->start(50);

    ui->leftForearmMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->leftForearmMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftForearmAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftArmMotorDutyScroller_2->installEventFilter(new WheelEventFilter());
    ui->leftArmMotorDecelerationScroller_2->installEventFilter(new WheelEventFilter());
    ui->leftArmMotorAccelerationScroller_2->installEventFilter(new WheelEventFilter());
    ui->leftLimbMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->leftLimbMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftLimbMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftForearmRollStepDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->leftForearmRollStepSwitchDelaySpinBox->installEventFilter(new WheelEventFilter());
}

void MainWindow::armControlLeftTabRefreshTimerUpdate()
{
    armControlLeftTabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_leftForearmMotorDutyScroller_valueChanged(int value)
{
    ui->leftForearmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftForearmMotorDutyMax(value);
}

void MainWindow::on_leftForearmMotorDecelerationScroller_valueChanged(int value)
{
    ui->leftForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftForearmMotorDeceleration(value);
}

void MainWindow::on_leftForearmAccelerationScroller_valueChanged(int value)
{
    ui->leftForearmAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftForearmMotorAcceleration(value);
}

void MainWindow::on_leftArmMotorDutyScroller_2_valueChanged(int value)
{
    ui->leftArmMotorDutyLabel_2->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftArmMotorDutyMax(value);
}

void MainWindow::on_leftArmMotorDecelerationScroller_2_valueChanged(int value)
{
    ui->leftArmMotorDecelerationLabel_2->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftArmMotorDeceleration(value);
}

void MainWindow::on_leftArmMotorAccelerationScroller_2_valueChanged(int value)
{
    ui->leftArmMotorAccelerationLabel_2->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftArmMotorAcceleration(value);
}

void MainWindow::on_leftLimbMotorDutyScroller_valueChanged(int value)
{
    ui->leftLimbMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftLimbMotorDutyMax(value);
}

void MainWindow::on_leftLimbMotorDecelerationScroller_valueChanged(int value)
{
    ui->leftLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftLimbMotorDeceleration(value);
}

void MainWindow::on_leftLimbMotorAccelerationScroller_valueChanged(int value)
{
    ui->leftLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftLimbMotorAcceleration(value);
}

void MainWindow::on_leftForearmMoveUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->prepareLeftForearmMovement())
    {
        //up
        if (armControlLeft->setLeftForearmMotorMovementDirection(false))
        {
            armControlLeft->setLeftForearmMotorActivated(true);
        }
    }
}

void MainWindow::on_leftForearmMoveUpButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftForearmMotorActivated(false);
}

void MainWindow::on_leftForearmMoveDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->prepareLeftForearmMovement())
    {
        //down
        if (armControlLeft->setLeftForearmMotorMovementDirection(true))
        {
            armControlLeft->setLeftForearmMotorActivated(true);
        }
    }
}

void MainWindow::on_leftForearmMoveDownButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftForearmMotorActivated(false);
}

void MainWindow::on_leftArmMoveUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->prepareLeftArmMovement())
    {
        //up
        if (armControlLeft->setLeftArmMotorMovementDirection(false))
        {
            armControlLeft->setLeftArmMotorActivated(true);
        }
    }
}

void MainWindow::on_leftArmMoveUpButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftArmMotorActivated(false);
}

void MainWindow::on_leftArmMoveDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->prepareLeftArmMovement())
    {
        //down
        if (armControlLeft->setLeftArmMotorMovementDirection(true))
        {
            armControlLeft->setLeftArmMotorActivated(true);
        }
    }
}

void MainWindow::on_leftArmMoveDownButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftArmMotorActivated(false);
}

void MainWindow::on_leftLimbMoveUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->prepareLeftLimbMovement())
    {
        //up
        if (armControlLeft->setLeftLimbMotorMovementDirection(false))
        {
            armControlLeft->setLeftLimbMotorActivated(true);
        }
    }
}

void MainWindow::on_leftLimbMoveUpButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftLimbMotorActivated(false);
}

void MainWindow::on_leftLimbMoveDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->prepareLeftLimbMovement())
    {
        //down
        if (armControlLeft->setLeftLimbMotorMovementDirection(true))
        {
            armControlLeft->setLeftLimbMotorActivated(true);
        }
    }
}

void MainWindow::on_leftLimbMoveDownButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setLeftLimbMotorActivated(false);
}

void MainWindow::on_leftHandYawCCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setHandYawDirection(false);
    armControlLeft->handYaw(true);
}

void MainWindow::on_leftHandYawCCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->handYaw(false);
}

void MainWindow::on_leftHandYawCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setHandYawDirection(true);
    armControlLeft->handYaw(true);
}

void MainWindow::on_leftHandYawCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->handYaw(false);
}

void MainWindow::on_leftHandPitchUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setHandPitchDirection(true);
    armControlLeft->handPitch(true);
}

void MainWindow::on_leftHandPitchUpButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->handPitch(false);
}

void MainWindow::on_leftHandPitchDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setHandPitchDirection(false);
    armControlLeft->handPitch(true);
}

void MainWindow::on_leftHandPitchDownButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->handPitch(false);
}

void MainWindow::on_leftForearmRollMotorOnOffButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->getForearmRollMotorState())
    {
        armControlLeft->setForearmRollMotorOnOff(false);
    }
    else
    {
        armControlLeft->setForearmRollMotorOnOff(true);
    }
}

void MainWindow::on_leftForearmRollCCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->getForearmRollMotorState())
    {
        armControlLeft->setForearmRollDirection(false); //CCW
        armControlLeft->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_leftForearmRollCCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmRollMotorActivated(false);
}

void MainWindow::on_leftForearmRollCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (armControlLeft->getForearmRollMotorState())
    {
        armControlLeft->setForearmRollDirection(true); //CW
        armControlLeft->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_leftForearmRollCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmRollMotorActivated(false);
}

void MainWindow::on_leftForearmRollStepDelaySpinBox_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmRollStepDelay(value);
}

void MainWindow::on_leftForearmRollStepSwitchDelaySpinBox_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmRollStepSwitchDelay(value);
}

void MainWindow::on_leftForearmResetPositionButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmRollResettingStepPosition(!armControlLeft->getForearmRollResettingStepPosition());
}

void MainWindow::on_leftArmLedsOnOffButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->turnArmLedsOnOff();
}

void MainWindow::on_armControlLeftLoadDefaultsButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->loadDefaults();
    loadArmControlLeftDefaults(ui);
}

void MainWindow::on_leftArmReadingsTable_itemClicked(QTableWidgetItem *item)
{
    setLeftArmReadingsPresets(item);
}

void MainWindow::on_leftHandSensorsTable_itemClicked(QTableWidgetItem *item)
{
    setLeftHandForceSensorsReadingsPresets(item);
}

void MainWindow::on_leftForearmYawCCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmYawDirection(false);
    armControlLeft->forearmYaw(true);
}

void MainWindow::on_leftForearmYawCCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->forearmYaw(false);
}

void MainWindow::on_leftForearmYawCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->setForearmYawDirection(true);
    armControlLeft->forearmYaw(true);
}

void MainWindow::on_leftForearmYawCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->forearmYaw(false);
}
