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
    ui->armControlLeftArmMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftArmMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftArmMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftLimbMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->leftLimbMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftLimbMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftForearmRollStepDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->leftForearmRollStepSwitchDelaySpinBox->installEventFilter(new WheelEventFilter());

    ui->armControlLeftFinger6PositionScoller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftFinger7PositionScoller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftFinger8PositionScoller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftFinger9PositionScoller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftFinger10PositionScoller->installEventFilter(new WheelEventFilter());
    ui->armControlLeftFinger11PositionScoller->installEventFilter(new WheelEventFilter());
}

void MainWindow::armControlLeftTabRefreshTimerUpdate()
{
    armControlLeftTabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_leftForearmMotorDutyScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftForearmMotorDutyScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftForearmMotorDutyMax(value);
    ui->leftForearmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftForearmMotorDecelerationScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftForearmMotorDecelerationScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftForearmMotorDeceleration(value);
    ui->leftForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftForearmAccelerationScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftForearmAccelerationScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftForearmMotorAcceleration(value);
    ui->leftForearmAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_armControlLeftArmMotorDutyScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftArmMotorDutyScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftArmMotorDutyMax(value);
    ui->armControlLeftArmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_armControlLeftArmMotorDecelerationScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftArmMotorDecelerationScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftArmMotorDeceleration(value);
    ui->armControlLeftArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_armControlLeftArmMotorAccelerationScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftArmMotorAccelerationScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftArmMotorAcceleration(value);
    ui->armControlLeftArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftLimbMotorDutyScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftLimbMotorDutyScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftLimbMotorDutyMax(value);
    ui->leftLimbMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftLimbMotorDecelerationScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftLimbMotorDecelerationScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftLimbMotorDeceleration(value);
    ui->leftLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftLimbMotorAccelerationScroller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftLimbMotorAccelerationScroller_valueChanged@%d", value));
    }
    armControlLeft->setLeftLimbMotorAcceleration(value);
    ui->leftLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftForearmMoveUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmMoveUpButton_pressed");
    }
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
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmMoveUpButton_released");
    }
    armControlLeft->setLeftForearmMotorActivated(false);
}

void MainWindow::on_leftForearmMoveDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmMoveDownButton_pressed");
    }
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
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmMoveDownButton_released");
    }
    armControlLeft->setLeftForearmMotorActivated(false);
}

void MainWindow::on_leftArmMoveUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmMoveUpButton_pressed");
    }
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
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmMoveUpButton_released");
    }
    armControlLeft->setLeftArmMotorActivated(false);
}

void MainWindow::on_leftArmMoveDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmMoveDownButton_pressed");
    }
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
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmMoveDownButton_released");
    }
    armControlLeft->setLeftArmMotorActivated(false);
}

void MainWindow::on_leftLimbMoveUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftLimbMoveUpButton_pressed");
    }
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
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftLimbMoveUpButton_released");
    }
    armControlLeft->setLeftLimbMotorActivated(false);
}

void MainWindow::on_leftLimbMoveDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftLimbMoveDownButton_pressed");
    }
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
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftLimbMoveDownButton_released");
    }
    armControlLeft->setLeftLimbMotorActivated(false);
}

void MainWindow::on_leftHandYawCCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandYawCCWButton_pressed");
    }
    armControlLeft->setHandYawDirection(false);
    armControlLeft->handYaw(true);
}

void MainWindow::on_leftHandYawCCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandYawCCWButton_released");
    }
    armControlLeft->handYaw(false);
}

void MainWindow::on_leftHandYawCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandYawCWButton_pressed");
    }
    armControlLeft->setHandYawDirection(true);
    armControlLeft->handYaw(true);
}

void MainWindow::on_leftHandYawCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandYawCWButton_released");
    }
    armControlLeft->handYaw(false);
}

void MainWindow::on_leftHandPitchUpButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandPitchUpButton_pressed");
    }
    armControlLeft->setHandPitchDirection(true);
    armControlLeft->handPitch(true);
}

void MainWindow::on_leftHandPitchUpButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandPitchUpButton_released");
    }
    armControlLeft->handPitch(false);
}

void MainWindow::on_leftHandPitchDownButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandPitchDownButton_pressed");
    }
    armControlLeft->setHandPitchDirection(false);
    armControlLeft->handPitch(true);
}

void MainWindow::on_leftHandPitchDownButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandPitchDownButton_released");
    }
    armControlLeft->handPitch(false);
}

void MainWindow::on_leftForearmRollMotorOnButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmRollMotorOnButton_clicked");
    }
    armControlLeft->setForearmRollMotorOnOff(true);
}

void MainWindow::on_leftForearmRollMotorOffButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmRollMotorOffButton_clicked");
    }
    armControlLeft->setForearmRollMotorOnOff(false);
}

void MainWindow::on_leftForearmRollCCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmRollCCWButton_pressed");
    }
    if (armControlLeft->getForearmRollMotorState())
    {
        armControlLeft->setForearmRollDirection(false); //CCW
        armControlLeft->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_leftForearmRollCCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmRollCCWButton_released");
    }
    armControlLeft->setForearmRollMotorActivated(false);
}

void MainWindow::on_leftForearmRollCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmRollCWButton_pressed");
    }
    if (armControlLeft->getForearmRollMotorState())
    {
        armControlLeft->setForearmRollDirection(true); //CW
        armControlLeft->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_leftForearmRollCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmRollCWButton_released");
    }
    armControlLeft->setForearmRollMotorActivated(false);
}

void MainWindow::on_leftForearmRollStepDelaySpinBox_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftForearmRollStepDelaySpinBox_valueChanged@%d", value));
    }
    armControlLeft->setForearmRollStepDelay(value);
}

void MainWindow::on_leftForearmRollStepSwitchDelaySpinBox_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftForearmRollStepSwitchDelaySpinBox_valueChanged@%d", value));
    }
    armControlLeft->setForearmRollStepSwitchDelay(value);
}

void MainWindow::on_leftForearmResetPositionButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmResetPositionButton_clicked");
    }
    armControlLeft->setForearmRollResettingStepPosition(!armControlLeft->getForearmRollResettingStepPosition());
}

void MainWindow::on_leftArmLedsOnOffButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmLedsOnOffButton_clicked");
    }
    armControlLeft->turnArmLedsOnOff();
}

void MainWindow::on_armControlLeftLoadDefaultsButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftLoadDefaultsButton_clicked");
    }
    armControlLeft->loadDefaults();
    loadArmControlLeftDefaults(ui);
}

void MainWindow::on_leftArmReadingsTable_itemClicked(QTableWidgetItem *item)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftArmReadingsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
    setLeftArmReadingsPresets(item);
}

void MainWindow::on_leftHandSensorsTable_itemClicked(QTableWidgetItem *item)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_leftHandSensorsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
    setLeftHandForceSensorsReadingsPresets(item);
}

void MainWindow::on_leftForearmYawCCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmYawCCWButton_pressed");
    }
    armControlLeft->setForearmYawDirection(false);
    armControlLeft->forearmYaw(true);
}

void MainWindow::on_leftForearmYawCCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmYawCCWButton_released");
    }
    armControlLeft->forearmYaw(false);
}

void MainWindow::on_leftForearmYawCWButton_pressed()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmYawCWButton_pressed");
    }
    armControlLeft->setForearmYawDirection(true);
    armControlLeft->forearmYaw(true);
}

void MainWindow::on_leftForearmYawCWButton_released()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftForearmYawCWButton_released");
    }
    armControlLeft->forearmYaw(false);
}

void MainWindow::on_leftHandSensorsTrackAllButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandSensorsTrackAllButton_clicked");
    }
    for (int i = 0; i < 13; i++)
    {
        armControlLeft->setHandSensorsTrack(i, true);
        ((QTableWidgetItem*)ui->leftHandSensorsTable->item(i, 0))->setCheckState(Qt::Checked);
    }
    ui->leftHandSensorsTable->viewport()->update();
}

void MainWindow::on_leftHandSensorsTrackNoneButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftHandSensorsTrackNoneButton_clicked");
    }
    for (int i = 0; i < 13; i++)
    {
        armControlLeft->setHandSensorsTrack(i, false);
        ((QTableWidgetItem*)ui->leftHandSensorsTable->item(i, 0))->setCheckState(Qt::Unchecked);
    }
    ui->leftHandSensorsTable->viewport()->update();
}

void MainWindow::on_leftArmReadingsTrackAllButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmReadingsTrackAllButton_clicked");
    }
    armControlLeft->setForearmPositionTrack(true);
    armControlLeft->setArmPositionTrack(true);
    armControlLeft->setLimbPositionTrack(true);
    armControlLeft->setForearmMotorCurrentTrack(true);
    armControlLeft->setArmMotorCurrentTrack(true);
    armControlLeft->setLimbMotorCurrentTrack(true);
    armControlLeft->setHandYawPositionTrack(true);
    armControlLeft->setHandPitchPositionTrack(true);
    armControlLeft->setForearmYawPositionTrack(true);
    armControlLeft->setForearmYawMotorCurrentTrack(true);
    armControlLeft->setHandYawMotorCurrentTrack(true);
    armControlLeft->setHandPitchMotorCurrentTrack(true);
    for (int i = 0; i < 12; i++)
    {
        ((QTableWidgetItem*)ui->leftArmReadingsTable->item(i, 0))->setCheckState(Qt::Checked);
    }
    ui->leftArmReadingsTable->viewport()->update();
}

void MainWindow::on_leftArmReadingsTrackNoneButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_leftArmReadingsTrackNoneButton_clicked");
    }
    armControlLeft->setForearmPositionTrack(false);
    armControlLeft->setArmPositionTrack(false);
    armControlLeft->setLimbPositionTrack(false);
    armControlLeft->setForearmMotorCurrentTrack(false);
    armControlLeft->setArmMotorCurrentTrack(false);
    armControlLeft->setLimbMotorCurrentTrack(false);
    armControlLeft->setHandYawPositionTrack(false);
    armControlLeft->setHandPitchPositionTrack(false);
    armControlLeft->setForearmYawPositionTrack(false);
    armControlLeft->setForearmYawMotorCurrentTrack(false);
    armControlLeft->setHandYawMotorCurrentTrack(false);
    armControlLeft->setHandPitchMotorCurrentTrack(false);
    for (int i = 0; i < 12; i++)
    {
        ((QTableWidgetItem*)ui->leftArmReadingsTable->item(i, 0))->setCheckState(Qt::Unchecked);
    }
    ui->leftArmReadingsTable->viewport()->update();
}

void MainWindow::on_armControlLeftStopAllWatchersButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftStopAllWatchersButton_clicked");
    }
    armControlLeft->stopAllWatchers();
}

void MainWindow::on_armControlLeftStartAllWatchersButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftStartAllWatchersButton_clicked");
    }
    armControlLeft->startAllWatchers();
}


void MainWindow::on_armControlLeftRedrawGUICheckBox_clicked(bool checked)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftRedrawGUICheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

//-----------------------------------FINGERS


void MainWindow::on_armControlLeftPalmReleaseButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftPalmReleaseButton_clicked");
    }
    armControlLeft->releaseAllFingers();
}

void MainWindow::on_armControlLeftPalmActivatedButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftPalmActivatedButton_clicked");
    }
    armControlLeft->fingersToInitialPositions();
}

void MainWindow::on_armControlLeftPalmGraspButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftPalmGraspButton_clicked");
    }
    armControlLeft->fingersGrasp();
}

void MainWindow::on_armControlLeftPalmSqueezeButton_clicked()
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (ui->armControlLeftRemoteControlCheckbox->isChecked())
    {
        armControlLeft->sendTCPCommand("on_armControlLeftPalmSqueezeButton_clicked");
    }
}

void MainWindow::on_armControlLeftFinger6PositionScoller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->getDefaultsLoading())
    {
        if (ui->armControlLeftRemoteControlCheckbox->isChecked())
        {
            armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftFinger6PositionScoller_valueChanged@%d", value));
        }
        armControlLeft->setFingerPosition(6, value);
    }
}

void MainWindow::on_armControlLeftFinger7PositionScoller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->getDefaultsLoading())
    {
        if (ui->armControlLeftRemoteControlCheckbox->isChecked())
        {
            armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftFinger7PositionScoller_valueChanged@%d", value));
        }
        armControlLeft->setFingerPosition(7, value);
    }
}

void MainWindow::on_armControlLeftFinger8PositionScoller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->getDefaultsLoading())
    {
        if (ui->armControlLeftRemoteControlCheckbox->isChecked())
        {
            armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftFinger8PositionScoller_valueChanged@%d", value));
        }
        armControlLeft->setFingerPosition(8, value);
    }
}

void MainWindow::on_armControlLeftFinger9PositionScoller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->getDefaultsLoading())
    {
        if (ui->armControlLeftRemoteControlCheckbox->isChecked())
        {
            armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftFinger9PositionScoller_valueChanged@%d", value));
        }
        armControlLeft->setFingerPosition(9, value);
    }
}

void MainWindow::on_armControlLeftFinger10PositionScoller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->getDefaultsLoading())
    {
        if (ui->armControlLeftRemoteControlCheckbox->isChecked())
        {
            armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftFinger10PositionScoller_valueChanged@%d", value));
        }
        armControlLeft->setFingerPosition(10, value);
    }
}

void MainWindow::on_armControlLeftFinger11PositionScoller_valueChanged(int value)
{
    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    if (!armControlLeft->getDefaultsLoading())
    {
        if (ui->armControlLeftRemoteControlCheckbox->isChecked())
        {
            armControlLeft->sendTCPCommand(Valter::format_string("on_armControlLeftFinger11PositionScoller_valueChanged@%d", value));
        }
        armControlLeft->setFingerPosition(11, value);
    }
}

void MainWindow::on_armControlLeftFinger6ReleaseButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlLeftFinger6ReleaseButton_clicked");
    }
    armControlRight->releaseFinger(6);
}

void MainWindow::on_armControlLeftFinger7ReleaseButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlLeftFinger7ReleaseButton_clicked");
    }
    armControlRight->releaseFinger(7);
}

void MainWindow::on_armControlLeftFinger8ReleaseButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlLeftFinger8ReleaseButton_clicked");
    }
    armControlRight->releaseFinger(8);
}

void MainWindow::on_armControlLeftFinger9ReleaseButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlLeftFinger9ReleaseButton_clicked");
    }
    armControlRight->releaseFinger(9);
}

void MainWindow::on_armControlLeftFinger10ReleaseButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlLeftFinger10ReleaseButton_clicked");
    }
    armControlRight->releaseFinger(10);
}

void MainWindow::on_armControlLeftFinger11ReleaseButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlLeftFinger11ReleaseButton_clicked");
    }
    armControlRight->releaseFinger(11);
}
