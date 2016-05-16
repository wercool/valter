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
    ui->armControlRightArmMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->armControlRightArmMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->armControlRightMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
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
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightForearmMotorDutyScroller_valueChanged@%d", value));
    }
    armControlRight->setRightForearmMotorDutyMax(value);
    ui->rightForearmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightForearmMotorDecelerationScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightForearmMotorDecelerationScroller_valueChanged@%d", value));
    }
    armControlRight->setRightForearmMotorDeceleration(value);
    ui->rightForearmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightForearmAccelerationScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightForearmAccelerationScroller_valueChanged@%d", value));
    }
    armControlRight->setRightForearmMotorAcceleration(value);
    ui->rightForearmAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_armControlRightArmMotorDutyScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_armControlRightArmMotorDutyScroller_valueChanged@%d", value));
    }
    armControlRight->setRightArmMotorDutyMax(value);
    ui->armControlRightArmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_armControlRightArmMotorDecelerationScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_armControlRightArmMotorDecelerationScroller_valueChanged@%d", value));
    }
    armControlRight->setRightArmMotorDeceleration(value);
    ui->armControlRightArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_armControlRightMotorAccelerationScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_armControlRightMotorAccelerationScroller_valueChanged@%d", value));
    }
    armControlRight->setRightArmMotorAcceleration(value);
    ui->armControlRightArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightLimbMotorDutyScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightLimbMotorDutyScroller_valueChanged@%d", value));
    }
    armControlRight->setRightLimbMotorDutyMax(value);
    ui->rightLimbMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightLimbMotorDecelerationScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightLimbMotorDecelerationScroller_valueChanged@%d", value));
    }
    armControlRight->setRightLimbMotorDeceleration(value);
    ui->rightLimbMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightLimbMotorAccelerationScroller_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightLimbMotorAccelerationScroller_valueChanged@%d", value));
    }
    armControlRight->setRightLimbMotorAcceleration(value);
    ui->rightLimbMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightForearmMoveUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmMoveUpButton_pressed");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmMoveUpButton_released");
    }
    armControlRight->setRightForearmMotorActivated(false);
}

void MainWindow::on_rightForearmMoveDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmMoveDownButton_pressed");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmMoveDownButton_released");
    }
    armControlRight->setRightForearmMotorActivated(false);
}

void MainWindow::on_rightArmMoveUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmMoveUpButton_pressed");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmMoveUpButton_released");
    }
    armControlRight->setRightArmMotorActivated(false);
}

void MainWindow::on_rightArmMoveDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmMoveDownButton_pressed");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmMoveDownButton_released");
    }
    armControlRight->setRightArmMotorActivated(false);
}

void MainWindow::on_rightLimbMoveUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightLimbMoveUpButton_pressed");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightLimbMoveUpButton_released");
    }
    armControlRight->setRightLimbMotorActivated(false);
}

void MainWindow::on_rightLimbMoveDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightLimbMoveDownButton_pressed");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightLimbMoveDownButton_released");
    }
    armControlRight->setRightLimbMotorActivated(false);
}

void MainWindow::on_rightHandYawCCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandYawCCWButton_pressed");
    }
    armControlRight->setHandYawDirection(false);
    armControlRight->handYaw(true);
}

void MainWindow::on_rightHandYawCCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandYawCCWButton_released");
    }
    armControlRight->handYaw(false);
}

void MainWindow::on_rightHandYawCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandYawCWButton_pressed");
    }
    armControlRight->setHandYawDirection(true);
    armControlRight->handYaw(true);
}

void MainWindow::on_rightHandYawCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandYawCWButton_released");
    }
    armControlRight->handYaw(false);
}

void MainWindow::on_rightHandPitchUpButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandPitchUpButton_pressed");
    }
    armControlRight->setHandPitchDirection(true);
    armControlRight->handPitch(true);
}

void MainWindow::on_rightHandPitchUpButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandPitchUpButton_released");
    }
    armControlRight->handPitch(false);
}

void MainWindow::on_rightHandPitchDownButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandPitchDownButton_pressed");
    }
    armControlRight->setHandPitchDirection(false);
    armControlRight->handPitch(true);
}

void MainWindow::on_rightHandPitchDownButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandPitchDownButton_released");
    }
    armControlRight->handPitch(false);
}

void MainWindow::on_rightForearmRollMotorOnOffButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmRollMotorOnOffButton_clicked");
    }
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
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmRollCCWButton_pressed");
    }
    if (armControlRight->getForearmRollMotorState())
    {
        armControlRight->setForearmRollDirection(true); //CW
        armControlRight->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_rightForearmRollCCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmRollCCWButton_released");
    }
    armControlRight->setForearmRollMotorActivated(false);
}

void MainWindow::on_rightForearmRollCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmRollCWButton_pressed");
    }
    if (armControlRight->getForearmRollMotorState())
    {
        armControlRight->setForearmRollDirection(false); //CCW
        armControlRight->setForearmRollMotorActivated(true);
    }
}

void MainWindow::on_rightForearmRollCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmRollCWButton_released");
    }
    armControlRight->setForearmRollMotorActivated(false);
}

void MainWindow::on_rightForearmRollStepDelaySpinBox_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightForearmRollStepDelaySpinBox_valueChanged@%d", value));
    }
    armControlRight->setForearmRollStepDelay(value);
}

void MainWindow::on_rightForearmRollStepSwitchDelaySpinBox_valueChanged(int value)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightForearmRollStepSwitchDelaySpinBox_valueChanged@%d", value));
    }
    armControlRight->setForearmRollStepSwitchDelay(value);
}

void MainWindow::on_rightForearmResetPositionButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmResetPositionButton_clicked");
    }
    armControlRight->setForearmRollResettingStepPosition(!armControlRight->getForearmRollResettingStepPosition());
}

void MainWindow::on_rightArmLedsOnOffButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmLedsOnOffButton_clicked");
    }
    armControlRight->turnArmLedsOnOff();
}

void MainWindow::on_armControlRightLoadDefaultsButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlRightLoadDefaultsButton_clicked");
    }
    armControlRight->loadDefaults();
    loadArmControlRightDefaults(ui);
}

void MainWindow::on_armControlRightRedrawGUICheckBox_clicked(bool checked)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_armControlRightRedrawGUICheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_rightArmReadingsTable_itemClicked(QTableWidgetItem *item)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightArmReadingsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
    setRightArmReadingsPresets(item);
}

void MainWindow::on_rightHandSensorsTable_itemClicked(QTableWidgetItem *item)
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand(Valter::format_string("on_rightHandSensorsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
    setRightHandForceSensorsReadingsPresets(item);
}

void MainWindow::on_rightForearmYawCCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmYawCCWButton_pressed");
    }
    armControlRight->setForearmYawDirection(false);
    armControlRight->forearmYaw(true);
}

void MainWindow::on_rightForearmYawCCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmYawCCWButton_released");
    }
    armControlRight->forearmYaw(false);
}

void MainWindow::on_rightForearmYawCWButton_pressed()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmYawCWButton_pressed");
    }
    armControlRight->setForearmYawDirection(true);
    armControlRight->forearmYaw(true);
}

void MainWindow::on_rightForearmYawCWButton_released()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightForearmYawCWButton_released");
    }
    armControlRight->forearmYaw(false);
}

void MainWindow::on_rightHandSensorsTrackAllButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandSensorsTrackAllButton_clicked");
    }
    for (int i = 0; i < 13; i++)
    {
        armControlRight->setHandSensorsTrack(i, true);
        ((QTableWidgetItem*)ui->rightHandSensorsTable->item(i, 0))->setCheckState(Qt::Checked);
    }
    ui->rightHandSensorsTable->viewport()->update();
}

void MainWindow::on_rightHandSensorsTrackNoneButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightHandSensorsTrackNoneButton_clicked");
    }
    for (int i = 0; i < 13; i++)
    {
        armControlRight->setHandSensorsTrack(i, false);
        ((QTableWidgetItem*)ui->rightHandSensorsTable->item(i, 0))->setCheckState(Qt::Unchecked);
    }
    ui->rightHandSensorsTable->viewport()->update();
}

void MainWindow::on_rightArmReadingsTrackAllButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmReadingsTrackAllButton_clicked");
    }
    armControlRight->setForearmPositionTrack(true);
    armControlRight->setArmPositionTrack(true);
    armControlRight->setLimbPositionTrack(true);
    armControlRight->setForearmMotorCurrentTrack(true);
    armControlRight->setArmMotorCurrentTrack(true);
    armControlRight->setLimbMotorCurrentTrack(true);
    armControlRight->setHandYawPositionTrack(true);
    armControlRight->setHandPitchPositionTrack(true);
    armControlRight->setForearmYawPositionTrack(true);
    armControlRight->setForearmYawMotorCurrentTrack(true);
    armControlRight->setHandYawMotorCurrentTrack(true);
    armControlRight->setHandPitchMotorCurrentTrack(true);
    for (int i = 0; i < 12; i++)
    {
        ((QTableWidgetItem*)ui->rightArmReadingsTable->item(i, 0))->setCheckState(Qt::Checked);
    }
    ui->rightArmReadingsTable->viewport()->update();
}

void MainWindow::on_rightArmReadingsTrackNoneButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_rightArmReadingsTrackNoneButton_clicked");
    }
    armControlRight->setForearmPositionTrack(false);
    armControlRight->setArmPositionTrack(false);
    armControlRight->setLimbPositionTrack(false);
    armControlRight->setForearmMotorCurrentTrack(false);
    armControlRight->setArmMotorCurrentTrack(false);
    armControlRight->setLimbMotorCurrentTrack(false);
    armControlRight->setHandYawPositionTrack(false);
    armControlRight->setHandPitchPositionTrack(false);
    armControlRight->setForearmYawPositionTrack(false);
    armControlRight->setForearmYawMotorCurrentTrack(false);
    armControlRight->setHandYawMotorCurrentTrack(false);
    armControlRight->setHandPitchMotorCurrentTrack(false);
    for (int i = 0; i < 12; i++)
    {
        ((QTableWidgetItem*)ui->rightArmReadingsTable->item(i, 0))->setCheckState(Qt::Unchecked);
    }
    ui->rightArmReadingsTable->viewport()->update();
}

void MainWindow::on_armControlRightStopAllWatchersButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlRightStopAllWatchersButton_clicked");
    }
    armControlRight->stopAllWatchers();
}

void MainWindow::on_armControlRightStartAllWatchersButton_clicked()
{
    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    if (ui->armControlRightRemoteControlCheckbox->isChecked())
    {
        armControlRight->sendTCPCommand("on_armControlRightStartAllWatchersButton_clicked");
    }
    armControlRight->startAllWatchers();
}
