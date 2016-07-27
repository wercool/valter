#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include <gui/guihelpers.h>

#include <valter.h>

#include <gui/bodycontrolp1GUI.h>

void MainWindow::initBodyControlP1(Ui::MainWindow *ui)
{
    bodyControlP1TabRefreshTimer = new QTimer(this);
    connect(bodyControlP1TabRefreshTimer, SIGNAL(timeout()), this, SLOT(bodyControlP1TabRefreshTimerUpdate()));
    bodyControlP1TabRefreshTimer->start(50);

    ui->bodyPitchMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->bodyPitchMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->bodyPitchMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->rightArmMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->rightArmMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->rightArmMotorAccelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftArmMotorDutyScroller->installEventFilter(new WheelEventFilter());
    ui->leftArmMotorDecelerationScroller->installEventFilter(new WheelEventFilter());
    ui->leftArmMotorAccelerationScroller->installEventFilter(new WheelEventFilter());

    ui->headYawStepDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->headYawStepSwitchDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->headYawStepDelayTestModeSpinBox->installEventFilter(new WheelEventFilter());
    ui->headYawMoveStepsSpinBox->installEventFilter(new WheelEventFilter());

    ui->headPitchStepDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->headPitchStepSwitchDelaySpinBox->installEventFilter(new WheelEventFilter());
    ui->headPitchStepDelayTestModeSpinBox->installEventFilter(new WheelEventFilter());
    ui->headPitchMoveStepsSpinBox->installEventFilter(new WheelEventFilter());

    ui->bodyCameraPositionScroller->installEventFilter(new WheelEventFilter());
}

void MainWindow::bodyControlP1TabRefreshTimerUpdate()
{
    bodyControlP1TabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_headYawLeftDirectonCheckButton_toggled(bool checked)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headYawLeftDirectonCheckButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    if (checked)
    {
        ui->headYawLeftDirectonCheckButton->setStyleSheet("background-color: rgb(246, 216, 29); border-radius:10px;");
        ui->headYawRightDirectonCheckButton->setChecked(false);
    }
    else
    {
        ui->headYawLeftDirectonCheckButton->setStyleSheet("");
    }
}


void MainWindow::on_headYawRightDirectonCheckButton_toggled(bool checked)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headYawRightDirectonCheckButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    if (checked)
    {
        ui->headYawRightDirectonCheckButton->setStyleSheet("background-color: rgb(246, 216, 29); border-radius:10px;");
        ui->headYawLeftDirectonCheckButton->setChecked(false);
    }
    else
    {
        ui->headYawRightDirectonCheckButton->setStyleSheet("");
    }
}


void MainWindow::on_pitchHeadDownDirectionCheckButton_toggled(bool checked)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_pitchHeadDownDirectionCheckButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    if (checked)
    {
        ui->pitchHeadDownDirectionCheckButton->setStyleSheet("background-color: rgb(246, 216, 29); border-radius:10px;");
        ui->pitchHeadUpDirectionCheckButton->setChecked(false);
    }
    else
    {
        ui->pitchHeadDownDirectionCheckButton->setStyleSheet("");
    }
}


void MainWindow::on_pitchHeadUpDirectionCheckButton_toggled(bool checked)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_pitchHeadUpDirectionCheckButton_toggled@%s", (!checked) ? "true" : "false"));
    }
    if (checked)
    {
        ui->pitchHeadUpDirectionCheckButton->setStyleSheet("background-color: rgb(246, 216, 29); border-radius:10px;");
        ui->pitchHeadDownDirectionCheckButton->setChecked(false);
    }
    else
    {
        ui->pitchHeadUpDirectionCheckButton->setStyleSheet("");
    }
}

void MainWindow::on_headYawStepSwitchDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headYawStepSwitchDelaySpinBox_valueChanged@%d", value));
    }
    bodyControlP1->setHeadYawStepSwitchDelay(value);
}


void MainWindow::on_headYawStepDelayTestModeSpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headYawStepDelayTestModeSpinBox_valueChanged@%d", value));
    }
}


void MainWindow::on_headYawMoveStepsSpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headYawMoveStepsSpinBox_valueChanged@%d", value));
    }
}


void MainWindow::on_headPitchMoveStepsSpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headPitchMoveStepsSpinBox_valueChanged@%d", value));
    }
}

void MainWindow::on_headYawStepDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headYawStepDelaySpinBox_valueChanged@%d", value));
    }
    bodyControlP1->setHeadYawStepDelay(value);
}

void MainWindow::on_headYawLeftRotateButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headYawLeftRotateButton_pressed");
    }
    bodyControlP1->setHeadYawDirection(false);
    bodyControlP1->setHeadYawMotorActivated(true);
}

void MainWindow::on_headYawLeftRotateButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headYawLeftRotateButton_released");
    }
    bodyControlP1->setHeadYawMotorActivated(false);
}

void MainWindow::on_headYawRightRotateButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headYawRightRotateButton_pressed");
    }
    bodyControlP1->setHeadYawDirection(true);
    bodyControlP1->setHeadYawMotorActivated(true);
}

void MainWindow::on_headYawRightRotateButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headYawRightRotateButton_released");
    }
    bodyControlP1->setHeadYawMotorActivated(false);
}

void MainWindow::on_headPitchStepDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headPitchStepDelaySpinBox_valueChanged@%d", value));
    }
    bodyControlP1->setHeadPitchStepDelay(value);
}

void MainWindow::on_headPitchStepSwitchDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headPitchStepSwitchDelaySpinBox_valueChanged@%d", value));
    }
    bodyControlP1->setHeadPitchStepSwitchDelay(value);
}


void MainWindow::on_headPitchStepDelayTestModeSpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_headPitchStepDelayTestModeSpinBox_valueChanged@%d", value));
    }
}

void MainWindow::on_headPitchDownButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headPitchDownButton_pressed");
    }
    bodyControlP1->setHeadPitchDirection(true);
    bodyControlP1->setHeadPitchMotorActivated(true);
}

void MainWindow::on_headPitchDownButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headPitchDownButton_released");
    }
    bodyControlP1->setHeadPitchMotorActivated(false);
}

void MainWindow::on_headPitchUpButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headPitchUpButton_pressed");
    }
    bodyControlP1->setHeadPitchDirection(false);
    bodyControlP1->setHeadPitchMotorActivated(true);
}

void MainWindow::on_headPitchUpButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headPitchUpButton_released");
    }
    bodyControlP1->setHeadPitchMotorActivated(false);
}

void MainWindow::on_head24VOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_head24VOnOffButton_clicked");
    }
    if (!bodyControlP1->getHead24VState())
    {
        bodyControlP1->setHead24VOnOff(true);
    }
    else
    {
        bodyControlP1->setHead24VOnOff(false);
    }
}

void MainWindow::on_headYawMotorEnableDisableButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headYawMotorEnableDisableButton_clicked");
    }
    if (!bodyControlP1->getHeadYawMotorState())
    {
        bodyControlP1->setHeadYawMotorOnOff(true);
    }
    else
    {
        bodyControlP1->setHeadYawMotorOnOff(false);
    }
}

void MainWindow::on_headPitchMotorEnableDisableButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headPitchMotorEnableDisableButton_clicked");
    }
    if (!bodyControlP1->getHeadPitchMotorState())
    {
        bodyControlP1->setHeadPitchMotorOnOff(true);
    }
    else
    {
        bodyControlP1->setHeadPitchMotorOnOff(false);
    }
}


void MainWindow::on_bodyControlP1LoadDefaultsButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyControlP1LoadDefaultsButton_clicked");
    }
    bodyControlP1->loadDefaults();
    loadBodyControlP1Defaults(ui);
}


void MainWindow::on_bodyControlP1CheckBox_clicked(bool checked)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_bodyControlP1CheckBox_clicked@%s", (!checked) ? "true" : "false"));
    }
}

void MainWindow::on_bodyPitchMotorDutyScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_bodyPitchMotorDutyScroller_valueChanged@%d", value));
    }
    bodyControlP1->setBodyPitchMotorDutyMax(value);
    ui->bodyPitchMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_bodyPitchMotorDecelerationScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_bodyPitchMotorDecelerationScroller_valueChanged@%d", value));
    }
    bodyControlP1->setBodyPitchMotorDeceleration(value);
    ui->bodyPitchMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_bodyPitchMotorAccelerationScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_bodyPitchMotorAccelerationScroller_valueChanged@%d", value));
    }
    bodyControlP1->setBodyPitchMotorAcceleration(value);
    ui->bodyPitchMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}


void MainWindow::on_rightArmMotorDutyScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_rightArmMotorDutyScroller_valueChanged@%d", value));
    }
    bodyControlP1->setRightArmYawMotorDutyMax(value);
    ui->rightArmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightArmMotorDecelerationScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_rightArmMotorDecelerationScroller_valueChanged@%d", value));
    }
    bodyControlP1->setRightArmYawMotorDeceleration(value);
    ui->rightArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_rightArmMotorAccelerationScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_rightArmMotorAccelerationScroller_valueChanged@%d", value));
    }
    bodyControlP1->setRightArmYawMotorAcceleration(value);
    ui->rightArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftArmMotorDutyScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_leftArmMotorDutyScroller_valueChanged@%d", value));
    }
    bodyControlP1->setLeftArmYawMotorDutyMax(value);
    ui->leftArmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftArmMotorDecelerationScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_leftArmMotorDecelerationScroller_valueChanged@%d", value));
    }
    bodyControlP1->setLeftArmYawMotorDeceleration(value);
    ui->leftArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_leftArmMotorAccelerationScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_leftArmMotorAccelerationScroller_valueChanged@%d", value));
    }
    bodyControlP1->setLeftArmYawMotorAcceleration(value);
    ui->leftArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
}

void MainWindow::on_bodyPitchUpButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyPitchUpButton_pressed");
    }
    if (bodyControlP1->prepareBodyPitchMovement())
    {
        //up
        if (bodyControlP1->setBodyPitchMovementDirection(false))
        {
            bodyControlP1->setBodyPitchMotorActivated(true);
        }
    }
}

void MainWindow::on_bodyPitchUpButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyPitchUpButton_released");
    }
    bodyControlP1->setBodyPitchMotorActivated(false);
}

void MainWindow::on_bodyPitchDownButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyPitchDownButton_pressed");
    }
    if (bodyControlP1->prepareBodyPitchMovement())
    {
        //down
        if (bodyControlP1->setBodyPitchMovementDirection(true))
        {
            bodyControlP1->setBodyPitchMotorActivated(true);
        }
    }
}

void MainWindow::on_bodyPitchDownButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyPitchDownButton_released");
    }
    bodyControlP1->setBodyPitchMotorActivated(false);
}


void MainWindow::on_rightArmYawOpenButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightArmYawOpenButton_pressed");
    }
    if (bodyControlP1->prepareRightArmYawMovement())
    {
        //open
        if (bodyControlP1->setRightArmYawMovementDirection(true))
        {
            bodyControlP1->setRightArmYawMotorActivated(true);
        }
    }
}

void MainWindow::on_rightArmYawOpenButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightArmYawOpenButton_released");
    }
    bodyControlP1->setRightArmYawMotorActivated(false);
}

void MainWindow::on_rightArmYawCloseButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightArmYawCloseButton_pressed");
    }
    if (bodyControlP1->prepareRightArmYawMovement())
    {
        //close
        if (bodyControlP1->setRightArmYawMovementDirection(false))
        {
            bodyControlP1->setRightArmYawMotorActivated(true);
        }
    }
}

void MainWindow::on_rightArmYawCloseButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightArmYawCloseButton_released");
    }
    bodyControlP1->setRightArmYawMotorActivated(false);
}


void MainWindow::on_leftArmYawOpenButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftArmYawOpenButton_pressed");
    }
    if (bodyControlP1->prepareLeftArmYawMovement())
    {
        //open
        if (bodyControlP1->setLeftArmYawMovementDirection(true))
        {
            bodyControlP1->setLeftArmYawMotorActivated(true);
        }
    }
}

void MainWindow::on_leftArmYawOpenButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftArmYawOpenButton_released");
    }
    bodyControlP1->setLeftArmYawMotorActivated(false);
}

void MainWindow::on_leftArmYawCloseButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftArmYawCloseButton_pressed");
    }
    if (bodyControlP1->prepareLeftArmYawMovement())
    {
        //close
        if (bodyControlP1->setLeftArmYawMovementDirection(false))
        {
            bodyControlP1->setLeftArmYawMotorActivated(true);
        }
    }
}

void MainWindow::on_leftArmYawCloseButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftArmYawCloseButton_released");
    }
    bodyControlP1->setLeftArmYawMotorActivated(false);
}

void MainWindow::on_bodyControlP1ShiftRegEnableButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyControlP1ShiftRegEnableButton_clicked");
    }
    bodyControlP1->shiftRegEnable();
}

void MainWindow::on_bodyControlP1ShiftRegDisableButton_clicked()
{
    if (QMessageBox::Yes == QMessageBox(QMessageBox::Information, "Cofirmation", "Are you sure you want to disable/enable BODY-CONTROL-P1 SR?", QMessageBox::Yes|QMessageBox::No, this, Qt::WindowStaysOnTopHint).exec())
    {
        BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
        if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
        {
            bodyControlP1->sendTCPCommand("on_bodyControlP1ShiftRegDisableButton_clicked");
        }
        bodyControlP1->shiftRegDisable();
    }
}

void MainWindow::on_bodyControlP1ShiftRegResetButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyControlP1ShiftRegResetButton_clicked");
    }
    bodyControlP1->shiftRegReset();
}

void MainWindow::on_bodyControlP1StopShiftRegResetButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyControlP1StopShiftRegResetButton_clicked");
    }
    bodyControlP1->stopShiftRegReset();
}

void MainWindow::on_powerSource5V5OnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_powerSource5V5OnOffButton_clicked");
    }
    if (bodyControlP1->getPowerSource5V5State())
    {
        bodyControlP1->setPowerSource5VOnOff(false);
    }
    else
    {
        bodyControlP1->setPowerSource5VOnOff(true);
    }
}

void MainWindow::on_wifiOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_wifiOnOffButton_clicked");
    }
    if (bodyControlP1->getWifiPowerState())
    {
        bodyControlP1->setWifiPowerOnOff(false);
    }
    else
    {
        bodyControlP1->setWifiPowerOnOff(true);
    }
}

void MainWindow::on_leftArm24VOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftArm24VOnOffButton_clicked");
    }
    if (bodyControlP1->getLeftArm24VPowerSourceState())
    {
        bodyControlP1->setLeftArm24VPowerOnOff(false);
    }
    else
    {
        bodyControlP1->setLeftArm24VPowerOnOff(true);
    }
}

void MainWindow::on_rightArm24VButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightArm24VButton_clicked");
    }
    if (bodyControlP1->getRightArm24VPowerSourceState())
    {
        bodyControlP1->setRightArm24VPowerOnOff(false);
    }
    else
    {
        bodyControlP1->setRightArm24VPowerOnOff(true);
    }
}

void MainWindow::on_headLedOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headLedOnOffButton_clicked");
    }
    if (bodyControlP1->getHeadLedState())
    {
        bodyControlP1->setHeadLedOnOff(false);
    }
    else
    {
        bodyControlP1->setHeadLedOnOff(true);
    }
}

void MainWindow::on_leftAccumulatorOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftAccumulatorOnOffButton_clicked");
    }
    if (bodyControlP1->getLeftAccumulatorConnectedState())
    {
        bodyControlP1->setLeftAccumulatorOnOff(false);
    }
    else
    {
        bodyControlP1->setLeftAccumulatorOnOff(true);
    }
}

void MainWindow::on_rightAccumulatorOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightAccumulatorOnOffButton_clicked");
    }
    if (bodyControlP1->getRightAccumulatorConnectedState())
    {
        bodyControlP1->setRightAccumulatorOnOff(false);
    }
    else
    {
        bodyControlP1->setRightAccumulatorOnOff(true);
    }
}

void MainWindow::on_leftArm12VOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_leftArm12VOnOffButton_clicked");
    }
    if (bodyControlP1->getLeftArm12VPowerSourceState())
    {
        bodyControlP1->setLeftArm12VPowerOnOff(false);
    }
    else
    {
        bodyControlP1->setLeftArm12VPowerOnOff(true);
    }
}

void MainWindow::on_rightArm12VButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_rightArm12VButton_clicked");
    }
    if (bodyControlP1->getRightArm12VPowerSourceState())
    {
        bodyControlP1->setRightArm12VPowerOnOff(false);
    }
    else
    {
        bodyControlP1->setRightArm12VPowerOnOff(true);
    }
}

void MainWindow::on_kinect1OnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_kinect1OnOffButton_clicked");
    }
    if (bodyControlP1->getKinect1PowerState())
    {
        bodyControlP1->setKinect1PowerOnOff(false);
    }
    else
    {
        bodyControlP1->setKinect1PowerOnOff(true);
    }
}

void MainWindow::on_kinect2OnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_kinect2OnOffButton_clicked");
    }
    if (bodyControlP1->getKinect2PowerState())
    {
        bodyControlP1->setKinect2PowerOnOff(false);
    }
    else
    {
        bodyControlP1->setKinect2PowerOnOff(true);
    }
}

void MainWindow::on_headYawTestModeExecuteButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headYawTestModeExecuteButton_clicked");
    }
    bool direction = ui->headYawRightDirectonCheckButton->isChecked();
    direction = !ui->headYawLeftDirectonCheckButton->isChecked();
    if (ui->headYawRightDirectonCheckButton->isChecked() || ui->headYawLeftDirectonCheckButton->isChecked())
    {
        int stepTime = ui->headYawStepDelayTestModeSpinBox->value();
        int steps = ui->headYawMoveStepsSpinBox->value();
        bodyControlP1->headYawMoveSteps(direction, stepTime, steps);
    }
}

void MainWindow::on_headPitchTestModeExecuteButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_headPitchTestModeExecuteButton_clicked");
    }
    bool direction = ui->pitchHeadDownDirectionCheckButton->isChecked();
    direction = !ui->pitchHeadUpDirectionCheckButton->isChecked();
    if (ui->pitchHeadDownDirectionCheckButton->isChecked() || ui->pitchHeadUpDirectionCheckButton->isChecked())
    {
        int stepTime = ui->headPitchStepDelayTestModeSpinBox->value();
        int steps = ui->headPitchMoveStepsSpinBox->value();
        bodyControlP1->headPitchMoveSteps(direction, stepTime, steps);
    }
}

void MainWindow::on_getHeadYawPositionButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_getHeadYawPositionButton_clicked");
    }
    bodyControlP1->requestHeadYawPosition();
}

void MainWindow::on_getHeadPitchPositionButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_getHeadPitchPositionButton_clicked");
    }
    bodyControlP1->requestHeadPitchPosition();
}

void MainWindow::on_bodyControlP1ReadingsTable_itemClicked(QTableWidgetItem *item)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_bodyControlP1ReadingsTable_itemClicked@%d@%d@%s", item->row(), item->column(), (item->checkState() == Qt::Checked ? "false" : "true")));
    }
}

void MainWindow::on_bodyCameraReleaseButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand("on_bodyCameraReleaseButton_clicked");
    }
    bodyControlP1->releaseBodyCamera();
}

void MainWindow::on_bodyCameraPositionScroller_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    if (ui->bodyControlP1RemoteControlCheckbox->isChecked())
    {
        bodyControlP1->sendTCPCommand(Valter::format_string("on_bodyCameraPositionScroller_valueChanged@%d", value));
    }
    bodyControlP1->setBodyCameraPosition(value * 4);
}
