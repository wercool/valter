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
}

void MainWindow::bodyControlP1TabRefreshTimerUpdate()
{
    bodyControlP1TabRefreshTimerUpdateWorker(ui);
}

void MainWindow::on_headYawLeftDirectonCheckButton_toggled(bool checked)
{
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
    bodyControlP1->setHeadYawStepSwitchDelay(value);
}

void MainWindow::on_headYawStepDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadYawStepDelay(value);
}

void MainWindow::on_headYawLeftRotateButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadYawDirection(false);
    bodyControlP1->setHeadYawMotorActivated(true);
}

void MainWindow::on_headYawLeftRotateButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadYawMotorActivated(false);
}

void MainWindow::on_headYawRightRotateButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadYawDirection(true);
    bodyControlP1->setHeadYawMotorActivated(true);
}

void MainWindow::on_headYawRightRotateButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadYawMotorActivated(false);
}

void MainWindow::on_headPitchStepDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchStepDelay(value);
}

void MainWindow::on_headPitchStepSwitchDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchStepSwitchDelay(value);
}

void MainWindow::on_headPitchDownButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchDirection(true);
    bodyControlP1->setHeadPitchMotorActivated(true);
}

void MainWindow::on_headPitchDownButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchMotorActivated(false);
}

void MainWindow::on_headPitchUpButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchDirection(false);
    bodyControlP1->setHeadPitchMotorActivated(true);
}

void MainWindow::on_headPitchUpButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setHeadPitchMotorActivated(false);
}

void MainWindow::on_head24VOnOffButton_clicked()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
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
    bodyControlP1->loadDefaults();
    loadBodyControlP1Defaults(ui);
}

void MainWindow::on_bodyPitchMotorDutyScroller_valueChanged(int value)
{
    ui->bodyPitchMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setBodyPitchMotorDutyMax(value);
}

void MainWindow::on_bodyPitchMotorDecelerationScroller_valueChanged(int value)
{
    ui->bodyPitchMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setBodyPitchMotorDeceleration(value);
}

void MainWindow::on_bodyPitchMotorAccelerationScroller_valueChanged(int value)
{
    ui->bodyPitchMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setBodyPitchMotorAcceleration(value);
}


void MainWindow::on_rightArmMotorDutyScroller_valueChanged(int value)
{
    ui->rightArmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setRightArmYawMotorDutyMax(value);
}

void MainWindow::on_rightArmMotorDecelerationScroller_valueChanged(int value)
{
    ui->rightArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setRightArmYawMotorDeceleration(value);
}

void MainWindow::on_rightArmMotorAccelerationScroller_valueChanged(int value)
{
    ui->rightArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setRightArmYawMotorAcceleration(value);
}

void MainWindow::on_leftArmMotorDutyScroller_valueChanged(int value)
{
    ui->leftArmMotorDutyLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setLeftArmYawMotorDutyMax(value);
}

void MainWindow::on_leftArmMotorDecelerationScroller_valueChanged(int value)
{
    ui->leftArmMotorDecelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setLeftArmYawMotorDeceleration(value);
}

void MainWindow::on_leftArmMotorAccelerationScroller_valueChanged(int value)
{
    ui->leftArmMotorAccelerationLabel->setText(Valter::format_string("[%d]", value).c_str());
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setLeftArmYawMotorAcceleration(value);
}

void MainWindow::on_bodyPitchUpButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
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
    bodyControlP1->setBodyPitchMotorActivated(false);
}

void MainWindow::on_bodyPitchDownButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
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
    bodyControlP1->setBodyPitchMotorActivated(false);
}


void MainWindow::on_rightArmYawOpenButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
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
    bodyControlP1->setRightArmYawMotorActivated(false);
}

void MainWindow::on_rightArmYawCloseButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
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
    bodyControlP1->setRightArmYawMotorActivated(false);
}


void MainWindow::on_leftArmYawOpenButton_pressed()
{

}

void MainWindow::on_leftArmYawOpenButton_released()
{

}

void MainWindow::on_leftArmYawCloseButton_pressed()
{

}

void MainWindow::on_leftArmYawCloseButton_released()
{

}
