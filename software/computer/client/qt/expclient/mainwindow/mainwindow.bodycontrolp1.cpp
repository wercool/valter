#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <sys/time.h>

#include <gui/guihelpers.h>

#include <valter.h>


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


void MainWindow::on_pitchHeadDownCheckButton_toggled(bool checked)
{
    if (checked)
    {
        ui->pitchHeadDownCheckButton->setStyleSheet("background-color: rgb(246, 216, 29); border-radius:10px;");
        ui->pitchHeadUpCheckButton->setChecked(false);
    }
    else
    {
        ui->pitchHeadDownCheckButton->setStyleSheet("");
    }
}

void MainWindow::on_pitchHeadUpCheckButton_toggled(bool checked)
{
    if (checked)
    {
        ui->pitchHeadUpCheckButton->setStyleSheet("background-color: rgb(246, 216, 29); border-radius:10px;");
        ui->pitchHeadDownCheckButton->setChecked(false);
    }
    else
    {
        ui->pitchHeadUpCheckButton->setStyleSheet("");
    }
}

void MainWindow::on_headYawStepSwitchDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setYawStepSwitchDelay(value);
}

void MainWindow::on_headYawStepDelaySpinBox_valueChanged(int value)
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setYawStepDelay(value);
}

void MainWindow::on_headYawLeftRotateButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setYawDirection(false);
    bodyControlP1->setYawMotorActivated(true);
}

void MainWindow::on_headYawLeftRotateButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setYawMotorActivated(false);
}

void MainWindow::on_headYawRightRotateButton_pressed()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setYawDirection(true);
    bodyControlP1->setYawMotorActivated(true);
}

void MainWindow::on_headYawRightRotateButton_released()
{
    BodyControlP1 *bodyControlP1 = BodyControlP1::getInstance();
    bodyControlP1->setYawMotorActivated(false);
}
