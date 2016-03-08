#ifndef MAINWINDOWUTILS_H
#define MAINWINDOWUTILS_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include <valter.h>

void loadPlatformControlP1Defaults(Ui::MainWindow *ui)
{
    PlatformControlP1 *platformControlP1 = (PlatformControlP1*)Valter::getInstance()->getValterModule(PlatformControlP1::getControlDeviceId());

    string deFaultvalue;
    char *deFaultvaluePtr;
    int curValue;
    int minValue;
    int maxValue;

    //platformControlP1WheelMotorsDutySlider
    deFaultvalue = platformControlP1->getDefault("platformControlP1WheelMotorsDutySlider");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    curValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    minValue = atoi(strtok(NULL, "," ));
    maxValue = atoi(strtok(NULL, "," ));
    ui->platformControlP1WheelMotorsDutySlider->setValue(curValue);
    ui->platformControlP1WheelMotorsDutySlider->setMinimum(minValue);
    ui->platformControlP1WheelMotorsDutySlider->setMaximum(maxValue);

    //platformMovementDecelerationSlider
    deFaultvalue = platformControlP1->getDefault("platformMovementDecelerationSlider");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    curValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    minValue = atoi(strtok(NULL, "," ));
    maxValue = atoi(strtok(NULL, "," ));
    ui->platformMovementDecelerationSlider->setValue(curValue);
    ui->platformMovementDecelerationSlider->setMinimum(minValue);
    ui->platformMovementDecelerationSlider->setMaximum(maxValue);
    platformControlP1->setPlatformDeceleration(curValue);

    //platformMovementAccelerationSlider
    deFaultvalue = platformControlP1->getDefault("platformMovementAccelerationSlider");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    curValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    minValue = atoi(strtok(NULL, "," ));
    maxValue = atoi(strtok(NULL, "," ));
    ui->platformMovementAccelerationSlider->setValue(curValue);
    ui->platformMovementAccelerationSlider->setMinimum(minValue);
    ui->platformMovementAccelerationSlider->setMaximum(maxValue);
    platformControlP1->setPlatformAcceleration(curValue);

    //leftMotorPlatformControlP1DutySlider
    deFaultvalue = platformControlP1->getDefault("leftMotorPlatformControlP1DutySlider");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    curValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    minValue = atoi(strtok(NULL, "," ));
    maxValue = atoi(strtok(NULL, "," ));
    ui->leftMotorPlatformControlP1DutySlider->setValue(curValue);
    ui->leftMotorPlatformControlP1DutySlider->setMinimum(minValue);
    ui->leftMotorPlatformControlP1DutySlider->setMaximum(maxValue);
    platformControlP1->setLeftMotorDutyMax(curValue);

    //rightMotorPlatformControlP1DutySlider
    deFaultvalue = platformControlP1->getDefault("rightMotorPlatformControlP1DutySlider");
    deFaultvaluePtr = Valter::stringToCharPtr(deFaultvalue);
    curValue = atoi(Valter::stringToCharPtr(strtok(deFaultvaluePtr, "," )));
    minValue = atoi(strtok(NULL, "," ));
    maxValue = atoi(strtok(NULL, "," ));
    ui->rightMotorPlatformControlP1DutySlider->setValue(curValue);
    ui->rightMotorPlatformControlP1DutySlider->setMinimum(minValue);
    ui->rightMotorPlatformControlP1DutySlider->setMaximum(maxValue);
    platformControlP1->setRightMotorDutyMax(curValue);
}

#endif // MAINWINDOWUTILS_H
