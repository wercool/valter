#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformmanipulatorandirbumper.h"


void PlatformManipulatorAndIRBumper::loadDefaults()
{
    ifstream defaultsFile(PlatformManipulatorAndIRBumper::defaultsFilePath);
    string line;
    while (getline(defaultsFile, line, '\n'))
    {
        if (line.substr(0, 2).compare("//") != 0)
        {
            char *lineStrPtr = Valter::stringToCharPtr(line);
            string defaultValueName(strtok(lineStrPtr, ":" ));
            string defaultValue(strtok(NULL, ":" ));
            addDefault(defaultValueName, defaultValue);
        }
    }
    defaultsFile.close();

    string defaultValue;
    char *defaultValuePtr;
    int curValue;
    int minValue;
    int maxValue;

    int val1;
    int val2;

    //link1MotorDuty
    defaultValue = getDefault("link1MotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLink1MotorDutyPresetMin(minValue);
    setLink1MotorDutyPresetMax(maxValue);
    setLink1MotorDutyPresetCur(curValue);
    setLink1MotorDutyMax(getLink1MotorDutyPresetMax());

    //link1MotorAcceleration
    defaultValue = getDefault("link1MotorAcceleration");
    link1MotorAcceleration = stof(Valter::stringToCharPtr(defaultValue));

    //link1MotorDeceleration
    defaultValue = getDefault("link1MotorDeceleration");
    link1MotorDeceleration = stof(Valter::stringToCharPtr(defaultValue));

    //link2MotorDuty
    defaultValue = getDefault("link2MotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLink2MotorDutyPresetMin(minValue);
    setLink2MotorDutyPresetMax(maxValue);
    setLink2MotorDutyPresetCur(curValue);
    setLink2MotorDutyMax(getLink2MotorDutyPresetMax());

    //link2MotorAcceleration
    defaultValue = getDefault("link2MotorAcceleration");
    link2MotorAcceleration = stof(Valter::stringToCharPtr(defaultValue));

    //link2MotorDeceleration
    defaultValue = getDefault("link2MotorDeceleration");
    link2MotorDeceleration = stof(Valter::stringToCharPtr(defaultValue));

    //manGripperRotationMotorDuty
    defaultValue = getDefault("manGripperRotationMotorDuty");
    manGripperRotationMotorDuty = stof(Valter::stringToCharPtr(defaultValue));

    //link1Rotation
    defaultValue = getDefault("link1Rotation");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLink1PositionTrack(val1);
    setLink1PositionADC(val2);

    //link2Rotation
    defaultValue = getDefault("link2Rotation");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLink2PositionTrack(val1);
    setLink2PositionADC(val2);

    //link1MotorCurrent
    defaultValue = getDefault("link1MotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLink1CurrentTrack(val1);
    setLink1CurrentADC(val2);

    //link2MotorCurrent
    defaultValue = getDefault("link2MotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLink2CurrentTrack(val1);
    setLink2CurrentADC(val2);
}

bool PlatformManipulatorAndIRBumper::getLink1MotorDecelerating() const
{
    return link1MotorDecelerating;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDecelerating(bool value)
{
    link1MotorDecelerating = value;
}

bool PlatformManipulatorAndIRBumper::getLink1MotorAccelerating() const
{
    return link1MotorAccelerating;
}

void PlatformManipulatorAndIRBumper::setLink1MotorAccelerating(bool value)
{
    link1MotorAccelerating = value;
}

bool PlatformManipulatorAndIRBumper::getPower24VOnOff() const
{
    return power24VOnOff;
}

void PlatformManipulatorAndIRBumper::setPower24VOn()
{
    power24VOnOff = true;
}

void PlatformManipulatorAndIRBumper::setPower24VOff()
{
    power24VOnOff = false;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyPresetMax() const
{
    return link2MotorDutyPresetMax;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyPresetMax(int value)
{
    link2MotorDutyPresetMax = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyPresetMin() const
{
    return link2MotorDutyPresetMin;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyPresetMin(int value)
{
    link2MotorDutyPresetMin = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyPresetCur() const
{
    return link2MotorDutyPresetCur;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyPresetCur(int value)
{
    link2MotorDutyPresetCur = value;
}

bool PlatformManipulatorAndIRBumper::getLink2MotorStop() const
{
    return link2MotorStop;
}

void PlatformManipulatorAndIRBumper::setLink2MotorStop(bool value)
{
    link2MotorStop = value;
}

bool PlatformManipulatorAndIRBumper::getLink2MotorActivated() const
{
    return link2MotorActivated;
}

void PlatformManipulatorAndIRBumper::setLink2MotorActivated(bool value)
{
    link2MotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setLink2MotorStop(false);
    }
}

bool PlatformManipulatorAndIRBumper::getLink2MovementDirection() const
{
    return link2MovementDirection;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDeceleration() const
{
    return link2MotorDeceleration;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDeceleration(int value)
{
    link2MotorDeceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorAcceleration() const
{
    return link2MotorAcceleration;
}

void PlatformManipulatorAndIRBumper::setLink2MotorAcceleration(int value)
{
    link2MotorAcceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDutyMax() const
{
    return link2MotorDutyMax;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDutyMax(int value)
{
    link2MotorDutyMax = value;
}

int PlatformManipulatorAndIRBumper::getLink2MotorDuty() const
{
    return link2MotorDuty;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDuty(int value)
{
    link2MotorDuty = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyPresetMax() const
{
    return link1MotorDutyPresetMax;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyPresetMax(int value)
{
    link1MotorDutyPresetMax = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyPresetMin() const
{
    return link1MotorDutyPresetMin;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyPresetMin(int value)
{
    link1MotorDutyPresetMin = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyPresetCur() const
{
    return link1MotorDutyPresetCur;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyPresetCur(int value)
{
    link1MotorDutyPresetCur = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDeceleration() const
{
    return link1MotorDeceleration;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDeceleration(int value)
{
    link1MotorDeceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorAcceleration() const
{
    return link1MotorAcceleration;
}

void PlatformManipulatorAndIRBumper::setLink1MotorAcceleration(int value)
{
    link1MotorAcceleration = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDutyMax() const
{
    return link1MotorDutyMax;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDutyMax(int value)
{
    link1MotorDutyMax = value;
}

int PlatformManipulatorAndIRBumper::getLink1MotorDuty() const
{
    return link1MotorDuty;
}

void PlatformManipulatorAndIRBumper::setLink1MotorDuty(int value)
{
    link1MotorDuty = value;
}

bool PlatformManipulatorAndIRBumper::getLink1MotorStop() const
{
    return link1MotorStop;
}

void PlatformManipulatorAndIRBumper::setLink1MotorStop(bool value)
{
    link1MotorStop = value;
}

bool PlatformManipulatorAndIRBumper::prepareManLink1Movement()
{
    return true;
}

bool PlatformManipulatorAndIRBumper::prepareManLink2Movement()
{
    return true;
}

bool PlatformManipulatorAndIRBumper::getLink1MotorActivated() const
{
    return link1MotorActivated;
}

void PlatformManipulatorAndIRBumper::setLink1MotorActivated(bool value)
{
    link1MotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setLink1MotorStop(false);
    }
}

bool PlatformManipulatorAndIRBumper::getLink1MovementDirection() const
{
    return link1MovementDirection;
}

bool PlatformManipulatorAndIRBumper::getLink2MotorDecelerating() const
{
    return link2MotorDecelerating;
}

void PlatformManipulatorAndIRBumper::setLink2MotorDecelerating(bool value)
{
    link2MotorDecelerating = value;
}

bool PlatformManipulatorAndIRBumper::getLink2MotorAccelerating() const
{
    return link2MotorAccelerating;
}

void PlatformManipulatorAndIRBumper::setLink2MotorAccelerating(bool value)
{
    link2MotorAccelerating = value;
}

string PlatformManipulatorAndIRBumper::getControlDeviceId()
{
    return controlDeviceId;
}

int PlatformManipulatorAndIRBumper::getManGripperRotationMotorDuty() const
{
    return manGripperRotationMotorDuty;
}

void PlatformManipulatorAndIRBumper::setManGripperRotationMotorDuty(int value)
{
    manGripperRotationMotorDuty = value;
}

bool PlatformManipulatorAndIRBumper::getLink2CurrentADC() const
{
    return link2CurrentADC;
}

void PlatformManipulatorAndIRBumper::setLink2CurrentADC(bool value)
{
    link2CurrentADC = value;
}

bool PlatformManipulatorAndIRBumper::getLink2CurrentTrack() const
{
    return link2CurrentTrack;
}

void PlatformManipulatorAndIRBumper::setLink2CurrentTrack(bool value)
{
    link2CurrentTrack = value;
}

bool PlatformManipulatorAndIRBumper::getLink1CurrentADC() const
{
    return link1CurrentADC;
}

void PlatformManipulatorAndIRBumper::setLink1CurrentADC(bool value)
{
    link1CurrentADC = value;
}

bool PlatformManipulatorAndIRBumper::getLink1CurrentTrack() const
{
    return link1CurrentTrack;
}

void PlatformManipulatorAndIRBumper::setLink1CurrentTrack(bool value)
{
    link1CurrentTrack = value;
}

bool PlatformManipulatorAndIRBumper::getLink2PositionADC() const
{
    return link2PositionADC;
}

void PlatformManipulatorAndIRBumper::setLink2PositionADC(bool value)
{
    link2PositionADC = value;
}

bool PlatformManipulatorAndIRBumper::getLink2PositionTrack() const
{
    return link2PositionTrack;
}

void PlatformManipulatorAndIRBumper::setLink2PositionTrack(bool value)
{
    link2PositionTrack = value;
}

bool PlatformManipulatorAndIRBumper::getLink1PositionADC() const
{
    return link1PositionADC;
}

void PlatformManipulatorAndIRBumper::setLink1PositionADC(bool value)
{
    link1PositionADC = value;
}

bool PlatformManipulatorAndIRBumper::getLink1PositionTrack() const
{
    return link1PositionTrack;
}

void PlatformManipulatorAndIRBumper::setLink1PositionTrack(bool value)
{
    link1PositionTrack = value;
}

double PlatformManipulatorAndIRBumper::getLink2Current() const
{
    return link2Current;
}

void PlatformManipulatorAndIRBumper::setLink2Current(double value)
{
    link2Current = value;
}

double PlatformManipulatorAndIRBumper::getLink1Current() const
{
    return link1Current;
}

void PlatformManipulatorAndIRBumper::setLink1Current(double value)
{
    link1Current = value;
}

double PlatformManipulatorAndIRBumper::getLink2Position() const
{
    return link2Position;
}

void PlatformManipulatorAndIRBumper::setLink2Position(double value)
{
    link2Position = value;
}

double PlatformManipulatorAndIRBumper::getLink1Position() const
{
    return link1Position;
}

void PlatformManipulatorAndIRBumper::setLink1Position(double value)
{
    link1Position = value;
}
