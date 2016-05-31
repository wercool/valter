#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"


PlatformManipulatorAndIRBumper *PlatformManipulatorAndIRBumper::pPlatformManipulatorAndIRBumper = NULL;
bool PlatformManipulatorAndIRBumper::instanceFlag = false;
const string PlatformManipulatorAndIRBumper::controlDeviceId = "PLATFORM-MANIPULATOR-AND-IR-BUMPER";
const string PlatformManipulatorAndIRBumper::defaultsFilePath = "settings/platform-manipulator-and-ir-bumper-defaults";

const double PlatformManipulatorAndIRBumper::rootX = 220;
const double PlatformManipulatorAndIRBumper::rootY = 130;
const double PlatformManipulatorAndIRBumper::man_l1      = 120;
const double PlatformManipulatorAndIRBumper::man_l2      = 128;
const double PlatformManipulatorAndIRBumper::man_l1_l2   = 30;
const double PlatformManipulatorAndIRBumper::man_l2_l3   = 10;
const double PlatformManipulatorAndIRBumper::man_l3      = 40;
double PlatformManipulatorAndIRBumper::man_a    = 0.0;
double PlatformManipulatorAndIRBumper::man_b    = 0.0;
double PlatformManipulatorAndIRBumper::man_g    = 0.0;

void PlatformManipulatorAndIRBumper::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    power24VOnOff           = false;

    //link1
    link1MovementDirection  = false;
    link1MotorActivated     = false;
    link1MotorStop          = true;

    link1MotorDuty          = 1;
    link1MotorDutyMax       = 30;
    link1MotorAcceleration  = 2;
    link1MotorDeceleration  = 2;

    link1MotorAccelerating  = false;
    link1MotorDecelerating  = false;

    link1MotorDutyPresetCur = 30;
    link1MotorDutyPresetMin = 1;
    link1MotorDutyPresetMax = 1;

    //link2
    link2MovementDirection  = false;
    link2MotorActivated     = false;
    link2MotorStop          = false;

    link2MotorDuty          = 1;
    link2MotorDutyMax       = 30;
    link2MotorAcceleration  = 2;
    link2MotorDeceleration  = 2;

    link2MotorAccelerating  = false;
    link2MotorDecelerating  = false;

    link2MotorDutyPresetCur = false;
    link2MotorDutyPresetMin = false;
    link2MotorDutyPresetMax = false;

    manGripperRotationMotorDuty         = 1;

    link1PositionTrack      = true;
    link1PositionADC        = true;
    link2PositionTrack      = true;
    link2PositionADC        = true;
    link1CurrentTrack       = true;
    link1CurrentADC         = true;
    link2CurrentTrack       = true;
    link2CurrentADC         = true;

    gripperForceSensor1Track            = true;
    gripperForceSensor2Track            = true;
    gripperForceSensor3Track            = true;
    gripperObjectDetectorTrack          = true;
    gripperTiltMotorCurrentTrack        = true;
    gripperOpenCloseMotorCurrentTrack   = true;
    gripperRotationMotorCurrentTrack    = true;

    gripperForceSensor1ADC              = true;
    gripperForceSensor2ADC              = true;
    gripperForceSensor3ADC              = true;
    gripperObjectDetectorADC            = true;
    gripperTiltMotorCurrentADC          = true;
    gripperOpenCloseMotorCurrentADC     = true;
    gripperRotationMotorCurrentADC      = true;

    link1Position   = 0;
    link2Position   = 0;
    link1Current    = 0;
    link2Current    = 0;

    link1ADCPosition    = 0;
    link2ADCPosition    = 0;
    link1ADCCurrent     = 0;
    link2ADCCurrent     = 0;

    gripperForceSensor1ADCValue             = 0;
    gripperForceSensor2ADCValue             = 0;
    gripperForceSensor3ADCValue             = 0;
    gripperObjectDetectorADCValue           = 0;
    gripperTiltMotorCurrentADCValue         = 0;
    gripperOpenCloseMotorCurrentADCValue    = 0;
    gripperRotationMotorCurrentADCValue     = 0;

    for (int i = 0; i < 16; i++)
    {
        irBumperTrack[i] = true;
        irBumperTicks[i] = true;
        irBumperTicksReading[i] = 0;
    }

    irBumperInitialized = false;
    irBumperEnabled     = false;

    irBumperFrequency   = 38000;
    irBumperDuty        = 50;
}

void PlatformManipulatorAndIRBumper::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + PlatformManipulatorAndIRBumper::defaultsFilePath);
    string line;
    while (getline(defaultsFile, line, '\n'))
    {
        if (line.substr(0, 2).compare("//") != 0 && line.length() > 0)
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
    setLink1MotorDutyMax(getLink1MotorDutyPresetCur());

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
    setLink2MotorDutyMax(getLink2MotorDutyPresetCur());

    //link2MotorAcceleration
    defaultValue = getDefault("link2MotorAcceleration");
    link2MotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //link2MotorDeceleration
    defaultValue = getDefault("link2MotorDeceleration");
    link2MotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //manGripperRotationMotorDuty
    defaultValue = getDefault("manGripperRotationMotorDuty");
    manGripperRotationMotorDuty = atoi(Valter::stringToCharPtr(defaultValue));

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

    //CH0 Gripper Tilt      CH0
    defaultValue = getDefault("CH0");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperTiltTrack(val1);
    setGripperTiltADC(val2);

    //CH1 Gripper Rotation      CH1
    defaultValue = getDefault("CH1");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperRotationTrack(val1);
    setGripperRotationADC(val2);

    //CH2 Gripper Position      CH2
    defaultValue = getDefault("CH2");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperPositionTrack(val1);
    setGripperPositionADC(val2);

    //CH3 Gripper Force Sensor 1      CH3
    defaultValue = getDefault("CH3");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperForceSensor1Track(val1);
    setGripperForceSensor1ADC(val2);

    //CH4 Gripper Force Sensor 2      CH4
    defaultValue = getDefault("CH4");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperForceSensor2Track(val1);
    setGripperForceSensor2ADC(val2);

    //CH5 Gripper Force Sensor 3      CH5
    defaultValue = getDefault("CH5");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperForceSensor3Track(val1);
    setGripperForceSensor3ADC(val2);

    //CH6 Gripper Object Detector      CH6
    defaultValue = getDefault("CH6");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperObjectDetectorTrack(val1);
    setGripperObjectDetectorADC(val2);

    //CH7 Gripper Tilt Motor Current      CH7
    defaultValue = getDefault("CH7");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperTiltMotorCurrentTrack(val1);
    setGripperTiltMotorCurrentADC(val2);

    //CH8 Gripper Open/Close Motor Current      CH8
    defaultValue = getDefault("CH8");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperOpenCloseMotorCurrentTrack(val1);
    setGripperOpenCloseMotorCurrentADC(val2);

    //CH9 Gripper Rotation Motor Current      CH9
    defaultValue = getDefault("CH9");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setGripperRotationMotorCurrentTrack(val1);
    setGripperRotationMotorCurrentADC(val2);

    //irBumperTrack
    defaultValue = getDefault("irBumperTrack");
    vector<string>defaultValue_str_values = Valter::split(defaultValue, ',');
    vector<string>::iterator iter = defaultValue_str_values.begin();
    int idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setIRBumperTrack(idx++, ((atoi(defaultValuePtr)) ? true : false));
    }

    //irBumperTicks
    defaultValue = getDefault("irBumperTicks");
    defaultValue_str_values = Valter::split(defaultValue, ',');
    iter = defaultValue_str_values.begin();
    idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setIRBumperTicks(idx++, ((atoi(defaultValuePtr)) ? true : false));
    }
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
    sendCommand(Valter::format_string("SETLINK2DRIVEDUTY#%d", link2MotorDuty));
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
    sendCommand(Valter::format_string("SETLINK1DRIVEDUTY#%d", link1MotorDuty));
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
    if (getLink1MotorAccelerating() || getLink1MotorDecelerating())
    {
        return false;
    }
    else
    {
        setLink1MotorDuty(getLink1MotorDutyPresetMin());
        return true;
    }
}

bool PlatformManipulatorAndIRBumper::prepareManLink2Movement()
{
    if (getLink2MotorAccelerating() || getLink2MotorDecelerating())
    {
        return false;
    }
    else
    {
        setLink2MotorDuty(getLink2MotorDutyPresetMin());
        return true;
    }
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

double PlatformManipulatorAndIRBumper::getGripperTilt() const
{
    return gripperTilt;
}

void PlatformManipulatorAndIRBumper::setGripperTilt(double value)
{
    gripperTilt = value;
}

int PlatformManipulatorAndIRBumper::getGripperADCTilt() const
{
    return gripperADCTilt;
}

void PlatformManipulatorAndIRBumper::setGripperADCTilt(int value)
{
    gripperADCTilt = value;
}

bool PlatformManipulatorAndIRBumper::getGripperTiltADC() const
{
    return gripperTiltADC;
}

void PlatformManipulatorAndIRBumper::setGripperTiltADC(bool value)
{
    gripperTiltADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperTiltTrack() const
{
    return gripperTiltTrack;
}

void PlatformManipulatorAndIRBumper::setGripperTiltTrack(bool value)
{
    gripperTiltTrack = value;
}

int PlatformManipulatorAndIRBumper::getLink2ADCCurrent() const
{
    return link2ADCCurrent;
}

void PlatformManipulatorAndIRBumper::setLink2ADCCurrent(int value)
{
    link2ADCCurrent = value;
}

int PlatformManipulatorAndIRBumper::getLink1ADCCurrent() const
{
    return link1ADCCurrent;
}

void PlatformManipulatorAndIRBumper::setLink1ADCCurrent(int value)
{
    link1ADCCurrent = value;
}

int PlatformManipulatorAndIRBumper::getLink2ADCPosition() const
{
    return link2ADCPosition;
}

void PlatformManipulatorAndIRBumper::setLink2ADCPosition(int value)
{
    link2ADCPosition = value;
    double degressValue = ((double)link2ADCPosition - 136) / 9.02; //max 948, min 136, 90deg max. 136 ~ 0, 948-136=812 812/90 = 9.02
    setLink2Position(degressValue);
}

int PlatformManipulatorAndIRBumper::getLink1ADCPosition() const
{
    return link1ADCPosition;
}

void PlatformManipulatorAndIRBumper::setLink1ADCPosition(int value)
{
    link1ADCPosition = value;
    double degressValue = link1ADCPosition / 16; //max 0, min 1023, 64deg max. 1023 ~ 0, 1024/64 = 16
    setLink1Position(degressValue);
}

double PlatformManipulatorAndIRBumper::getGripperPosition() const
{
    return gripperPosition;
}

void PlatformManipulatorAndIRBumper::setGripperPosition(double value)
{
    gripperPosition = value;
}

void PlatformManipulatorAndIRBumper::setIRBumperTrack(int idx, bool state)
{
    irBumperTrack[idx] = state;
}

bool PlatformManipulatorAndIRBumper::getIRBumperTrack(int idx)
{
    return irBumperTrack[idx];
}

bool PlatformManipulatorAndIRBumper::getIRBumperTracked()
{
    bool state = false;
    for (int i = 0; i < 16; i++)
    {
        if (getIRBumperTrack(i))
        {
            state = true;
        }
    }
    return state;
}

void PlatformManipulatorAndIRBumper::setIRBumperTicks(int idx, bool state)
{
    irBumperTicks[idx] = state;
}

bool PlatformManipulatorAndIRBumper::getIRBumperTicks(int idx)
{
    return irBumperTicks[idx];
}

double PlatformManipulatorAndIRBumper::getGripperRotation() const
{
    return gripperRotation;
}

void PlatformManipulatorAndIRBumper::setGripperRotation(double value)
{
    gripperRotation = value;
}

int PlatformManipulatorAndIRBumper::getGripperADCPosition() const
{
    return gripperADCPosition;
}

void PlatformManipulatorAndIRBumper::setGripperADCPosition(int value)
{
    gripperADCPosition = value;
}

int PlatformManipulatorAndIRBumper::getGripperADCRotation() const
{
    return gripperADCRotation;
}

void PlatformManipulatorAndIRBumper::setGripperADCRotation(int value)
{
    gripperADCRotation = value;
}

bool PlatformManipulatorAndIRBumper::getGripperPositionADC() const
{
    return gripperPositionADC;
}

void PlatformManipulatorAndIRBumper::setGripperPositionADC(bool value)
{
    gripperPositionADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperPositionTrack() const
{
    return gripperPositionTrack;
}

void PlatformManipulatorAndIRBumper::setGripperPositionTrack(bool value)
{
    gripperPositionTrack = value;
}

bool PlatformManipulatorAndIRBumper::getGripperRotationADC() const
{
    return gripperRotationADC;
}

void PlatformManipulatorAndIRBumper::setGripperRotationADC(bool value)
{
    gripperRotationADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperRotationTrack() const
{
    return gripperRotationTrack;
}

void PlatformManipulatorAndIRBumper::setGripperRotationTrack(bool value)
{
    gripperRotationTrack = value;
}

bool PlatformManipulatorAndIRBumper::getGripperRotationMotorCurrentADC() const
{
    return gripperRotationMotorCurrentADC;
}

void PlatformManipulatorAndIRBumper::setGripperRotationMotorCurrentADC(bool value)
{
    gripperRotationMotorCurrentADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperOpenCloseMotorCurrentADC() const
{
    return gripperOpenCloseMotorCurrentADC;
}

void PlatformManipulatorAndIRBumper::setGripperOpenCloseMotorCurrentADC(bool value)
{
    gripperOpenCloseMotorCurrentADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperTiltMotorCurrentADC() const
{
    return gripperTiltMotorCurrentADC;
}

void PlatformManipulatorAndIRBumper::setGripperTiltMotorCurrentADC(bool value)
{
    gripperTiltMotorCurrentADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperObjectDetectorADC() const
{
    return gripperObjectDetectorADC;
}

void PlatformManipulatorAndIRBumper::setGripperObjectDetectorADC(bool value)
{
    gripperObjectDetectorADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperForceSensor3ADC() const
{
    return gripperForceSensor3ADC;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor3ADC(bool value)
{
    gripperForceSensor3ADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperForceSensor2ADC() const
{
    return gripperForceSensor2ADC;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor2ADC(bool value)
{
    gripperForceSensor2ADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperForceSensor1ADC() const
{
    return gripperForceSensor1ADC;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor1ADC(bool value)
{
    gripperForceSensor1ADC = value;
}

bool PlatformManipulatorAndIRBumper::getGripperRotationMotorCurrentTrack() const
{
    return gripperRotationMotorCurrentTrack;
}

void PlatformManipulatorAndIRBumper::setGripperRotationMotorCurrentTrack(bool value)
{
    gripperRotationMotorCurrentTrack = value;
}

bool PlatformManipulatorAndIRBumper::getGripperOpenCloseMotorCurrentTrack() const
{
    return gripperOpenCloseMotorCurrentTrack;
}

void PlatformManipulatorAndIRBumper::setGripperOpenCloseMotorCurrentTrack(bool value)
{
    gripperOpenCloseMotorCurrentTrack = value;
}

bool PlatformManipulatorAndIRBumper::getGripperTiltMotorCurrentTrack() const
{
    return gripperTiltMotorCurrentTrack;
}

void PlatformManipulatorAndIRBumper::setGripperTiltMotorCurrentTrack(bool value)
{
    gripperTiltMotorCurrentTrack = value;
}

bool PlatformManipulatorAndIRBumper::getGripperObjectDetectorTrack() const
{
    return gripperObjectDetectorTrack;
}

void PlatformManipulatorAndIRBumper::setGripperObjectDetectorTrack(bool value)
{
    gripperObjectDetectorTrack = value;
}

bool PlatformManipulatorAndIRBumper::getGripperForceSensor3Track() const
{
    return gripperForceSensor3Track;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor3Track(bool value)
{
    gripperForceSensor3Track = value;
}

bool PlatformManipulatorAndIRBumper::getGripperForceSensor2Track() const
{
    return gripperForceSensor2Track;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor2Track(bool value)
{
    gripperForceSensor2Track = value;
}

bool PlatformManipulatorAndIRBumper::getGripperForceSensor1Track() const
{
    return gripperForceSensor1Track;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor1Track(bool value)
{
    gripperForceSensor1Track = value;
}

int PlatformManipulatorAndIRBumper::getIrBumperDuty() const
{
    return irBumperDuty;
}

int PlatformManipulatorAndIRBumper::getGripperRotationMotorCurrentADCValue() const
{
    return gripperRotationMotorCurrentADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperRotationMotorCurrentADCValue(int value)
{
    gripperRotationMotorCurrentADCValue = value;
}

int PlatformManipulatorAndIRBumper::getGripperOpenCloseMotorCurrentADCValue() const
{
    return gripperOpenCloseMotorCurrentADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperOpenCloseMotorCurrentADCValue(int value)
{
    gripperOpenCloseMotorCurrentADCValue = value;
}

int PlatformManipulatorAndIRBumper::getGripperTiltMotorCurrentADCValue() const
{
    return gripperTiltMotorCurrentADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperTiltMotorCurrentADCValue(int value)
{
    gripperTiltMotorCurrentADCValue = value;
}

int PlatformManipulatorAndIRBumper::getGripperObjectDetectorADCValue() const
{
    return gripperObjectDetectorADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperObjectDetectorADCValue(int value)
{
    gripperObjectDetectorADCValue = value;
}

int PlatformManipulatorAndIRBumper::getGripperForceSensor3ADCValue() const
{
    return gripperForceSensor3ADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor3ADCValue(int value)
{
    gripperForceSensor3ADCValue = value;
}

int PlatformManipulatorAndIRBumper::getGripperForceSensor2ADCValue() const
{
    return gripperForceSensor2ADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor2ADCValue(int value)
{
    gripperForceSensor2ADCValue = value;
}

int PlatformManipulatorAndIRBumper::getGripperForceSensor1ADCValue() const
{
    return gripperForceSensor1ADCValue;
}

void PlatformManipulatorAndIRBumper::setGripperForceSensor1ADCValue(int value)
{
    gripperForceSensor1ADCValue = value;
}
