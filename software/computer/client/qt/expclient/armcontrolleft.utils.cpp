#include "valter.h"
#include "armcontrolleft.h"

void ArmControlLeft::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    leftForearmMotorDuty                = 1;
    leftForearmMotorDutyMax             = 40;
    leftForearmMotorAcceleration        = 2;
    leftForearmMotorDeceleration        = 2;

    leftForearmMotorAccelerating        = false;
    leftForearmMotorDecelerating        = false;

    leftForearmMotorMovementDirection   = true;        //true - down, false - up
    leftForearmMotorActivated           = false;
    leftForearmMotorStop                = true;

    leftForearmADCPosition              = 0;
    leftForearmADCCurrent               = 0;

    //presets
    leftForearmMotorDutyPresetCur       = 40;
    leftForearmMotorDutyPresetMin       = 1;
    leftForearmMotorDutyPresetMax       = 100;

    //---------------arm
    leftArmMotorDuty                    = 1;
    leftArmMotorDutyMax                 = 40;
    leftArmMotorAcceleration            = 2;
    leftArmMotorDeceleration            = 2;

    leftArmMotorAccelerating            = false;
    leftArmMotorDecelerating            = false;

    leftArmMotorMovementDirection       = true;        //true - down, false - up
    leftArmMotorActivated               = false;
    leftArmMotorStop                    = true;

    leftArmADCPosition                  = 0;
    leftArmADCCurrent                   = 0;

    //presets
    leftArmMotorDutyPresetCur           = 40;
    leftArmMotorDutyPresetMin           = 1;
    leftArmMotorDutyPresetMax           = 100;

    //---------------limb
    leftLimbMotorDuty                   = 1;
    leftLimbMotorDutyMax                = 40;
    leftLimbMotorAcceleration           = 2;
    leftLimbMotorDeceleration           = 2;

    leftLimbMotorAccelerating           = false;
    leftLimbMotorDecelerating           = false;

    leftLimbMotorMovementDirection      = true;        //true - down, false - up
    leftLimbMotorActivated              = false;
    leftLimbMotorStop                   = true;

    leftLimbADCPosition                 = 0;
    leftLimbADCCurrent                  = 0;

    //presets
    leftLimbMotorDutyPresetCur          = 40;
    leftLimbMotorDutyPresetMin          = 1;
    leftLimbMotorDutyPresetMax          = 100;

    //------------------------hand yaw
    handYawDirection                    = false;

    //------------------------hand pitch
    handPitchDirection                  = false;  //true - up, false - down

    //------------------------forearm roll
    forearmRollDirection                = true;  //true - CCW, false - CW
    forearmRollMotorState               = false;  //true - on, false - off
}

void ArmControlLeft::loadDefaults()
{
    ifstream defaultsFile(ArmControlLeft::defaultsFilePath);
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
    int val1, val2;

    //leftForearmMotorDuty
    defaultValue = getDefault("leftForearmMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftForearmMotorDutyPresetMin(minValue);
    setLeftForearmMotorDutyPresetMax(maxValue);
    setLeftForearmMotorDutyPresetCur(curValue);
    setLeftForearmMotorDutyMax(getLeftForearmMotorDutyPresetCur());

    //leftForearmMotorDeceleration
    defaultValue = getDefault("leftForearmMotorDeceleration");
    leftForearmMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftForearmMotorAcceleration
    defaultValue = getDefault("leftForearmMotorAcceleration");
    leftForearmMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftArmMotorDuty
    defaultValue = getDefault("leftArmMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftArmMotorDutyPresetMin(minValue);
    setLeftArmMotorDutyPresetMax(maxValue);
    setLeftArmMotorDutyPresetCur(curValue);
    setLeftArmMotorDutyMax(getLeftArmMotorDutyPresetCur());

    //leftArmMotorDeceleration
    defaultValue = getDefault("leftArmMotorDeceleration");
    leftArmMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftArmMotorAcceleration
    defaultValue = getDefault("leftArmMotorAcceleration");
    leftArmMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));


    //leftLimbMotorDuty
    defaultValue = getDefault("leftLimbMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftLimbMotorDutyPresetMin(minValue);
    setLeftLimbMotorDutyPresetMax(maxValue);
    setLeftLimbMotorDutyPresetCur(curValue);
    setLeftLimbMotorDutyMax(getLeftLimbMotorDutyPresetCur());

    //leftLimbMotorDeceleration
    defaultValue = getDefault("leftLimbMotorDeceleration");
    leftLimbMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftLimbMotorAcceleration
    defaultValue = getDefault("leftLimbMotorAcceleration");
    leftLimbMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));
}
int ArmControlLeft::getLeftLimbMotorDutyPresetMax() const
{
    return leftLimbMotorDutyPresetMax;
}

void ArmControlLeft::setLeftLimbMotorDutyPresetMax(int value)
{
    leftLimbMotorDutyPresetMax = value;
}

