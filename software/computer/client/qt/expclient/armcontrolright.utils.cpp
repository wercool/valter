#include "valter.h"
#include "armcontrolright.h"

void ArmControlRight::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    rightForearmMotorDuty                = 1;
    rightForearmMotorDutyMax             = 40;
    rightForearmMotorAcceleration        = 2;
    rightForearmMotorDeceleration        = 2;

    rightForearmMotorAccelerating        = false;
    rightForearmMotorDecelerating        = false;

    rightForearmMotorMovementDirection   = true;        //true - down, false - up
    rightForearmMotorActivated           = false;
    rightForearmMotorStop                = true;

    rightForearmADCPosition              = 0;
    rightForearmADCCurrent               = 0;

    //presets
    rightForearmMotorDutyPresetCur       = 40;
    rightForearmMotorDutyPresetMin       = 1;
    rightForearmMotorDutyPresetMax       = 100;

    //---------------arm
    rightArmMotorDuty                    = 1;
    rightArmMotorDutyMax                 = 40;
    rightArmMotorAcceleration            = 2;
    rightArmMotorDeceleration            = 2;

    rightArmMotorAccelerating            = false;
    rightArmMotorDecelerating            = false;

    rightArmMotorMovementDirection       = true;        //true - down, false - up
    rightArmMotorActivated               = false;
    rightArmMotorStop                    = true;

    rightArmADCPosition                  = 0;
    rightArmADCCurrent                   = 0;

    //presets
    rightArmMotorDutyPresetCur           = 40;
    rightArmMotorDutyPresetMin           = 1;
    rightArmMotorDutyPresetMax           = 100;

    //---------------limb
    rightLimbMotorDuty                   = 1;
    rightLimbMotorDutyMax                = 40;
    rightLimbMotorAcceleration           = 2;
    rightLimbMotorDeceleration           = 2;

    rightLimbMotorAccelerating           = false;
    rightLimbMotorDecelerating           = false;

    rightLimbMotorMovementDirection      = true;        //true - down, false - up
    rightLimbMotorActivated              = false;
    rightLimbMotorStop                   = true;

    rightLimbADCPosition                 = 0;
    rightLimbADCCurrent                  = 0;

    //presets
    rightLimbMotorDutyPresetCur          = 40;
    rightLimbMotorDutyPresetMin          = 1;
    rightLimbMotorDutyPresetMax          = 100;

    //------------------------hand yaw
    handYawDirection                    = false;

    //------------------------hand pitch
    handPitchDirection                  = false;  //true - up, false - down

    //------------------------forearm roll
    forearmRollDirection                = true;  //true - CCW, false - CW
    forearmRollMotorState               = false;  //true - on, false - off
    forearmRollMotorActivated           = false;
    forearmRollStepSwitchDelay          = 100;
    forearmRollStepDelay                = 2500;
    forearmRollStepPosition             = 0;
    forearmRollCWLimit                  = false;
    forearmRollCCWLimit                 = false;
    forearmRollResettingStepPosition    = false;

    //------------------------forearm yaw
    handYawDirection                    = true; //true - CW, false - CCW

    armLedsState                        = false;

    //right arm readings presets
    forearmPositionTrack                = true;
    forearmPositionADC                  = true;
    armPositionTrack                    = true;
    armPositionADC                      = true;
    limbPositionTrack                   = true;
    limbPositionADC                     = true;
    forearmMotorCurrentTrack            = true;
    forearmMotorCurrentADC              = true;
    armMotorCurrentTrack                = true;
    armMotorCurrentADC                  = true;
    limbMotorCurrentTrack               = true;
    limbMotorCurrentADC                 = true;
    handYawPositionTrack                = true;
    handYawPositionADC                  = true;
    handPitchPositionTrack              = true;
    handPitchPositionADC                = true;
    forearmYawPositionTrack             = true;
    forearmYawPositionADC               = true;
    forearmYawMotorCurrentTrack         = true;
    forearmYawMotorCurrentADC           = true;
    handYawMotorCurrentTrack            = true;
    handYawMotorCurrentADC              = true;
    handPitchMotorCurrentTrack          = true;
    handPitchMotorCurrentADC            = true;

    //right arm readings
    forearmADCPosition                  = 0;
    armADCPosition                      = 0;
    limbADCPosition                     = 0;
    forearmMotorADCCurrent              = 0;
    armMotorADCCurrent                  = 0;
    limbMotorADCCurrent                 = 0;
    handYawADCPosition                  = 0;
    handPitchADCPosition                = 0;
    forearmYawADCPosition               = 0;
    forearmYawMotorADCCurrent           = 0;
    handYawMotorADCCurrent              = 0;
    handPitchMotorADCCurrent            = 0;

    //hand finger sesnors preset
    handSensorsTrack[13]                = {false};
    handSensorsADCForce[13]             = {0};
}

void ArmControlRight::loadDefaults()
{
    ifstream defaultsFile(ArmControlRight::defaultsFilePath);
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
}
