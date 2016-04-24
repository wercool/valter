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

    //rightForearmMotorDuty
    defaultValue = getDefault("rightForearmMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightForearmMotorDutyPresetMin(minValue);
    setRightForearmMotorDutyPresetMax(maxValue);
    setRightForearmMotorDutyPresetCur(curValue);
    setRightForearmMotorDutyMax(getRightForearmMotorDutyPresetCur());

    //rightForearmMotorDeceleration
    defaultValue = getDefault("rightForearmMotorDeceleration");
    rightForearmMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightForearmMotorAcceleration
    defaultValue = getDefault("rightForearmMotorAcceleration");
    rightForearmMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightArmMotorDuty
    defaultValue = getDefault("rightArmMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightArmMotorDutyPresetMin(minValue);
    setRightArmMotorDutyPresetMax(maxValue);
    setRightArmMotorDutyPresetCur(curValue);
    setRightArmMotorDutyMax(getRightArmMotorDutyPresetCur());

    //rightArmMotorDeceleration
    defaultValue = getDefault("rightArmMotorDeceleration");
    rightArmMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightArmMotorAcceleration
    defaultValue = getDefault("rightArmMotorAcceleration");
    rightArmMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightLimbMotorDuty
    defaultValue = getDefault("rightLimbMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightLimbMotorDutyPresetMin(minValue);
    setRightLimbMotorDutyPresetMax(maxValue);
    setRightLimbMotorDutyPresetCur(curValue);
    setRightLimbMotorDutyMax(getRightLimbMotorDutyPresetCur());

    //rightLimbMotorDeceleration
    defaultValue = getDefault("rightLimbMotorDeceleration");
    rightLimbMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightLimbMotorAcceleration
    defaultValue = getDefault("rightLimbMotorAcceleration");
    rightLimbMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));
    //forearmPosition
    defaultValue = getDefault("forearmPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setForearmPositionTrack(val1);
    setForearmPositionADC(val2);

    //armPosition
    defaultValue = getDefault("armPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setArmPositionTrack(val1);
    setArmPositionADC(val2);

    //limbPosition
    defaultValue = getDefault("limbPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLimbPositionTrack(val1);
    setLimbPositionADC(val2);

    //forearmMotorCurrent
    defaultValue = getDefault("forearmMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setForearmMotorCurrentTrack(val1);
    setForearmMotorCurrentADC(val2);

    //armMotorCurrent
    defaultValue = getDefault("armMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setArmMotorCurrentTrack(val1);
    setArmMotorCurrentADC(val2);

    //limbMotorCurrent
    defaultValue = getDefault("limbMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLimbMotorCurrentTrack(val1);
    setLimbMotorCurrentADC(val2);

    //handYawPosition
    defaultValue = getDefault("handYawPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setHandYawPositionTrack(val1);
    setHandYawPositionADC(val2);

    //handPitchPosition
    defaultValue = getDefault("handPitchPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setHandPitchPositionTrack(val1);
    setHandPitchPositionADC(val2);

    //forearmYawPosition
    defaultValue = getDefault("forearmYawPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setForearmYawPositionTrack(val1);
    setForearmYawPositionADC(val2);

    //forearmYawMotorCurrent
    defaultValue = getDefault("forearmYawMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setForearmYawMotorCurrentTrack(val1);
    setForearmYawMotorCurrentADC(val2);

    //handYawMotorCurrent
    defaultValue = getDefault("handYawMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setHandYawMotorCurrentTrack(val1);
    setHandYawMotorCurrentADC(val2);

    //handPitchMotorCurrent
    defaultValue = getDefault("handPitchMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setHandPitchMotorCurrentTrack(val1);
    setHandPitchMotorCurrentADC(val2);

    //trackHandSensors
    defaultValue = getDefault("trackHandSensors");
    vector<string>defaultValue_str_values = Valter::split(defaultValue, ',');
    vector<string>::iterator iter = defaultValue_str_values.begin();
    int idx = 0;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        setHandSensorsTrack(idx++, ((atoi(defaultValuePtr)) ? true : false));
    }
}
