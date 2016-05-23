#include "valter.h"

void ArmControlRight::resetToDefault()
{
    defaultsLoading = false;

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
    for (unsigned int i = 0 ; i < 13; i++)
    {
        handSensorsTrack[i] = false;
        handSensorsADCForce[i] = 0;
    }

    //hand finger servo positions
    for (unsigned int i = 0 ; i < 6; i++)
    {
        fingerInitialPositions[i] = 0;
        fingerGraspedPositions[i] = 0;
    }

    finger0TipThreshold         = 903;
    finger0PhalanxThreshold     = 590;
    finger1PhalanxThreshold     = 10;
    finger1TipThreshold         = 0;
    finger2PhalanxThreshold     = 550;
    finger3PhalanxThreshold     = 395;
    finger4TipThreshold         = 670;
    finger4PhalanxThreshold     = 432;
    finger5PhalanxThreshold     = 601;
    finger5TipThreshold         = 425;
    palmUpperThreshold          = 680;
    palmLowerThreshold          = 640;
    palmJambThreshold           = 980;

    finger0TipReading           = 0;
    finger0PhalanxReading       = 0;
    finger1PhalanxReading       = 0;
    finger1TipReading           = 0;
    finger2PhalanxReading       = 0;
    finger3PhalanxReading       = 0;
    finger4TipReading           = 0;
    finger4PhalanxReading       = 0;
    finger5PhalanxReading       = 0;
    finger5TipReading           = 0;
    palmUpperReading            = 0;
    palmLowerReading            = 0;
    palmJambReading             = 0;

    finger0TipReadingRelative       = 0;
    finger0PhalanxReadingRelative   = 0;
    finger1PhalanxReadingRelative   = 0;
    finger1TipReadingRelative       = 0;
    finger2PhalanxReadingRelative   = 0;
    finger3PhalanxReadingRelative   = 0;
    finger4TipReadingRelative       = 0;
    finger4PhalanxReadingRelative   = 0;
    finger5PhalanxReadingRelative   = 0;
    finger5TipReadingRelative       = 0;
    palmUpperReadingRelative        = 0;
    palmLowerReadingRelative        = 0;
    palmJambReadingRelative         = 0;
}

void ArmControlRight::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + ArmControlRight::defaultsFilePath);
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

    //finger0InitialGrasped
    defaultValue = getDefault("finger0InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[0] = val1;
    fingerGraspedPositions[0] = val2;

    //finger1InitialGrasped
    defaultValue = getDefault("finger1InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[1] = val1;
    fingerGraspedPositions[1] = val2;

    //finger2InitialGrasped
    defaultValue = getDefault("finger2InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[2] = val1;
    fingerGraspedPositions[2] = val2;

    //finger3InitialGrasped
    defaultValue = getDefault("finger3InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[3] = val1;
    fingerGraspedPositions[3] = val2;

    //finger4InitialGrasped
    defaultValue = getDefault("finger4InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[4] = val1;
    fingerGraspedPositions[4] = val2;

    //finger5InitialGrasped
    defaultValue = getDefault("finger5InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[5] = val1;
    fingerGraspedPositions[5] = val2;

    //finger0Tip
    defaultValue = getDefault("finger0Tip");
    finger0TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger0Phalanx
    defaultValue = getDefault("finger0Phalanx");
    finger0PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger1Phalanx
    defaultValue = getDefault("finger1Phalanx");
    finger1PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger1Tip
    defaultValue = getDefault("finger1Tip");
    finger1TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger2Phalanx
    defaultValue = getDefault("finger2Phalanx");
    finger2PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger3Phalanx
    defaultValue = getDefault("finger3Phalanx");
    finger3PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger4Tip
    defaultValue = getDefault("finger4Tip");
    finger4TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger4Phalanx
    defaultValue = getDefault("finger4Phalanx");
    finger4PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger5Phalanx
    defaultValue = getDefault("finger5Phalanx");
    finger5PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger5Tip
    defaultValue = getDefault("finger5Tip");
    finger5TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //rightPalmUpper
    defaultValue = getDefault("rightPalmUpper");
    palmUpperThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //rightPalmLower
    defaultValue = getDefault("rightPalmUpper");
    palmUpperThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //rightPalmJamb
    defaultValue = getDefault("rightPalmJamb");
    palmJambThreshold = atoi(Valter::stringToCharPtr(defaultValue));
}

int ArmControlRight::getRightLimbMotorDutyPresetMax() const
{
    return rightLimbMotorDutyPresetMax;
}

void ArmControlRight::setRightLimbMotorDutyPresetMax(int value)
{
    rightLimbMotorDutyPresetMax = value;
}
int ArmControlRight::getRightLimbMotorDutyPresetMin() const
{
    return rightLimbMotorDutyPresetMin;
}

void ArmControlRight::setRightLimbMotorDutyPresetMin(int value)
{
    rightLimbMotorDutyPresetMin = value;
}

int ArmControlRight::getRightLimbMotorDutyPresetCur() const
{
    return rightLimbMotorDutyPresetCur;
}

void ArmControlRight::setRightLimbMotorDutyPresetCur(int value)
{
    rightLimbMotorDutyPresetCur = value;
}

int ArmControlRight::getRightLimbADCCurrent() const
{
    return rightLimbADCCurrent;
}

void ArmControlRight::setRightLimbADCCurrent(int value)
{
    rightLimbADCCurrent = value;
}

int ArmControlRight::getRightLimbADCPosition() const
{
    return rightLimbADCPosition;
}

void ArmControlRight::setRightLimbADCPosition(int value)
{
    rightLimbADCPosition = value;
}

bool ArmControlRight::getRightLimbMotorStop() const
{
    return rightLimbMotorStop;
}

void ArmControlRight::setRightLimbMotorStop(bool value)
{
    rightLimbMotorStop = value;
}

bool ArmControlRight::getRightLimbMotorActivated() const
{
    return rightLimbMotorActivated;
}

void ArmControlRight::setRightLimbMotorActivated(bool value)
{
    rightLimbMotorActivated = value;
    if (value)//if activated motor is not stopped
    {
        setRightLimbMotorStop(false);
    }
}

bool ArmControlRight::getRightLimbMotorMovementDirection() const
{
    return rightLimbMotorMovementDirection;
}

bool ArmControlRight::getRightLimbMotorDecelerating() const
{
    return rightLimbMotorDecelerating;
}

void ArmControlRight::setRightLimbMotorDecelerating(bool value)
{
    rightLimbMotorDecelerating = value;
}

bool ArmControlRight::getRightLimbMotorAccelerating() const
{
    return rightLimbMotorAccelerating;
}

void ArmControlRight::setRightLimbMotorAccelerating(bool value)
{
    rightLimbMotorAccelerating = value;
}

int ArmControlRight::getRightLimbMotorDeceleration() const
{
    return rightLimbMotorDeceleration;
}

void ArmControlRight::setRightLimbMotorDeceleration(int value)
{
    rightLimbMotorDeceleration = value;
}

int ArmControlRight::getRightLimbMotorAcceleration() const
{
    return rightLimbMotorAcceleration;
}

void ArmControlRight::setRightLimbMotorAcceleration(int value)
{
    rightLimbMotorAcceleration = value;
}

int ArmControlRight::getRightLimbMotorDutyMax() const
{
    return rightLimbMotorDutyMax;
}

void ArmControlRight::setRightLimbMotorDutyMax(int value)
{
    rightLimbMotorDutyMax = value;
}

int ArmControlRight::getRightLimbMotorDuty() const
{
    return rightLimbMotorDuty;
}

void ArmControlRight::setRightLimbMotorDuty(int value)
{
    rightLimbMotorDuty = value;
    sendCommand(Valter::format_string("SETLIMBLIFTUPDUTY#%d", rightLimbMotorDuty));
}

bool ArmControlRight::getArmLedsState() const
{
    return armLedsState;
}

void ArmControlRight::setArmLedsState(bool value)
{
    armLedsState = value;
}

int ArmControlRight::getHandPitchMotorADCCurrent() const
{
    return handPitchMotorADCCurrent;
}

void ArmControlRight::setHandPitchMotorADCCurrent(int value)
{
    handPitchMotorADCCurrent = value;
}

void ArmControlRight::setHandSensorsTrack(int idx, bool state)
{
    handSensorsTrack[idx] = state;
}

bool ArmControlRight::getHandSensorsTrack(int idx)
{
    return handSensorsTrack[idx];
}

int ArmControlRight::getHandSensorsADCForce(int idx)
{
    return handSensorsADCForce[idx];
}

int ArmControlRight::getHandYawMotorADCCurrent() const
{
    return handYawMotorADCCurrent;
}

void ArmControlRight::setHandYawMotorADCCurrent(int value)
{
    handYawMotorADCCurrent = value;
}

int ArmControlRight::getForearmYawMotorADCCurrent() const
{
    return forearmYawMotorADCCurrent;
}

void ArmControlRight::setForearmYawMotorADCCurrent(int value)
{
    forearmYawMotorADCCurrent = value;
}

int ArmControlRight::getForearmYawADCPosition() const
{
    return forearmYawADCPosition;
}

void ArmControlRight::setForearmYawADCPosition(int value)
{
    forearmYawADCPosition = value;
}

int ArmControlRight::getHandPitchADCPosition() const
{
    return handPitchADCPosition;
}

void ArmControlRight::setHandPitchADCPosition(int value)
{
    handPitchADCPosition = value;
}

int ArmControlRight::getHandYawADCPosition() const
{
    return handYawADCPosition;
}

void ArmControlRight::setHandYawADCPosition(int value)
{
    handYawADCPosition = value;
}

int ArmControlRight::getLimbMotorADCCurrent() const
{
    return limbMotorADCCurrent;
}

void ArmControlRight::setLimbMotorADCCurrent(int value)
{
    limbMotorADCCurrent = value;
}

int ArmControlRight::getArmMotorADCCurrent() const
{
    return armMotorADCCurrent;
}

void ArmControlRight::setArmMotorADCCurrent(int value)
{
    armMotorADCCurrent = value;
}

int ArmControlRight::getForearmMotorADCCurrent() const
{
    return forearmMotorADCCurrent;
}

void ArmControlRight::setForearmMotorADCCurrent(int value)
{
    forearmMotorADCCurrent = value;
}

int ArmControlRight::getLimbADCPosition() const
{
    return limbADCPosition;
}

void ArmControlRight::setLimbADCPosition(int value)
{
    limbADCPosition = value;
}

int ArmControlRight::getArmADCPosition() const
{
    return armADCPosition;
}

void ArmControlRight::setArmADCPosition(int value)
{
    armADCPosition = value;
}

int ArmControlRight::getForearmADCPosition() const
{
    return forearmADCPosition;
}

void ArmControlRight::setForearmADCPosition(int value)
{
    forearmADCPosition = value;
}

bool ArmControlRight::getHandPitchMotorCurrentADC() const
{
    return handPitchMotorCurrentADC;
}

void ArmControlRight::setHandPitchMotorCurrentADC(bool value)
{
    handPitchMotorCurrentADC = value;
}

bool ArmControlRight::getHandPitchMotorCurrentTrack() const
{
    return handPitchMotorCurrentTrack;
}

void ArmControlRight::setHandPitchMotorCurrentTrack(bool value)
{
    handPitchMotorCurrentTrack = value;
}

bool ArmControlRight::getHandYawMotorCurrentADC() const
{
    return handYawMotorCurrentADC;
}

void ArmControlRight::setHandYawMotorCurrentADC(bool value)
{
    handYawMotorCurrentADC = value;
}

bool ArmControlRight::getHandYawMotorCurrentTrack() const
{
    return handYawMotorCurrentTrack;
}

void ArmControlRight::setHandYawMotorCurrentTrack(bool value)
{
    handYawMotorCurrentTrack = value;
}

bool ArmControlRight::getForearmYawMotorCurrentADC() const
{
    return forearmYawMotorCurrentADC;
}

void ArmControlRight::setForearmYawMotorCurrentADC(bool value)
{
    forearmYawMotorCurrentADC = value;
}

bool ArmControlRight::getForearmYawMotorCurrentTrack() const
{
    return forearmYawMotorCurrentTrack;
}

void ArmControlRight::setForearmYawMotorCurrentTrack(bool value)
{
    forearmYawMotorCurrentTrack = value;
}

bool ArmControlRight::getForearmYawPositionADC() const
{
    return forearmYawPositionADC;
}

void ArmControlRight::setForearmYawPositionADC(bool value)
{
    forearmYawPositionADC = value;
}

bool ArmControlRight::getForearmYawPositionTrack() const
{
    return forearmYawPositionTrack;
}

void ArmControlRight::setForearmYawPositionTrack(bool value)
{
    forearmYawPositionTrack = value;
}

bool ArmControlRight::getHandPitchPositionADC() const
{
    return handPitchPositionADC;
}

void ArmControlRight::setHandPitchPositionADC(bool value)
{
    handPitchPositionADC = value;
}

bool ArmControlRight::getHandPitchPositionTrack() const
{
    return handPitchPositionTrack;
}

void ArmControlRight::setHandPitchPositionTrack(bool value)
{
    handPitchPositionTrack = value;
}

bool ArmControlRight::getHandYawPositionADC() const
{
    return handYawPositionADC;
}

void ArmControlRight::setHandYawPositionADC(bool value)
{
    handYawPositionADC = value;
}

bool ArmControlRight::getHandYawPositionTrack() const
{
    return handYawPositionTrack;
}

void ArmControlRight::setHandYawPositionTrack(bool value)
{
    handYawPositionTrack = value;
}

bool ArmControlRight::getLimbMotorCurrentADC() const
{
    return limbMotorCurrentADC;
}

void ArmControlRight::setLimbMotorCurrentADC(bool value)
{
    limbMotorCurrentADC = value;
}

bool ArmControlRight::getLimbMotorCurrentTrack() const
{
    return limbMotorCurrentTrack;
}

void ArmControlRight::setLimbMotorCurrentTrack(bool value)
{
    limbMotorCurrentTrack = value;
}

bool ArmControlRight::getArmMotorCurrentADC() const
{
    return armMotorCurrentADC;
}

void ArmControlRight::setArmMotorCurrentADC(bool value)
{
    armMotorCurrentADC = value;
}

bool ArmControlRight::getArmMotorCurrentTrack() const
{
    return armMotorCurrentTrack;
}

void ArmControlRight::setArmMotorCurrentTrack(bool value)
{
    armMotorCurrentTrack = value;
}

bool ArmControlRight::getForearmMotorCurrentADC() const
{
    return forearmMotorCurrentADC;
}

void ArmControlRight::setForearmMotorCurrentADC(bool value)
{
    forearmMotorCurrentADC = value;
}

bool ArmControlRight::getForearmMotorCurrentTrack() const
{
    return forearmMotorCurrentTrack;
}

void ArmControlRight::setForearmMotorCurrentTrack(bool value)
{
    forearmMotorCurrentTrack = value;
}

bool ArmControlRight::getLimbPositionADC() const
{
    return limbPositionADC;
}

void ArmControlRight::setLimbPositionADC(bool value)
{
    limbPositionADC = value;
}

bool ArmControlRight::getLimbPositionTrack() const
{
    return limbPositionTrack;
}

void ArmControlRight::setLimbPositionTrack(bool value)
{
    limbPositionTrack = value;
}

bool ArmControlRight::getArmPositionADC() const
{
    return armPositionADC;
}

void ArmControlRight::setArmPositionADC(bool value)
{
    armPositionADC = value;
}

bool ArmControlRight::getArmPositionTrack() const
{
    return armPositionTrack;
}

void ArmControlRight::setArmPositionTrack(bool value)
{
    armPositionTrack = value;
}

bool ArmControlRight::getForearmPositionADC() const
{
    return forearmPositionADC;
}

void ArmControlRight::setForearmPositionADC(bool value)
{
    forearmPositionADC = value;
}

bool ArmControlRight::getForearmPositionTrack() const
{
    return forearmPositionTrack;
}

void ArmControlRight::setForearmPositionTrack(bool value)
{
    forearmPositionTrack = value;
}

bool ArmControlRight::getForearmYawDirection() const
{
    return forearmYawDirection;
}

bool ArmControlRight::getForearmRollCCWLimit() const
{
    return forearmRollCCWLimit;
}

void ArmControlRight::setForearmRollCCWLimit(bool value)
{
    forearmRollCCWLimit = value;
}

bool ArmControlRight::getForearmRollCWLimit() const
{
    return forearmRollCWLimit;
}

void ArmControlRight::setForearmRollCWLimit(bool value)
{
    forearmRollCWLimit = value;
}

int ArmControlRight::getForearmRollStepPosition() const
{
    return forearmRollStepPosition;
}

void ArmControlRight::setForearmRollStepPosition(int value)
{
    forearmRollStepPosition = value;
}

int ArmControlRight::getForearmRollStepSwitchDelay() const
{
    return forearmRollStepSwitchDelay;
}

void ArmControlRight::setForearmRollStepSwitchDelay(int value)
{
    forearmRollStepSwitchDelay = value;
}

int ArmControlRight::getForearmRollStepDelay() const
{
    return forearmRollStepDelay;
}

void ArmControlRight::setForearmRollStepDelay(int value)
{
    forearmRollStepDelay = value;
}

bool ArmControlRight::getForearmRollMotorActivated() const
{
    return forearmRollMotorActivated;
}

void ArmControlRight::setForearmRollMotorActivated(bool value)
{
    forearmRollMotorActivated = value;
}

bool ArmControlRight::getForearmRollMotorState() const
{
    return forearmRollMotorState;
}

void ArmControlRight::setForearmRollMotorState(bool value)
{
    forearmRollMotorState = value;
}

bool ArmControlRight::getForearmRollDirection() const
{
    return forearmRollDirection;
}

int ArmControlRight::getRightArmMotorDutyPresetMax() const
{
    return rightArmMotorDutyPresetMax;
}

void ArmControlRight::setRightArmMotorDutyPresetMax(int value)
{
    rightArmMotorDutyPresetMax = value;
}

int ArmControlRight::getRightArmMotorDutyPresetMin() const
{
    return rightArmMotorDutyPresetMin;
}

void ArmControlRight::setRightArmMotorDutyPresetMin(int value)
{
    rightArmMotorDutyPresetMin = value;
}

int ArmControlRight::getRightArmMotorDutyPresetCur() const
{
    return rightArmMotorDutyPresetCur;
}

void ArmControlRight::setRightArmMotorDutyPresetCur(int value)
{
    rightArmMotorDutyPresetCur = value;
}

int ArmControlRight::getRightArmADCCurrent() const
{
    return rightArmADCCurrent;
}

void ArmControlRight::setRightArmADCCurrent(int value)
{
    rightArmADCCurrent = value;
}

int ArmControlRight::getRightArmADCPosition() const
{
    return rightArmADCPosition;
}

void ArmControlRight::setRightArmADCPosition(int value)
{
    rightArmADCPosition = value;
}

bool ArmControlRight::getRightArmMotorStop() const
{
    return rightArmMotorStop;
}

void ArmControlRight::setRightArmMotorStop(bool value)
{
    rightArmMotorStop = value;
}

bool ArmControlRight::getRightArmMotorActivated() const
{
    return rightArmMotorActivated;
}

bool ArmControlRight::getRightArmMotorDecelerating() const
{
    return rightArmMotorDecelerating;
}

void ArmControlRight::setRightArmMotorDecelerating(bool value)
{
    rightArmMotorDecelerating = value;
}

bool ArmControlRight::getRightArmMotorAccelerating() const
{
    return rightArmMotorAccelerating;
}

void ArmControlRight::setRightArmMotorAccelerating(bool value)
{
    rightArmMotorAccelerating = value;
}

int ArmControlRight::getRightArmMotorDeceleration() const
{
    return rightArmMotorDeceleration;
}

void ArmControlRight::setRightArmMotorDeceleration(int value)
{
    rightArmMotorDeceleration = value;
}

int ArmControlRight::getRightArmMotorAcceleration() const
{
    return rightArmMotorAcceleration;
}

void ArmControlRight::setRightArmMotorAcceleration(int value)
{
    rightArmMotorAcceleration = value;
}

int ArmControlRight::getRightArmMotorDutyMax() const
{
    return rightArmMotorDutyMax;
}

void ArmControlRight::setRightArmMotorDutyMax(int value)
{
    rightArmMotorDutyMax = value;
}

int ArmControlRight::getRightArmMotorDuty() const
{
    return rightArmMotorDuty;
}

void ArmControlRight::setRightArmMotorDuty(int value)
{
    rightArmMotorDuty = value;
    sendCommand(Valter::format_string("SETARMLIFTUPDUTY#%d", rightArmMotorDuty));
}

int ArmControlRight::getRightForearmMotorDutyPresetMax() const
{
    return rightForearmMotorDutyPresetMax;
}

void ArmControlRight::setRightForearmMotorDutyPresetMax(int value)
{
    rightForearmMotorDutyPresetMax = value;
}

int ArmControlRight::getRightForearmMotorDutyPresetMin() const
{
    return rightForearmMotorDutyPresetMin;
}

void ArmControlRight::setRightForearmMotorDutyPresetMin(int value)
{
    rightForearmMotorDutyPresetMin = value;
}

int ArmControlRight::getRightForearmMotorDutyPresetCur() const
{
    return rightForearmMotorDutyPresetCur;
}

void ArmControlRight::setRightForearmMotorDutyPresetCur(int value)
{
    rightForearmMotorDutyPresetCur = value;
}

int ArmControlRight::getRightForearmADCCurrent() const
{
    return rightForearmADCCurrent;
}

void ArmControlRight::setRightForearmADCCurrent(int value)
{
    rightForearmADCCurrent = value;
}

int ArmControlRight::getRightForearmADCPosition() const
{
    return rightForearmADCPosition;
}

void ArmControlRight::setRightForearmADCPosition(int value)
{
    rightForearmADCPosition = value;
}

bool ArmControlRight::getRightForearmMotorStop() const
{
    return rightForearmMotorStop;
}

void ArmControlRight::setRightForearmMotorStop(bool value)
{
    rightForearmMotorStop = value;
}

bool ArmControlRight::getRightForearmMotorActivated() const
{
    return rightForearmMotorActivated;
}

bool ArmControlRight::getRightForearmMotorDecelerating() const
{
    return rightForearmMotorDecelerating;
}

void ArmControlRight::setRightForearmMotorDecelerating(bool value)
{
    rightForearmMotorDecelerating = value;
}

bool ArmControlRight::getRightForearmMotorAccelerating() const
{
    return rightForearmMotorAccelerating;
}

void ArmControlRight::setRightForearmMotorAccelerating(bool value)
{
    rightForearmMotorAccelerating = value;
}

int ArmControlRight::getRightForearmMotorDeceleration() const
{
    return rightForearmMotorDeceleration;
}

void ArmControlRight::setRightForearmMotorDeceleration(int value)
{
    rightForearmMotorDeceleration = value;
}

int ArmControlRight::getRightForearmMotorAcceleration() const
{
    return rightForearmMotorAcceleration;
}

void ArmControlRight::setRightForearmMotorAcceleration(int value)
{
    rightForearmMotorAcceleration = value;
}

int ArmControlRight::getRightForearmMotorDutyMax() const
{
    return rightForearmMotorDutyMax;
}

void ArmControlRight::setRightForearmMotorDutyMax(int value)
{
    rightForearmMotorDutyMax = value;
}

int ArmControlRight::getRightForearmMotorDuty() const
{
    return rightForearmMotorDuty;
}

bool ArmControlRight::getDefaultsLoading() const
{
    return defaultsLoading;
}

void ArmControlRight::setDefaultsLoading(bool value)
{
    defaultsLoading = value;
}
unsigned int ArmControlRight::getPalmJambReading() const
{
    return palmJambReading;
}

void ArmControlRight::setPalmJambReading(unsigned int value)
{
    palmJambReading = value;
    float relationToInitial = ((double)palmJambReading - (double)getPalmJambThreshold()) / ((double)1023 - (double)getPalmJambThreshold());
    setPalmJambReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getPalmLowerReading() const
{
    return palmLowerReading;
}

void ArmControlRight::setPalmLowerReading(unsigned int value)
{
    palmLowerReading = value;
    float relationToInitial = ((double)palmLowerReading - (double)getPalmLowerThreshold()) / ((double)1023 - (double)getPalmLowerThreshold());
    setPalmLowerReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getPalmUpperReading() const
{
    return palmUpperReading;
}

void ArmControlRight::setPalmUpperReading(unsigned int value)
{
    palmUpperReading = value;
    float relationToInitial = ((double)palmUpperReading - (double)getPalmUpperThreshold()) / ((double)1023 - (double)getPalmUpperThreshold());
    setPalmUpperReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger5TipReading() const
{
    return finger5TipReading;
}

void ArmControlRight::setFinger5TipReading(unsigned int value)
{
    finger5TipReading = value;
    float relationToInitial = ((double)finger5TipReading - (double)getFinger5TipThreshold()) / ((double)1023 - (double)getFinger5TipThreshold());
    setFinger5TipReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger5PhalanxReading() const
{
    return finger5PhalanxReading;
}

void ArmControlRight::setFinger5PhalanxReading(unsigned int value)
{
    finger5PhalanxReading = value;
    float relationToInitial = ((double)finger5PhalanxReading - (double)getFinger5PhalanxThreshold()) / ((double)1023 - (double)getFinger5PhalanxThreshold());
    setFinger5PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger4PhalanxReading() const
{
    return finger4PhalanxReading;
}

void ArmControlRight::setFinger4PhalanxReading(unsigned int value)
{
    finger4PhalanxReading = value;
    float relationToInitial = ((double)finger4PhalanxReading - (double)getFinger4PhalanxThreshold()) / ((double)1023 - (double)getFinger4PhalanxThreshold());
    setFinger4PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger4TipReading() const
{
    return finger4TipReading;
}

void ArmControlRight::setFinger4TipReading(unsigned int value)
{
    finger4TipReading = value;
    float relationToInitial = ((double)finger4TipReading - (double)getFinger4TipThreshold()) / ((double)1023 - (double)getFinger4TipThreshold());
    setFinger4TipReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger3PhalanxReading() const
{
    return finger3PhalanxReading;
}

void ArmControlRight::setFinger3PhalanxReading(unsigned int value)
{
    finger3PhalanxReading = value;
    float relationToInitial = ((double)finger3PhalanxReading - (double)getFinger3PhalanxThreshold()) / ((double)1023 - (double)getFinger3PhalanxThreshold());
    setFinger3PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger2PhalanxReading() const
{
    return finger2PhalanxReading;
}

void ArmControlRight::setFinger2PhalanxReading(unsigned int value)
{
    finger2PhalanxReading = value;
    float relationToInitial = ((double)finger2PhalanxReading - (double)getFinger2PhalanxThreshold()) / ((double)1023 - (double)getFinger2PhalanxThreshold());
    setFinger2PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger1TipReading() const
{
    return finger1TipReading;
}

void ArmControlRight::setFinger1TipReading(unsigned int value)
{
    finger1TipReading = value;
    float relationToInitial = ((double)finger1TipReading - (double)getFinger1TipThreshold()) / ((double)1023 - (double)getFinger1TipThreshold());
    setFinger1TipReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger1PhalanxReading() const
{
    return finger1PhalanxReading;
}

void ArmControlRight::setFinger1PhalanxReading(unsigned int value)
{
    finger1PhalanxReading = value;
    float relationToInitial = ((double)finger1PhalanxReading - (double)getFinger1PhalanxThreshold()) / ((double)1023 - (double)getFinger1PhalanxThreshold());
    setFinger1PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger0PhalanxReading() const
{
    return finger0PhalanxReading;
}

void ArmControlRight::setFinger0PhalanxReading(unsigned int value)
{
    finger0PhalanxReading = value;
    float relationToInitial = ((double)finger0PhalanxReading - (double)getFinger0PhalanxThreshold()) / ((double)1023 - (double)getFinger0PhalanxThreshold());
    setFinger0PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getFinger0TipReading() const
{
    return finger0TipReading;
}

void ArmControlRight::setFinger0TipReading(unsigned int value)
{
    finger0TipReading = value;
    float relationToInitial = ((double)finger0TipReading - (double)getFinger0TipThreshold()) / ((double)1023 - (double)getFinger0TipThreshold());
    setFinger0TipReadingRelative(relationToInitial);
}

unsigned int ArmControlRight::getPalmJambThreshold() const
{
    return palmJambThreshold;
}

void ArmControlRight::setPalmJambThreshold(unsigned int value)
{
    palmJambThreshold = value;
}

unsigned int ArmControlRight::getPalmLowerThreshold() const
{
    return palmLowerThreshold;
}

void ArmControlRight::setPalmLowerThreshold(unsigned int value)
{
    palmLowerThreshold = value;
}

unsigned int ArmControlRight::getPalmUpperThreshold() const
{
    return palmUpperThreshold;
}

void ArmControlRight::setPalmUpperThreshold(unsigned int value)
{
    palmUpperThreshold = value;
}

unsigned int ArmControlRight::getFinger5TipThreshold() const
{
    return finger5TipThreshold;
}

void ArmControlRight::setFinger5TipThreshold(unsigned int value)
{
    finger5TipThreshold = value;
}

unsigned int ArmControlRight::getFinger5PhalanxThreshold() const
{
    return finger5PhalanxThreshold;
}

void ArmControlRight::setFinger5PhalanxThreshold(unsigned int value)
{
    finger5PhalanxThreshold = value;
}

unsigned int ArmControlRight::getFinger4PhalanxThreshold() const
{
    return finger4PhalanxThreshold;
}

void ArmControlRight::setFinger4PhalanxThreshold(unsigned int value)
{
    finger4PhalanxThreshold = value;
}

unsigned int ArmControlRight::getFinger4TipThreshold() const
{
    return finger4TipThreshold;
}

void ArmControlRight::setFinger4TipThreshold(unsigned int value)
{
    finger4TipThreshold = value;
}

unsigned int ArmControlRight::getFinger3PhalanxThreshold() const
{
    return finger3PhalanxThreshold;
}

void ArmControlRight::setFinger3PhalanxThreshold(unsigned int value)
{
    finger3PhalanxThreshold = value;
}

unsigned int ArmControlRight::getFinger2PhalanxThreshold() const
{
    return finger2PhalanxThreshold;
}

void ArmControlRight::setFinger2PhalanxThreshold(unsigned int value)
{
    finger2PhalanxThreshold = value;
}

unsigned int ArmControlRight::getFinger1TipThreshold() const
{
    return finger1TipThreshold;
}

void ArmControlRight::setFinger1TipThreshold(unsigned int value)
{
    finger1TipThreshold = value;
}

unsigned int ArmControlRight::getFinger1PhalanxThreshold() const
{
    return finger1PhalanxThreshold;
}

void ArmControlRight::setFinger1PhalanxThreshold(unsigned int value)
{
    finger1PhalanxThreshold = value;
}

unsigned int ArmControlRight::getFinger0PhalanxThreshold() const
{
    return finger0PhalanxThreshold;
}

void ArmControlRight::setFinger0PhalanxThreshold(unsigned int value)
{
    finger0PhalanxThreshold = value;
}

unsigned int ArmControlRight::getFinger0TipThreshold() const
{
    return finger0TipThreshold;
}

void ArmControlRight::setFinger0TipThreshold(unsigned int value)
{
    finger0TipThreshold = value;
}
float ArmControlRight::getPalmJambReadingRelative()
{
    if (palmJambReadingRelative > 1)
        palmJambReadingRelative = 1;
    if (palmJambReadingRelative < 0)
        palmJambReadingRelative = 0;
    return palmJambReadingRelative;
}

void ArmControlRight::setPalmJambReadingRelative(float value)
{
    palmJambReadingRelative = value;
}

float ArmControlRight::getPalmLowerReadingRelative()
{
    if (palmLowerReadingRelative > 1)
        palmLowerReadingRelative = 1;
    if (palmLowerReadingRelative < 0)
        palmLowerReadingRelative = 0;
    return palmLowerReadingRelative;
}

void ArmControlRight::setPalmLowerReadingRelative(float value)
{
    palmLowerReadingRelative = value;
}

float ArmControlRight::getPalmUpperReadingRelative()
{
    if (palmUpperReadingRelative > 1)
        palmUpperReadingRelative = 1;
    if (palmUpperReadingRelative < 0)
        palmUpperReadingRelative = 0;
    return palmUpperReadingRelative;
}

void ArmControlRight::setPalmUpperReadingRelative(float value)
{
    palmUpperReadingRelative = value;
}

float ArmControlRight::getFinger5TipReadingRelative()
{
    if (finger5TipReadingRelative > 1)
        finger5TipReadingRelative = 1;
    if (finger5TipReadingRelative < 0)
        finger5TipReadingRelative = 0;
    return finger5TipReadingRelative;
}

void ArmControlRight::setFinger5TipReadingRelative(float value)
{
    finger5TipReadingRelative = value;
}

float ArmControlRight::getFinger5PhalanxReadingRelative()
{
    if (finger5PhalanxReadingRelative > 1)
        finger5PhalanxReadingRelative = 1;
    if (finger5PhalanxReadingRelative < 0)
        finger5PhalanxReadingRelative = 0;
    return finger5PhalanxReadingRelative;
}

void ArmControlRight::setFinger5PhalanxReadingRelative(float value)
{
    finger5PhalanxReadingRelative = value;
}

float ArmControlRight::getFinger4PhalanxReadingRelative()
{
    if (finger4PhalanxReadingRelative > 1)
        finger4PhalanxReadingRelative = 1;
    if (finger4PhalanxReadingRelative < 0)
        finger4PhalanxReadingRelative = 0;
    return finger4PhalanxReadingRelative;
}

void ArmControlRight::setFinger4PhalanxReadingRelative(float value)
{
    finger4PhalanxReadingRelative = value;
}

float ArmControlRight::getFinger4TipReadingRelative()
{
    if (finger4TipReadingRelative > 1)
        finger4TipReadingRelative = 1;
    if (finger4TipReadingRelative < 0)
        finger4TipReadingRelative = 0;
    return finger4TipReadingRelative;
}

void ArmControlRight::setFinger4TipReadingRelative(float value)
{
    finger4TipReadingRelative = value;
}

float ArmControlRight::getFinger3PhalanxReadingRelative()
{
    if (finger3PhalanxReadingRelative > 1)
        finger3PhalanxReadingRelative = 1;
    if (finger3PhalanxReadingRelative < 0)
        finger3PhalanxReadingRelative = 0;
    return finger3PhalanxReadingRelative;
}

void ArmControlRight::setFinger3PhalanxReadingRelative(float value)
{
    finger3PhalanxReadingRelative = value;
}

float ArmControlRight::getFinger2PhalanxReadingRelative()
{
    if (finger2PhalanxReadingRelative > 1)
        finger2PhalanxReadingRelative = 1;
    if (finger2PhalanxReadingRelative < 0)
        finger2PhalanxReadingRelative = 0;
    return finger2PhalanxReadingRelative;
}

void ArmControlRight::setFinger2PhalanxReadingRelative(float value)
{
    finger2PhalanxReadingRelative = value;
}

float ArmControlRight::getFinger1TipReadingRelative()
{
    if (finger1TipReadingRelative > 1)
        finger1TipReadingRelative = 1;
    if (finger1TipReadingRelative < 0)
        finger1TipReadingRelative = 0;
    return finger1TipReadingRelative;
}

void ArmControlRight::setFinger1TipReadingRelative(float value)
{
    finger1TipReadingRelative = value;
}

float ArmControlRight::getFinger1PhalanxReadingRelative()
{
    if (finger1PhalanxReadingRelative > 1)
        finger1PhalanxReadingRelative = 1;
    if (finger1PhalanxReadingRelative < 0)
        finger1PhalanxReadingRelative = 0;
    return finger1PhalanxReadingRelative;
}

void ArmControlRight::setFinger1PhalanxReadingRelative(float value)
{
    finger1PhalanxReadingRelative = value;
}

float ArmControlRight::getFinger0PhalanxReadingRelative()
{
    if (finger0PhalanxReadingRelative > 1)
        finger0PhalanxReadingRelative = 1;
    if (finger0PhalanxReadingRelative < 0)
        finger0PhalanxReadingRelative = 0;
    return finger0PhalanxReadingRelative;
}

void ArmControlRight::setFinger0PhalanxReadingRelative(float value)
{
    finger0PhalanxReadingRelative = value;
}

float ArmControlRight::getFinger0TipReadingRelative()
{
    if (finger0TipReadingRelative > 1)
        finger0TipReadingRelative = 1;
    if (finger0TipReadingRelative < 0)
        finger0TipReadingRelative = 0;
    return finger0TipReadingRelative;
}

void ArmControlRight::setFinger0TipReadingRelative(float value)
{
    finger0TipReadingRelative = value;
}
