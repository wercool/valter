#include "valter.h"

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

    //left arm readings presets
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

    //left arm readings
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
    for (unsigned int i = 0; i < 13; i++)
    {
        handSensorsTrack[i]  = false;
        handSensorsADCForce[i] = 0;
    }
}

void ArmControlLeft::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + ArmControlLeft::defaultsFilePath);
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

    //finger6InitialGrasped
    defaultValue = getDefault("finger6InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[0] = val1;
    fingerGraspedPositions[0] = val2;

    //finger7InitialGrasped
    defaultValue = getDefault("finger7InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[1] = val1;
    fingerGraspedPositions[1] = val2;

    //finger8InitialGrasped
    defaultValue = getDefault("finger8InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[2] = val1;
    fingerGraspedPositions[2] = val2;

    //finger9InitialGrasped
    defaultValue = getDefault("finger9InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[3] = val1;
    fingerGraspedPositions[3] = val2;

    //finger10InitialGrasped
    defaultValue = getDefault("finger10InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[4] = val1;
    fingerGraspedPositions[4] = val2;

    //finger11InitialGrasped
    defaultValue = getDefault("finger11InitialGrasped");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    fingerInitialPositions[5] = val1;
    fingerGraspedPositions[5] = val2;
}

int ArmControlLeft::getLeftLimbMotorDutyPresetMax() const
{
    return leftLimbMotorDutyPresetMax;
}

void ArmControlLeft::setLeftLimbMotorDutyPresetMax(int value)
{
    leftLimbMotorDutyPresetMax = value;
}

int ArmControlLeft::getForearmRollStepDelay() const
{
    return forearmRollStepDelay;
}

void ArmControlLeft::setForearmRollStepDelay(int value)
{
    forearmRollStepDelay = value;
}

int ArmControlLeft::getForearmRollStepSwitchDelay() const
{
    return forearmRollStepSwitchDelay;
}

void ArmControlLeft::setForearmRollStepSwitchDelay(int value)
{
    forearmRollStepSwitchDelay = value;
}

bool ArmControlLeft::getForearmRollMotorActivated() const
{
    return forearmRollMotorActivated;
}

void ArmControlLeft::setForearmRollMotorActivated(bool value)
{
    forearmRollMotorActivated = value;
}

bool ArmControlLeft::getForearmRollMotorState() const
{
    return forearmRollMotorState;
}

void ArmControlLeft::setForearmRollMotorState(bool value)
{
    forearmRollMotorState = value;
}

bool ArmControlLeft::getForearmRollDirection() const
{
    return forearmRollDirection;
}

bool ArmControlLeft::getHandPitchDirection() const
{
    return handPitchDirection;
}
int ArmControlLeft::getHandPitchMotorADCCurrent() const
{
    return handPitchMotorADCCurrent;
}

void ArmControlLeft::setHandPitchMotorADCCurrent(int value)
{
    handPitchMotorADCCurrent = value;
}

void ArmControlLeft::setHandSensorsTrack(int idx, bool state)
{
    handSensorsTrack[idx] = state;
}

bool ArmControlLeft::getHandSensorsTrack(int idx)
{
    return handSensorsTrack[idx];
}

void ArmControlLeft::setHandSensorsADCForce(int idx, int value)
{
    handSensorsADCForce[idx] = value;
}

int ArmControlLeft::getHandSensorsADCForce(int idx)
{
    return handSensorsADCForce[idx];
}

int ArmControlLeft::getHandYawMotorADCCurrent() const
{
    return handYawMotorADCCurrent;
}

void ArmControlLeft::setHandYawMotorADCCurrent(int value)
{
    handYawMotorADCCurrent = value;
}

int ArmControlLeft::getForearmYawMotorADCCurrent() const
{
    return forearmYawMotorADCCurrent;
}

void ArmControlLeft::setForearmYawMotorADCCurrent(int value)
{
    forearmYawMotorADCCurrent = value;
}

int ArmControlLeft::getForearmYawADCPosition() const
{
    return forearmYawADCPosition;
}

void ArmControlLeft::setForearmYawADCPosition(int value)
{
    forearmYawADCPosition = value;
}

int ArmControlLeft::getHandPitchADCPosition() const
{
    return handPitchADCPosition;
}

void ArmControlLeft::setHandPitchADCPosition(int value)
{
    handPitchADCPosition = value;
}

int ArmControlLeft::getHandYawADCPosition() const
{
    return handYawADCPosition;
}

void ArmControlLeft::setHandYawADCPosition(int value)
{
    handYawADCPosition = value;
}

int ArmControlLeft::getLimbMotorADCCurrent() const
{
    return limbMotorADCCurrent;
}

void ArmControlLeft::setLimbMotorADCCurrent(int value)
{
    limbMotorADCCurrent = value;
}

int ArmControlLeft::getArmMotorADCCurrent() const
{
    return armMotorADCCurrent;
}

void ArmControlLeft::setArmMotorADCCurrent(int value)
{
    armMotorADCCurrent = value;
}

int ArmControlLeft::getForearmMotorADCCurrent() const
{
    return forearmMotorADCCurrent;
}

void ArmControlLeft::setForearmMotorADCCurrent(int value)
{
    forearmMotorADCCurrent = value;
}

int ArmControlLeft::getLimbADCPosition() const
{
    return limbADCPosition;
}

void ArmControlLeft::setLimbADCPosition(int value)
{
    limbADCPosition = value;
}

int ArmControlLeft::getArmADCPosition() const
{
    return armADCPosition;
}

void ArmControlLeft::setArmADCPosition(int value)
{
    armADCPosition = value;
}

int ArmControlLeft::getForearmADCPosition() const
{
    return forearmADCPosition;
}

void ArmControlLeft::setForearmADCPosition(int value)
{
    forearmADCPosition = value;
}

bool ArmControlLeft::getHandPitchMotorCurrentADC() const
{
    return handPitchMotorCurrentADC;
}

void ArmControlLeft::setHandPitchMotorCurrentADC(bool value)
{
    handPitchMotorCurrentADC = value;
}

bool ArmControlLeft::getHandPitchMotorCurrentTrack() const
{
    return handPitchMotorCurrentTrack;
}

void ArmControlLeft::setHandPitchMotorCurrentTrack(bool value)
{
    handPitchMotorCurrentTrack = value;
}

bool ArmControlLeft::getHandYawMotorCurrentADC() const
{
    return handYawMotorCurrentADC;
}

void ArmControlLeft::setHandYawMotorCurrentADC(bool value)
{
    handYawMotorCurrentADC = value;
}

bool ArmControlLeft::getHandYawMotorCurrentTrack() const
{
    return handYawMotorCurrentTrack;
}

void ArmControlLeft::setHandYawMotorCurrentTrack(bool value)
{
    handYawMotorCurrentTrack = value;
}

bool ArmControlLeft::getForearmYawMotorCurrentADC() const
{
    return forearmYawMotorCurrentADC;
}

void ArmControlLeft::setForearmYawMotorCurrentADC(bool value)
{
    forearmYawMotorCurrentADC = value;
}

bool ArmControlLeft::getForearmYawMotorCurrentTrack() const
{
    return forearmYawMotorCurrentTrack;
}

void ArmControlLeft::setForearmYawMotorCurrentTrack(bool value)
{
    forearmYawMotorCurrentTrack = value;
}

bool ArmControlLeft::getForearmYawPositionADC() const
{
    return forearmYawPositionADC;
}

void ArmControlLeft::setForearmYawPositionADC(bool value)
{
    forearmYawPositionADC = value;
}

bool ArmControlLeft::getForearmYawPositionTrack() const
{
    return forearmYawPositionTrack;
}

void ArmControlLeft::setForearmYawPositionTrack(bool value)
{
    forearmYawPositionTrack = value;
}

bool ArmControlLeft::getHandPitchPositionADC() const
{
    return handPitchPositionADC;
}

void ArmControlLeft::setHandPitchPositionADC(bool value)
{
    handPitchPositionADC = value;
}

bool ArmControlLeft::getHandPitchPositionTrack() const
{
    return handPitchPositionTrack;
}

void ArmControlLeft::setHandPitchPositionTrack(bool value)
{
    handPitchPositionTrack = value;
}

bool ArmControlLeft::getHandYawPositionADC() const
{
    return handYawPositionADC;
}

void ArmControlLeft::setHandYawPositionADC(bool value)
{
    handYawPositionADC = value;
}

bool ArmControlLeft::getHandYawPositionTrack() const
{
    return handYawPositionTrack;
}

void ArmControlLeft::setHandYawPositionTrack(bool value)
{
    handYawPositionTrack = value;
}

bool ArmControlLeft::getLimbMotorCurrentADC() const
{
    return limbMotorCurrentADC;
}

void ArmControlLeft::setLimbMotorCurrentADC(bool value)
{
    limbMotorCurrentADC = value;
}

bool ArmControlLeft::getLimbMotorCurrentTrack() const
{
    return limbMotorCurrentTrack;
}

void ArmControlLeft::setLimbMotorCurrentTrack(bool value)
{
    limbMotorCurrentTrack = value;
}

bool ArmControlLeft::getArmMotorCurrentADC() const
{
    return armMotorCurrentADC;
}

void ArmControlLeft::setArmMotorCurrentADC(bool value)
{
    armMotorCurrentADC = value;
}

bool ArmControlLeft::getArmMotorCurrentTrack() const
{
    return armMotorCurrentTrack;
}

void ArmControlLeft::setArmMotorCurrentTrack(bool value)
{
    armMotorCurrentTrack = value;
}

bool ArmControlLeft::getForearmMotorCurrentADC() const
{
    return forearmMotorCurrentADC;
}

void ArmControlLeft::setForearmMotorCurrentADC(bool value)
{
    forearmMotorCurrentADC = value;
}

bool ArmControlLeft::getForearmMotorCurrentTrack() const
{
    return forearmMotorCurrentTrack;
}

void ArmControlLeft::setForearmMotorCurrentTrack(bool value)
{
    forearmMotorCurrentTrack = value;
}

bool ArmControlLeft::getLimbPositionADC() const
{
    return limbPositionADC;
}

void ArmControlLeft::setLimbPositionADC(bool value)
{
    limbPositionADC = value;
}

bool ArmControlLeft::getLimbPositionTrack() const
{
    return limbPositionTrack;
}

void ArmControlLeft::setLimbPositionTrack(bool value)
{
    limbPositionTrack = value;
}

bool ArmControlLeft::getArmPositionADC() const
{
    return armPositionADC;
}

void ArmControlLeft::setArmPositionADC(bool value)
{
    armPositionADC = value;
}

bool ArmControlLeft::getArmPositionTrack() const
{
    return armPositionTrack;
}

void ArmControlLeft::setArmPositionTrack(bool value)
{
    armPositionTrack = value;
}

bool ArmControlLeft::getForearmPositionADC() const
{
    return forearmPositionADC;
}

void ArmControlLeft::setForearmPositionADC(bool value)
{
    forearmPositionADC = value;
}

bool ArmControlLeft::getForearmPositionTrack() const
{
    return forearmPositionTrack;
}

void ArmControlLeft::setForearmPositionTrack(bool value)
{
    forearmPositionTrack = value;
}

bool ArmControlLeft::getArmLedsState() const
{
    return armLedsState;
}

void ArmControlLeft::setArmLedsState(bool value)
{
    armLedsState = value;
}

bool ArmControlLeft::getForearmRollResettingStepPosition() const
{
    return forearmRollResettingStepPosition;
}
bool ArmControlLeft::getForearmRollCCWLimit() const
{
    return forearmRollCCWLimit;
}

void ArmControlLeft::setForearmRollCCWLimit(bool value)
{
    forearmRollCCWLimit = value;
}

bool ArmControlLeft::getForearmRollCWLimit() const
{
    return forearmRollCWLimit;
}

void ArmControlLeft::setForearmRollCWLimit(bool value)
{
    forearmRollCWLimit = value;
}

int ArmControlLeft::getForearmRollStepPosition() const
{
    return forearmRollStepPosition;
}

void ArmControlLeft::setForearmRollStepPosition(int value)
{
    forearmRollStepPosition = value;
}

bool ArmControlLeft::getHandYawDirection() const
{
    return handYawDirection;
}
int ArmControlLeft::getLeftLimbMotorDutyPresetMin() const
{
    return leftLimbMotorDutyPresetMin;
}

void ArmControlLeft::setLeftLimbMotorDutyPresetMin(int value)
{
    leftLimbMotorDutyPresetMin = value;
}

int ArmControlLeft::getLeftLimbMotorDutyPresetCur() const
{
    return leftLimbMotorDutyPresetCur;
}

void ArmControlLeft::setLeftLimbMotorDutyPresetCur(int value)
{
    leftLimbMotorDutyPresetCur = value;
}

int ArmControlLeft::getLeftLimbADCCurrent() const
{
    return leftLimbADCCurrent;
}

void ArmControlLeft::setLeftLimbADCCurrent(int value)
{
    leftLimbADCCurrent = value;
}

int ArmControlLeft::getLeftLimbADCPosition() const
{
    return leftLimbADCPosition;
}

void ArmControlLeft::setLeftLimbADCPosition(int value)
{
    leftLimbADCPosition = value;
}

bool ArmControlLeft::getLeftLimbMotorStop() const
{
    return leftLimbMotorStop;
}

void ArmControlLeft::setLeftLimbMotorStop(bool value)
{
    leftLimbMotorStop = value;
}

bool ArmControlLeft::getLeftLimbMotorActivated() const
{
    return leftLimbMotorActivated;
}
bool ArmControlLeft::getLeftLimbMotorDecelerating() const
{
    return leftLimbMotorDecelerating;
}

void ArmControlLeft::setLeftLimbMotorDecelerating(bool value)
{
    leftLimbMotorDecelerating = value;
}

bool ArmControlLeft::getLeftLimbMotorAccelerating() const
{
    return leftLimbMotorAccelerating;
}

void ArmControlLeft::setLeftLimbMotorAccelerating(bool value)
{
    leftLimbMotorAccelerating = value;
}

int ArmControlLeft::getLeftLimbMotorDeceleration() const
{
    return leftLimbMotorDeceleration;
}

void ArmControlLeft::setLeftLimbMotorDeceleration(int value)
{
    leftLimbMotorDeceleration = value;
}

int ArmControlLeft::getLeftLimbMotorAcceleration() const
{
    return leftLimbMotorAcceleration;
}

void ArmControlLeft::setLeftLimbMotorAcceleration(int value)
{
    leftLimbMotorAcceleration = value;
}

int ArmControlLeft::getLeftLimbMotorDutyMax() const
{
    return leftLimbMotorDutyMax;
}

void ArmControlLeft::setLeftLimbMotorDutyMax(int value)
{
    leftLimbMotorDutyMax = value;
}

int ArmControlLeft::getLeftLimbMotorDuty() const
{
    return leftLimbMotorDuty;
}
int ArmControlLeft::getLeftArmMotorDutyPresetMax() const
{
    return leftArmMotorDutyPresetMax;
}

void ArmControlLeft::setLeftArmMotorDutyPresetMax(int value)
{
    leftArmMotorDutyPresetMax = value;
}

int ArmControlLeft::getLeftArmMotorDutyPresetMin() const
{
    return leftArmMotorDutyPresetMin;
}

void ArmControlLeft::setLeftArmMotorDutyPresetMin(int value)
{
    leftArmMotorDutyPresetMin = value;
}

int ArmControlLeft::getLeftArmMotorDutyPresetCur() const
{
    return leftArmMotorDutyPresetCur;
}

void ArmControlLeft::setLeftArmMotorDutyPresetCur(int value)
{
    leftArmMotorDutyPresetCur = value;
}

int ArmControlLeft::getLeftArmADCCurrent() const
{
    return leftArmADCCurrent;
}

void ArmControlLeft::setLeftArmADCCurrent(int value)
{
    leftArmADCCurrent = value;
}

int ArmControlLeft::getLeftArmADCPosition() const
{
    return leftArmADCPosition;
}

void ArmControlLeft::setLeftArmADCPosition(int value)
{
    leftArmADCPosition = value;
}

bool ArmControlLeft::getLeftArmMotorStop() const
{
    return leftArmMotorStop;
}

void ArmControlLeft::setLeftArmMotorStop(bool value)
{
    leftArmMotorStop = value;
}

bool ArmControlLeft::getLeftArmMotorActivated() const
{
    return leftArmMotorActivated;
}
bool ArmControlLeft::getLeftArmMotorDecelerating() const
{
    return leftArmMotorDecelerating;
}

void ArmControlLeft::setLeftArmMotorDecelerating(bool value)
{
    leftArmMotorDecelerating = value;
}

bool ArmControlLeft::getLeftArmMotorAccelerating() const
{
    return leftArmMotorAccelerating;
}

void ArmControlLeft::setLeftArmMotorAccelerating(bool value)
{
    leftArmMotorAccelerating = value;
}

int ArmControlLeft::getLeftArmMotorDeceleration() const
{
    return leftArmMotorDeceleration;
}

void ArmControlLeft::setLeftArmMotorDeceleration(int value)
{
    leftArmMotorDeceleration = value;
}

int ArmControlLeft::getLeftArmMotorAcceleration() const
{
    return leftArmMotorAcceleration;
}

void ArmControlLeft::setLeftArmMotorAcceleration(int value)
{
    leftArmMotorAcceleration = value;
}

int ArmControlLeft::getLeftArmMotorDutyMax() const
{
    return leftArmMotorDutyMax;
}

void ArmControlLeft::setLeftArmMotorDutyMax(int value)
{
    leftArmMotorDutyMax = value;
}

int ArmControlLeft::getLeftArmMotorDuty() const
{
    return leftArmMotorDuty;
}
int ArmControlLeft::getLeftForearmMotorDutyPresetMax() const
{
    return leftForearmMotorDutyPresetMax;
}

void ArmControlLeft::setLeftForearmMotorDutyPresetMax(int value)
{
    leftForearmMotorDutyPresetMax = value;
}

int ArmControlLeft::getLeftForearmMotorDutyPresetMin() const
{
    return leftForearmMotorDutyPresetMin;
}

void ArmControlLeft::setLeftForearmMotorDutyPresetMin(int value)
{
    leftForearmMotorDutyPresetMin = value;
}

int ArmControlLeft::getLeftForearmMotorDutyPresetCur() const
{
    return leftForearmMotorDutyPresetCur;
}

void ArmControlLeft::setLeftForearmMotorDutyPresetCur(int value)
{
    leftForearmMotorDutyPresetCur = value;
}

int ArmControlLeft::getLeftForearmADCCurrent() const
{
    return leftForearmADCCurrent;
}

void ArmControlLeft::setLeftForearmADCCurrent(int value)
{
    leftForearmADCCurrent = value;
}

int ArmControlLeft::getLeftForearmADCPosition() const
{
    return leftForearmADCPosition;
}

void ArmControlLeft::setLeftForearmADCPosition(int value)
{
    leftForearmADCPosition = value;
}

bool ArmControlLeft::getLeftForearmMotorStop() const
{
    return leftForearmMotorStop;
}

void ArmControlLeft::setLeftForearmMotorStop(bool value)
{
    leftForearmMotorStop = value;
}

bool ArmControlLeft::getLeftForearmMotorActivated() const
{
    return leftForearmMotorActivated;
}
bool ArmControlLeft::getLeftForearmMotorDecelerating() const
{
    return leftForearmMotorDecelerating;
}

void ArmControlLeft::setLeftForearmMotorDecelerating(bool value)
{
    leftForearmMotorDecelerating = value;
}

bool ArmControlLeft::getLeftForearmMotorAccelerating() const
{
    return leftForearmMotorAccelerating;
}

void ArmControlLeft::setLeftForearmMotorAccelerating(bool value)
{
    leftForearmMotorAccelerating = value;
}

int ArmControlLeft::getLeftForearmMotorDeceleration() const
{
    return leftForearmMotorDeceleration;
}

void ArmControlLeft::setLeftForearmMotorDeceleration(int value)
{
    leftForearmMotorDeceleration = value;
}

int ArmControlLeft::getLeftForearmMotorAcceleration() const
{
    return leftForearmMotorAcceleration;
}

void ArmControlLeft::setLeftForearmMotorAcceleration(int value)
{
    leftForearmMotorAcceleration = value;
}

int ArmControlLeft::getLeftForearmMotorDutyMax() const
{
    return leftForearmMotorDutyMax;
}

void ArmControlLeft::setLeftForearmMotorDutyMax(int value)
{
    leftForearmMotorDutyMax = value;
}

int ArmControlLeft::getLeftForearmMotorDuty() const
{
    return leftForearmMotorDuty;
}

