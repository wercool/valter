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
    forearmRollStepDelay                = 6000;
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
    defaultsLoading = false;

    //fingers thresholds (finger sensors)
    finger6TipThreshold         = 832;
    finger6PhalanxThreshold     = 654;
    finger7PhalanxThreshold     = 394;
    finger7TipThreshold         = 401;
    finger8PhalanxThreshold     = 780;
    finger9PhalanxThreshold     = 410;
    finger10TipThreshold        = 637;
    finger10PhalanxThreshold    = 659;
    finger11TipThreshold        = 803;
    finger11PhalanxThreshold    = 870;
    palmUpperThreshold          = 793;
    palmLowerThreshold          = 793;
    palmJambThreshold           = 967;

    finger6TipReading       = 0;
    finger6PhalanxReading   = 0;
    finger7PhalanxReading   = 0;
    finger7TipReading       = 0;
    finger8PhalanxReading   = 0;
    finger9PhalanxReading   = 0;
    finger10TipReading      = 0;
    finger10PhalanxReading  = 0;
    finger11TipReading      = 0;
    finger11PhalanxReading  = 0;
    palmUpperReading        = 0;
    palmLowerReading        = 0;
    palmJambReading         = 0;

    finger6TipReadingRelative       = 0;
    finger6PhalanxReadingRelative   = 0;
    finger7PhalanxReadingRelative   = 0;
    finger7TipReadingRelative       = 0;
    finger8PhalanxReadingRelative   = 0;
    finger9PhalanxReadingRelative   = 0;
    finger10TipReadingRelative      = 0;
    finger10PhalanxReadingRelative  = 0;
    finger11TipReadingRelative      = 0;
    finger11PhalanxReadingRelative  = 0;
    palmUpperReadingRelative        = 0;
    palmLowerReadingRelative        = 0;
    palmJambReadingRelative         = 0;

    forearmPosition     = 0;
    armPosition         = 0;
    limbPosition        = 0;
}

void ArmControlLeft::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + ArmControlLeft::defaultsFilePath);
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

    //finger6Tip
    defaultValue = getDefault("finger6Tip");
    finger6TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger6Phalanx
    defaultValue = getDefault("finger6Phalanx");
    finger6PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger7Phalanx
    defaultValue = getDefault("finger7Phalanx");
    finger7PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger7Tip
    defaultValue = getDefault("finger7Tip");
    finger7TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger8Phalanx
    defaultValue = getDefault("finger8Phalanx");
    finger8PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger9Phalanx
    defaultValue = getDefault("finger9Phalanx");
    finger9PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger10Tip
    defaultValue = getDefault("finger10Tip");
    finger10TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger10Phalanx
    defaultValue = getDefault("finger10Phalanx");
    finger10PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger11Tip
    defaultValue = getDefault("finger11Tip");
    finger11TipThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //finger11Phalanx
    defaultValue = getDefault("finger11Phalanx");
    finger11PhalanxThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //leftPalmUpper
    defaultValue = getDefault("leftPalmUpper");
    palmUpperThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //leftPalmLower
    defaultValue = getDefault("leftPalmLower");
    palmLowerThreshold = atoi(Valter::stringToCharPtr(defaultValue));

    //leftPalmJamb
    defaultValue = getDefault("leftPalmJamb");
    palmJambThreshold = atoi(Valter::stringToCharPtr(defaultValue));
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
    static double prevValue = -180;
    static double avgValue = 0;
    static int avgValueCnt = 0;
    static int resetPositionCnt = 0;

    limbADCPosition = value;
    double degreesValue = ((double)(limbADCPosition - ArmControlLeft::limbAngleADCZero)) / ArmControlLeft::limbDegreesDiv;

    if (prevValue == -180)
    {
        prevValue = degreesValue;
    }

    if (abs(prevValue - degreesValue) < 10.0)
    {
        if (avgValueCnt < 5)
        {
            avgValue += degreesValue;
            avgValueCnt++;
        }
        else
        {
            double avgResult = avgValue / (double) avgValueCnt;
            //qDebug("Left Limb Position (avg) %.2f", avgResult);
            setLimbPosition(avgResult);
            avgValueCnt = 0;
            avgValue = 0;
            resetPositionCnt = 0;
        }
        prevValue = degreesValue;
    }
    else
    {
        resetPositionCnt++;
        if (resetPositionCnt > 10)
        {
            setLimbPosition(degreesValue);
            prevValue = degreesValue;
            avgValueCnt = 0;
            avgValue = 0;
            resetPositionCnt = 0;
            string msg = Valter::format_string("POSITION RESET ArmControlLeft::setLimbPosition(%.2f)", degreesValue);
            qDebug("%s", msg.c_str());
        }
//        string msg = Valter::format_string("IGNORE ArmControlLeft::setLimbADCPosition    [diff: %.2f]     prevValue = %.2f degreesValue = %.2f", abs(prevValue - degreesValue), prevValue, degreesValue);
//        qDebug("%s", msg.c_str());
    }

    /*
    limbADCPosition = value;
    double degreesValue = ((double)(limbADCPosition - ArmControlLeft::limbAngleADCZero)) / ArmControlLeft::limbDegreesDiv;
    setLimbPosition(degreesValue);
    */
}

double ArmControlLeft::getLimbPosition() const
{
    return limbPosition;
}

void ArmControlLeft::setLimbPosition(double value)
{
    limbPosition = value;
}

int ArmControlLeft::getArmADCPosition() const
{
    return armADCPosition;
}

void ArmControlLeft::setArmADCPosition(int value)
{
    static double prevValue = -180;
    static double avgValue = 0;
    static int avgValueCnt = 0;
    static int resetPositionCnt = 0;

    armADCPosition = value;
    double degreesValue = ((double)(ArmControlLeft::armAngleADCZero - armADCPosition)) / ArmControlLeft::armDegreesDiv;

    if (prevValue == -180)
    {
        prevValue = degreesValue;
    }

    if (abs(prevValue - degreesValue) < 10.0)
    {
        if (avgValueCnt < 5)
        {
            avgValue += degreesValue;
            avgValueCnt++;
        }
        else
        {
            double avgResult = avgValue / (double) avgValueCnt;
            //qDebug("Left Arm Position (avg) %.2f", avgResult);
            setArmPosition(avgResult);
            avgValueCnt = 0;
            avgValue = 0;
            resetPositionCnt = 0;
        }
        prevValue = degreesValue;
    }
    else
    {
        resetPositionCnt++;
        if (resetPositionCnt > 10)
        {
            setArmPosition(degreesValue);
            prevValue = degreesValue;
            avgValueCnt = 0;
            avgValue = 0;
            resetPositionCnt = 0;
            string msg = Valter::format_string("POSITION RESET ArmControlLeft::setArmPosition(%.2f)", degreesValue);
            qDebug("%s", msg.c_str());
        }
//        string msg = Valter::format_string("IGNORE ArmControlLeft::setArmADCPosition    [diff: %.2f]     prevValue = %.2f degreesValue = %.2f", abs(prevValue - degreesValue), prevValue, degreesValue);
//        qDebug("%s", msg.c_str());
    }
    /*
    armADCPosition = value;
    double degreesValue = ((double)(ArmControlLeft::armAngleADCZero - armADCPosition)) / ArmControlLeft::armDegreesDiv;
    setArmPosition(degreesValue);
    */
}

double ArmControlLeft::getArmPosition() const
{
    return armPosition;
}

void ArmControlLeft::setArmPosition(double value)
{
    armPosition = value;
}

int ArmControlLeft::getForearmADCPosition() const
{
    return forearmADCPosition;
}

void ArmControlLeft::setForearmADCPosition(int value)
{
    static double prevValue = -180;
    static double avgValue = 0;
    static int avgValueCnt = 0;
    static int resetPositionCnt = 0;

    forearmADCPosition = value;
    double degreesValue = ((double)(forearmADCPosition - ArmControlLeft::forearmAngleADCZero)) / ArmControlLeft::forearmDegreesDiv;

    if (prevValue == -180)
    {
        prevValue = degreesValue;
    }

    if (abs(prevValue - degreesValue) < 10.0)
    {
        if (avgValueCnt < 5)
        {
            avgValue += degreesValue;
            avgValueCnt++;
        }
        else
        {
            double avgResult = avgValue / (double) avgValueCnt;
//            qDebug("Left Forearm Position (avg) %.2f", avgResult);
            setForearmPosition(avgResult);
            avgValueCnt = 0;
            avgValue = 0;
            resetPositionCnt = 0;
        }
        prevValue = degreesValue;
    }
    else
    {
        resetPositionCnt++;
        if (resetPositionCnt > 10)
        {
            setForearmPosition(degreesValue);
            prevValue = degreesValue;
            avgValueCnt = 0;
            avgValue = 0;
            resetPositionCnt = 0;
            string msg = Valter::format_string("POSITION RESET ArmControlLeft::setForearmPosition(%.2f)", degreesValue);
            qDebug("%s", msg.c_str());
        }
//        string msg = Valter::format_string("IGNORE ArmControlLeft::setForearmADCPosition    [diff: %.2f]     prevValue = %.2f degreesValue = %.2f", abs(prevValue - degreesValue), prevValue, degreesValue);
//        qDebug("%s", msg.c_str());
    }

    /*
    forearmADCPosition = value;
    double degreesValue = ((double)(forearmADCPosition - ArmControlLeft::forearmAngleADCZero)) / ArmControlLeft::forearmDegreesDiv;
    setForearmPosition(degreesValue);
    */
}

double ArmControlLeft::getForearmPosition() const
{
    return forearmPosition;
}

void ArmControlLeft::setForearmPosition(double value)
{
    forearmPosition = value;
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

bool ArmControlLeft::getDefaultsLoading() const
{
    return defaultsLoading;
}

void ArmControlLeft::setDefaultsLoading(bool value)
{
    defaultsLoading = value;
}

float ArmControlLeft::getPalmJambReadingRelative()
{
    if (palmJambReadingRelative > 1)
        palmJambReadingRelative = 1;
    if (palmJambReadingRelative < 0)
        palmJambReadingRelative = 0;
    return palmJambReadingRelative;
}

void ArmControlLeft::setPalmJambReadingRelative(float value)
{
    palmJambReadingRelative = value;
}

float ArmControlLeft::getPalmLowerReadingRelative()
{
    if (palmLowerReadingRelative > 1)
        palmLowerReadingRelative = 1;
    if (palmLowerReadingRelative < 0)
        palmLowerReadingRelative = 0;
    return palmLowerReadingRelative;
}

void ArmControlLeft::setPalmLowerReadingRelative(float value)
{
    palmLowerReadingRelative = value;
}

float ArmControlLeft::getPalmUpperReadingRelative()
{
    if (palmUpperReadingRelative > 1)
        palmUpperReadingRelative = 1;
    if (palmUpperReadingRelative < 0)
        palmUpperReadingRelative = 0;
    return palmUpperReadingRelative;
}

void ArmControlLeft::setPalmUpperReadingRelative(float value)
{
    palmUpperReadingRelative = value;
}

float ArmControlLeft::getFinger11PhalanxReadingRelative()
{
    if (finger11PhalanxReadingRelative > 1)
        finger11PhalanxReadingRelative = 1;
    if (finger11PhalanxReadingRelative < 0)
        finger11PhalanxReadingRelative = 0;
    return finger11PhalanxReadingRelative;
}

void ArmControlLeft::setFinger11PhalanxReadingRelative(float value)
{
    finger11PhalanxReadingRelative = value;
}

float ArmControlLeft::getFinger11TipReadingRelative()
{
    if (finger11TipReadingRelative > 1)
        finger11TipReadingRelative = 1;
    if (finger11TipReadingRelative < 0)
        finger11TipReadingRelative = 0;
    return finger11TipReadingRelative;
}

void ArmControlLeft::setFinger11TipReadingRelative(float value)
{
    finger11TipReadingRelative = value;
}

float ArmControlLeft::getFinger10PhalanxReadingRelative()
{
    if (finger10PhalanxReadingRelative > 1)
        finger10PhalanxReadingRelative = 1;
    if (finger10PhalanxReadingRelative < 0)
        finger10PhalanxReadingRelative = 0;
    return finger10PhalanxReadingRelative;
}

void ArmControlLeft::setFinger10PhalanxReadingRelative(float value)
{
    finger10PhalanxReadingRelative = value;
}

float ArmControlLeft::getFinger10TipReadingRelative()
{
    if (finger10TipReadingRelative > 1)
        finger10TipReadingRelative = 1;
    if (finger10TipReadingRelative < 0)
        finger10TipReadingRelative = 0;
    return finger10TipReadingRelative;
}

void ArmControlLeft::setFinger10TipReadingRelative(float value)
{
    finger10TipReadingRelative = value;
}

float ArmControlLeft::getFinger9PhalanxReadingRelative()
{
    if (finger9PhalanxReadingRelative > 1)
        finger9PhalanxReadingRelative = 1;
    if (finger9PhalanxReadingRelative < 0)
        finger9PhalanxReadingRelative = 0;
    return finger9PhalanxReadingRelative;
}

void ArmControlLeft::setFinger9PhalanxReadingRelative(float value)
{
    finger9PhalanxReadingRelative = value;
}

float ArmControlLeft::getFinger8PhalanxReadingRelative()
{
    if (finger8PhalanxReadingRelative > 1)
        finger8PhalanxReadingRelative = 1;
    if (finger8PhalanxReadingRelative < 0)
        finger8PhalanxReadingRelative = 0;
    return finger8PhalanxReadingRelative;
}

void ArmControlLeft::setFinger8PhalanxReadingRelative(float value)
{
    finger8PhalanxReadingRelative = value;
}

float ArmControlLeft::getFinger7TipReadingRelative()
{
    if (finger7TipReadingRelative > 1)
        finger7TipReadingRelative = 1;
    if (finger7TipReadingRelative < 0)
        finger7TipReadingRelative = 0;
    return finger7TipReadingRelative;
}

void ArmControlLeft::setFinger7TipReadingRelative(float value)
{
    finger7TipReadingRelative = value;
}

float ArmControlLeft::getFinger7PhalanxReadingRelative()
{
    if (finger7PhalanxReadingRelative > 1)
        finger7PhalanxReadingRelative = 1;
    if (finger7PhalanxReadingRelative < 0)
        finger7PhalanxReadingRelative = 0;
    return finger7PhalanxReadingRelative;
}

void ArmControlLeft::setFinger7PhalanxReadingRelative(float value)
{
    finger7PhalanxReadingRelative = value;
}

float ArmControlLeft::getFinger6PhalanxReadingRelative()
{
    if (finger6PhalanxReadingRelative > 1)
        finger6PhalanxReadingRelative = 1;
    if (finger6PhalanxReadingRelative < 0)
        finger6PhalanxReadingRelative = 0;
    return finger6PhalanxReadingRelative;
}

void ArmControlLeft::setFinger6PhalanxReadingRelative(float value)
{
    finger6PhalanxReadingRelative = value;
}

float ArmControlLeft::getFinger6TipReadingRelative()
{
    if (finger6TipReadingRelative > 1)
        finger6TipReadingRelative = 1;
    if (finger6TipReadingRelative < 0)
        finger6TipReadingRelative = 0;
    return finger6TipReadingRelative;
}

void ArmControlLeft::setFinger6TipReadingRelative(float value)
{
    finger6TipReadingRelative = value;
}

unsigned int ArmControlLeft::getPalmJambReading() const
{
    return palmJambReading;
}

void ArmControlLeft::setPalmJambReading(unsigned int value)
{
    palmJambReading = value;
    float relationToInitial = ((double)palmJambReading - (double)getPalmJambThreshold()) / ((double)1023 - (double)getPalmJambThreshold());
    setPalmJambReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getPalmLowerReading() const
{
    return palmLowerReading;
}

void ArmControlLeft::setPalmLowerReading(unsigned int value)
{
    palmLowerReading = value;
    float relationToInitial = ((double)palmLowerReading - (double)getPalmLowerThreshold()) / ((double)1023 - (double)getPalmLowerThreshold());
    setPalmLowerReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getPalmUpperReading() const
{
    return palmUpperReading;
}

void ArmControlLeft::setPalmUpperReading(unsigned int value)
{
    palmUpperReading = value;
    float relationToInitial = ((double)palmUpperReading - (double)getPalmUpperThreshold()) / ((double)1023 - (double)getPalmUpperThreshold());
    setPalmUpperReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger11PhalanxReading() const
{
    return finger11PhalanxReading;
}

void ArmControlLeft::setFinger11PhalanxReading(unsigned int value)
{
    finger11PhalanxReading = value;
    float relationToInitial = ((double)finger11PhalanxReading - (double)getFinger11PhalanxThreshold()) / ((double)1023 - (double)getFinger11PhalanxThreshold());
    setFinger11PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger11TipReading() const
{
    return finger11TipReading;
}

void ArmControlLeft::setFinger11TipReading(unsigned int value)
{
    finger11TipReading = value;
    float relationToInitial = ((double)finger11TipReading - (double)getFinger11TipThreshold()) / ((double)1023 - (double)getFinger11TipThreshold());
    setFinger11TipReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger10PhalanxReading() const
{
    return finger10PhalanxReading;
}

void ArmControlLeft::setFinger10PhalanxReading(unsigned int value)
{
    finger10PhalanxReading = value;
    float relationToInitial = ((double)finger10PhalanxReading - (double)getFinger10PhalanxThreshold()) / ((double)1023 - (double)getFinger10PhalanxThreshold());
    setFinger10PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger10TipReading() const
{
    return finger10TipReading;
}

void ArmControlLeft::setFinger10TipReading(unsigned int value)
{
    finger10TipReading = value;
    float relationToInitial = ((double)finger10TipReading - (double)getFinger10TipThreshold()) / ((double)1023 - (double)getFinger10TipThreshold());
    setFinger10TipReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger9PhalanxReading() const
{
    return finger9PhalanxReading;
}

void ArmControlLeft::setFinger9PhalanxReading(unsigned int value)
{
    finger9PhalanxReading = value;
    float relationToInitial = ((double)finger9PhalanxReading - (double)getFinger9PhalanxThreshold()) / ((double)1023 - (double)getFinger9PhalanxThreshold());
    setFinger9PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger8PhalanxReading() const
{
    return finger8PhalanxReading;
}

void ArmControlLeft::setFinger8PhalanxReading(unsigned int value)
{
    finger8PhalanxReading = value;
    float relationToInitial = ((double)finger8PhalanxReading - (double)getFinger8PhalanxThreshold()) / ((double)1023 - (double)getFinger8PhalanxThreshold());
    setFinger8PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger7TipReading() const
{
    return finger7TipReading;
}

void ArmControlLeft::setFinger7TipReading(unsigned int value)
{
    finger7TipReading = value;
    float relationToInitial = ((double)finger7TipReading - (double)getFinger7TipThreshold()) / ((double)1023 - (double)getFinger7TipThreshold());
    setFinger7TipReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger7PhalanxReading() const
{
    return finger7PhalanxReading;
}

void ArmControlLeft::setFinger7PhalanxReading(unsigned int value)
{
    finger7PhalanxReading = value;
    float relationToInitial = ((double)finger7PhalanxReading - (double)getFinger7PhalanxThreshold()) / ((double)1023 - (double)getFinger7PhalanxThreshold());
    setFinger7PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger6PhalanxReading() const
{
    return finger6PhalanxReading;
}

void ArmControlLeft::setFinger6PhalanxReading(unsigned int value)
{
    finger6PhalanxReading = value;
    float relationToInitial = ((double)finger6PhalanxReading - (double)getFinger6PhalanxThreshold()) / ((double)1023 - (double)getFinger6PhalanxThreshold());
    setFinger6PhalanxReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getFinger6TipReading() const
{
    return finger6TipReading;
}

void ArmControlLeft::setFinger6TipReading(unsigned int value)
{
    finger6TipReading = value;
    float relationToInitial = ((double)finger6TipReading - (double)getFinger6TipThreshold()) / ((double)1023 - (double)getFinger6TipThreshold());
    setFinger6TipReadingRelative(relationToInitial);
}

unsigned int ArmControlLeft::getPalmJambThreshold() const
{
    return palmJambThreshold;
}

void ArmControlLeft::setPalmJambThreshold(unsigned int value)
{
    palmJambThreshold = value;
}

unsigned int ArmControlLeft::getPalmLowerThreshold() const
{
    return palmLowerThreshold;
}

void ArmControlLeft::setPalmLowerThreshold(unsigned int value)
{
    palmLowerThreshold = value;
}

unsigned int ArmControlLeft::getPalmUpperThreshold() const
{
    return palmUpperThreshold;
}

void ArmControlLeft::setPalmUpperThreshold(unsigned int value)
{
    palmUpperThreshold = value;
}

unsigned int ArmControlLeft::getFinger11PhalanxThreshold() const
{
    return finger11PhalanxThreshold;
}

void ArmControlLeft::setFinger11PhalanxThreshold(unsigned int value)
{
    finger11PhalanxThreshold = value;
}

unsigned int ArmControlLeft::getFinger11TipThreshold() const
{
    return finger11TipThreshold;
}

void ArmControlLeft::setFinger11TipThreshold(unsigned int value)
{
    finger11TipThreshold = value;
}

unsigned int ArmControlLeft::getFinger10PhalanxThreshold() const
{
    return finger10PhalanxThreshold;
}

void ArmControlLeft::setFinger10PhalanxThreshold(unsigned int value)
{
    finger10PhalanxThreshold = value;
}

unsigned int ArmControlLeft::getFinger10TipThreshold() const
{
    return finger10TipThreshold;
}

void ArmControlLeft::setFinger10TipThreshold(unsigned int value)
{
    finger10TipThreshold = value;
}

unsigned int ArmControlLeft::getFinger9PhalanxThreshold() const
{
    return finger9PhalanxThreshold;
}

void ArmControlLeft::setFinger9PhalanxThreshold(unsigned int value)
{
    finger9PhalanxThreshold = value;
}

unsigned int ArmControlLeft::getFinger8PhalanxThreshold() const
{
    return finger8PhalanxThreshold;
}

void ArmControlLeft::setFinger8PhalanxThreshold(unsigned int value)
{
    finger8PhalanxThreshold = value;
}

unsigned int ArmControlLeft::getFinger7TipThreshold() const
{
    return finger7TipThreshold;
}

void ArmControlLeft::setFinger7TipThreshold(unsigned int value)
{
    finger7TipThreshold = value;
}

unsigned int ArmControlLeft::getFinger7PhalanxThreshold() const
{
    return finger7PhalanxThreshold;
}

void ArmControlLeft::setFinger7PhalanxThreshold(unsigned int value)
{
    finger7PhalanxThreshold = value;
}

unsigned int ArmControlLeft::getFinger6PhalanxThreshold() const
{
    return finger6PhalanxThreshold;
}

void ArmControlLeft::setFinger6PhalanxThreshold(unsigned int value)
{
    finger6PhalanxThreshold = value;
}

unsigned int ArmControlLeft::getFinger6TipThreshold() const
{
    return finger6TipThreshold;
}

void ArmControlLeft::setFinger6TipThreshold(unsigned int value)
{
    finger6TipThreshold = value;
}

bool ArmControlLeft::getForearmYawDirection() const
{
    return forearmYawDirection;
}
