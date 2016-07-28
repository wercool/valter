#include "valter.h"


void BodyControlP1::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    head24VState            = false;
    headYawMotorState       = false;
    headPitchMotorState     = false;

    headYawDirection            = false;  //true - turn right, false - turn left
    headYawMotorActivated       = false;
    headYawStepDelay            = 8000;
    headYawStepSwitchDelay      = 100;
    headYawADCPosition          = 0;
    headYawStepPostion          = 0;

    headPitchDirection          = true;  //true - pitch down, false - pitch up
    headPitchMotorActivated     = false;
    headPitchStepDelay          = 4000; //us
    headPitchStepSwitchDelay    = 100; //us
    headPitchADCPosition        = 0;
    headPitchStepPosition       = 0;

    //------------------------------------------------body pitch
    bodyPitchMotorDuty          = 1;
    bodyPitchMotorDutyMax       = 30;
    bodyPitchMotorAcceleration  = 2;
    bodyPitchMotorDeceleration  = 2;

    bodyPitchMotorAccelerating  = false;
    bodyPitchMotorDecelerating  = false;

    bodyPitchMovementDirection  = true; //true - pitch down, false - pitch up
    bodyPitchMotorActivated     = false;
    bodyPitchMotorStop          = true;

    bodyPitchADCPosition        = 0;
    bodyPitchADCCurrent         = 0;

    //presets
    bodyPitchMotorDutyPresetCur     = 30;
    bodyPitchMotorDutyPresetMin     = 1;
    bodyPitchMotorDutyPresetMax     = 100;

    //------------------------------------------------right arm yaw
    rightArmYawMotorDuty            = 1;
    rightArmYawMotorDutyMax         = 30;
    rightArmYawMotorAcceleration    = 2;
    rightArmYawMotorDeceleration    = 2;

    rightArmYawMotorAccelerating    = false;
    rightArmYawMotorDecelerating    = false;

    rightArmYawMovementDirection    = true; //true - open, false - close
    rightArmYawMotorActivated       = false;
    rightArmYawMotorStop            = false;

    rightArmYawADCPosition          = 0;
    rightArmYawADCCurrent           = 0;

    //presets
    rightArmYawMotorDutyPresetCur   = 30;
    rightArmYawMotorDutyPresetMin   = 1;
    rightArmYawMotorDutyPresetMax   = 100;

    //------------------------------------------------left arm yaw
    leftArmYawMotorDuty             = 1;
    leftArmYawMotorDutyMax          = 30;
    leftArmYawMotorAcceleration     = 2;
    leftArmYawMotorDeceleration     = 2;

    leftArmYawMotorAccelerating     = false;
    leftArmYawMotorDecelerating     = false;

    leftArmYawMovementDirection     = true; //true - open, false - close
    leftArmYawMotorActivated        = false;
    leftArmYawMotorStop             = false;

    leftArmYawADCPosition           = 0;
    leftArmYawADCCurrent            = 0;

    //presets
    leftArmYawMotorDutyPresetCur    = 30;
    leftArmYawMotorDutyPresetMin    = 1;
    leftArmYawMotorDutyPresetMax    = 100;

    //body control p1 sensors reading settings
    bodyPitchPositionTrack          = false;
    bodyPitchPositionADC            = true;
    bodyPitchCurrentTrack           = false;
    bodyPitchCurrentADC             = true;
    rightArmYawPositionTrack        = false;
    rightArmYawPositionADC          = true;
    rightArmYawCurrentTrack         = false;
    rightArmYawCurrentADC           = true;
    leftArmYawPositionTrack         = false;
    leftArmYawPositionADC           = true;
    leftArmYawCurrentTrack          = false;
    leftArmYawCurrentADC            = true;
    headPitchPositionTrack          = false;
    headPitchPositionADC            = true;
    headYawPositionTrack            = false;
    headYawPositionADC              = true;

    //5.5V
    powerSource5V5State             = false;
    //wifi
    wifiPowerState                  = false;

    //right arm roll 24V state
    rightArm24VPowerSourceState     = false;

    //left arm roll 24V state
    leftArm24VPowerSourceState      = false;

    //head led
    headLedState                    = false;

    //left accumulator connected state
    leftAccumulatorConnectedState   = false;

    //right accumulator connected state
    rightAccumulatorConnectedState  = false;

    kinect1PowerState               = false;
    kinect2PowerState               = false;

    bodyCameraLowerPosition     = 1650;
    bodyCameraCenterPosition    = 1460;
    bodyCameraUpperPosition     = 1020;
}

void BodyControlP1::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + BodyControlP1::defaultsFilePath);
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

    //bodyPitchMotorDuty
    defaultValue = getDefault("bodyPitchMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setBodyPitchMotorDutyPresetMin(minValue);
    setBodyPitchMotorDutyPresetMax(maxValue);
    setBodyPitchMotorDutyPresetCur(curValue);
    setBodyPitchMotorDutyMax(getBodyPitchMotorDutyPresetCur());

    //bodyPitchMotorDeceleration
    defaultValue = getDefault("bodyPitchMotorDeceleration");
    bodyPitchMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //bodyPitchMotorAcceleration
    defaultValue = getDefault("bodyPitchMotorAcceleration");
    bodyPitchMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightArmYawMotorDuty
    defaultValue = getDefault("rightArmYawMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightArmYawMotorDutyPresetMin(minValue);
    setRightArmYawMotorDutyPresetMax(maxValue);
    setRightArmYawMotorDutyPresetCur(curValue);
    setRightArmYawMotorDutyMax(getRightArmYawMotorDutyPresetCur());

    //rightArmYawMotorDeceleration
    defaultValue = getDefault("rightArmYawMotorDeceleration");
    rightArmYawMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //rightArmYawMotorAcceleration
    defaultValue = getDefault("rightArmYawMotorAcceleration");
    rightArmYawMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftArmYawMotorDuty
    defaultValue = getDefault("leftArmYawMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftArmYawMotorDutyPresetMin(minValue);
    setLeftArmYawMotorDutyPresetMax(maxValue);
    setLeftArmYawMotorDutyPresetCur(curValue);
    setLeftArmYawMotorDutyMax(getLeftArmYawMotorDutyPresetCur());

    //leftArmYawMotorDeceleration
    defaultValue = getDefault("leftArmYawMotorDeceleration");
    leftArmYawMotorDeceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //leftArmYawMotorAcceleration
    defaultValue = getDefault("leftArmYawMotorAcceleration");
    leftArmYawMotorAcceleration = atoi(Valter::stringToCharPtr(defaultValue));

    //bodyPitchPosition
    defaultValue = getDefault("bodyPitchPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setBodyPitchPositionTrack(val1);
    setBodyPitchPositionADC(val2);

    //bodyPitchCurrent
    defaultValue = getDefault("bodyPitchCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setBodyPitchCurrentTrack(val1);
    setBodyPitchCurrentADC(val2);

    //rightArmYawPosition
    defaultValue = getDefault("rightArmYawPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setRightArmYawPositionTrack(val1);
    setRightArmYawPositionADC(val2);

    //rightArmYawCurrent
    defaultValue = getDefault("rightArmYawCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setRightArmYawCurrentTrack(val1);
    setRightArmYawCurrentADC(val2);

    //leftArmYawPosition
    defaultValue = getDefault("leftArmYawPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLeftArmYawPositionTrack(val1);
    setLeftArmYawPositionADC(val2);

    //leftArmYawCurrent
    defaultValue = getDefault("leftArmYawCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setLeftArmYawCurrentTrack(val1);
    setLeftArmYawCurrentADC(val2);

    //headPitchPosition
    defaultValue = getDefault("headPitchPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setHeadPitchPositionTrack(val1);
    setHeadPitchPositionADC(val2);

    //headYawPosition
    defaultValue = getDefault("headYawPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    val1 = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    val2 = atoi(strtok(NULL, "," ));
    setHeadYawPositionTrack(val1);
    setHeadYawPositionADC(val2);

    //bodyCameraServoPosition
    defaultValue = getDefault("bodyCameraServoPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    maxValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    minValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setBodyCameraLowerPosition(maxValue);
    setBodyCameraUpperPosition(minValue);
    setBodyCameraCenterPosition(curValue);
}

int BodyControlP1::getLeftArmYawMotorDutyPresetMin() const
{
    return leftArmYawMotorDutyPresetMin;
}

void BodyControlP1::setLeftArmYawMotorDutyPresetMin(int value)
{
    leftArmYawMotorDutyPresetMin = value;
}

int BodyControlP1::getLeftArmYawMotorDutyPresetCur() const
{
    return leftArmYawMotorDutyPresetCur;
}

void BodyControlP1::setLeftArmYawMotorDutyPresetCur(int value)
{
    leftArmYawMotorDutyPresetCur = value;
}

bool BodyControlP1::getLeftArmYawMotorStop() const
{
    return leftArmYawMotorStop;
}

void BodyControlP1::setLeftArmYawMotorStop(bool value)
{
    leftArmYawMotorStop = value;
}

bool BodyControlP1::getLeftArmYawMotorActivated() const
{
    return leftArmYawMotorActivated;
}

bool BodyControlP1::getLeftArmYawMotorDecelerating() const
{
    return leftArmYawMotorDecelerating;
}

void BodyControlP1::setLeftArmYawMotorDecelerating(bool value)
{
    leftArmYawMotorDecelerating = value;
}

bool BodyControlP1::getLeftArmYawMotorAccelerating() const
{
    return leftArmYawMotorAccelerating;
}

void BodyControlP1::setLeftArmYawMotorAccelerating(bool value)
{
    leftArmYawMotorAccelerating = value;
}

int BodyControlP1::getLeftArmYawMotorDeceleration() const
{
    return leftArmYawMotorDeceleration;
}

void BodyControlP1::setLeftArmYawMotorDeceleration(int value)
{
    leftArmYawMotorDeceleration = value;
}

int BodyControlP1::getLeftArmYawMotorAcceleration() const
{
    return leftArmYawMotorAcceleration;
}

void BodyControlP1::setLeftArmYawMotorAcceleration(int value)
{
    leftArmYawMotorAcceleration = value;
}

int BodyControlP1::getLeftArmYawMotorDutyMax() const
{
    return leftArmYawMotorDutyMax;
}

void BodyControlP1::setLeftArmYawMotorDutyMax(int value)
{
    leftArmYawMotorDutyMax = value;
}

int BodyControlP1::getLeftArmYawMotorDuty() const
{
    return leftArmYawMotorDuty;
}

int BodyControlP1::getRightArmYawMotorDutyPresetMax() const
{
    return rightArmYawMotorDutyPresetMax;
}

void BodyControlP1::setRightArmYawMotorDutyPresetMax(int value)
{
    rightArmYawMotorDutyPresetMax = value;
}

int BodyControlP1::getRightArmYawMotorDutyPresetMin() const
{
    return rightArmYawMotorDutyPresetMin;
}

void BodyControlP1::setRightArmYawMotorDutyPresetMin(int value)
{
    rightArmYawMotorDutyPresetMin = value;
}

int BodyControlP1::getRightArmYawMotorDutyPresetCur() const
{
    return rightArmYawMotorDutyPresetCur;
}

void BodyControlP1::setRightArmYawMotorDutyPresetCur(int value)
{
    rightArmYawMotorDutyPresetCur = value;
}

bool BodyControlP1::getRightArmYawMotorStop() const
{
    return rightArmYawMotorStop;
}

void BodyControlP1::setRightArmYawMotorStop(bool value)
{
    rightArmYawMotorStop = value;
}

bool BodyControlP1::getRightArmYawMotorActivated() const
{
    return rightArmYawMotorActivated;
}

bool BodyControlP1::getRightArmYawMotorDecelerating() const
{
    return rightArmYawMotorDecelerating;
}

void BodyControlP1::setRightArmYawMotorDecelerating(bool value)
{
    rightArmYawMotorDecelerating = value;
}

bool BodyControlP1::getRightArmYawMotorAccelerating() const
{
    return rightArmYawMotorAccelerating;
}

void BodyControlP1::setRightArmYawMotorAccelerating(bool value)
{
    rightArmYawMotorAccelerating = value;
}

int BodyControlP1::getRightArmYawMotorDeceleration() const
{
    return rightArmYawMotorDeceleration;
}

void BodyControlP1::setRightArmYawMotorDeceleration(int value)
{
    rightArmYawMotorDeceleration = value;
}

int BodyControlP1::getRightArmYawMotorAcceleration() const
{
    return rightArmYawMotorAcceleration;
}

void BodyControlP1::setRightArmYawMotorAcceleration(int value)
{
    rightArmYawMotorAcceleration = value;
}

int BodyControlP1::getRightArmYawMotorDutyMax() const
{
    return rightArmYawMotorDutyMax;
}

void BodyControlP1::setRightArmYawMotorDutyMax(int value)
{
    rightArmYawMotorDutyMax = value;
}

int BodyControlP1::getRightArmYawMotorDuty() const
{
    return rightArmYawMotorDuty;
}

int BodyControlP1::getBodyPitchMotorDutyPresetMax() const
{
    return bodyPitchMotorDutyPresetMax;
}

void BodyControlP1::setBodyPitchMotorDutyPresetMax(int value)
{
    bodyPitchMotorDutyPresetMax = value;
}

int BodyControlP1::getBodyPitchMotorDutyPresetMin() const
{
    return bodyPitchMotorDutyPresetMin;
}

void BodyControlP1::setBodyPitchMotorDutyPresetMin(int value)
{
    bodyPitchMotorDutyPresetMin = value;
}

int BodyControlP1::getBodyPitchMotorDutyPresetCur() const
{
    return bodyPitchMotorDutyPresetCur;
}

void BodyControlP1::setBodyPitchMotorDutyPresetCur(int value)
{
    bodyPitchMotorDutyPresetCur = value;
}

bool BodyControlP1::getBodyPitchMotorStop() const
{
    return bodyPitchMotorStop;
}

void BodyControlP1::setBodyPitchMotorStop(bool value)
{
    bodyPitchMotorStop = value;
}

bool BodyControlP1::getBodyPitchMotorActivated() const
{
    return bodyPitchMotorActivated;
}

bool BodyControlP1::getBodyPitchMotorDecelerating() const
{
    return bodyPitchMotorDecelerating;
}

void BodyControlP1::setBodyPitchMotorDecelerating(bool value)
{
    bodyPitchMotorDecelerating = value;
}

bool BodyControlP1::getBodyPitchMotorAccelerating() const
{
    return bodyPitchMotorAccelerating;
}

void BodyControlP1::setBodyPitchMotorAccelerating(bool value)
{
    bodyPitchMotorAccelerating = value;
}

int BodyControlP1::getBodyPitchMotorDeceleration() const
{
    return bodyPitchMotorDeceleration;
}

void BodyControlP1::setBodyPitchMotorDeceleration(int value)
{
    bodyPitchMotorDeceleration = value;
}

int BodyControlP1::getBodyPitchMotorAcceleration() const
{
    return bodyPitchMotorAcceleration;
}

void BodyControlP1::setBodyPitchMotorAcceleration(int value)
{
    bodyPitchMotorAcceleration = value;
}

int BodyControlP1::getBodyPitchMotorDutyMax() const
{
    return bodyPitchMotorDutyMax;
}

void BodyControlP1::setBodyPitchMotorDutyMax(int value)
{
    bodyPitchMotorDutyMax = value;
}

int BodyControlP1::getBodyPitchMotorDuty() const
{
    return bodyPitchMotorDuty;
}


bool BodyControlP1::getHeadPitchMotorState() const
{
    return headPitchMotorState;
}

void BodyControlP1::setHeadPitchMotorState(bool value)
{
    headPitchMotorState = value;
}

bool BodyControlP1::getHeadYawMotorState() const
{
    return headYawMotorState;
}

void BodyControlP1::setHeadYawMotorState(bool value)
{
    headYawMotorState = value;
}

bool BodyControlP1::getHead24VState() const
{
    return head24VState;
}

void BodyControlP1::setHead24VState(bool value)
{
    head24VState = value;
}

int BodyControlP1::getHeadPitchStepSwitchDelay() const
{
    return headPitchStepSwitchDelay;
}

void BodyControlP1::setHeadPitchStepSwitchDelay(int value)
{
    headPitchStepSwitchDelay = value;
}

int BodyControlP1::getHeadPitchStepDelay() const
{
    return headPitchStepDelay;
}

void BodyControlP1::setHeadPitchStepDelay(int value)
{
    headPitchStepDelay = value;
}

bool BodyControlP1::getHeadPitchMotorActivated() const
{
    return headPitchMotorActivated;
}

void BodyControlP1::setHeadPitchMotorActivated(bool value)
{
    headPitchMotorActivated = value;
}

bool BodyControlP1::getHeadPitchDirection() const
{
    return headPitchDirection;
}

int BodyControlP1::getHeadYawStepSwitchDelay() const
{
    return headYawStepSwitchDelay;
}

void BodyControlP1::setHeadYawStepSwitchDelay(int value)
{
    headYawStepSwitchDelay = value;
}

int BodyControlP1::getHeadYawStepDelay() const
{
    return headYawStepDelay;
}

void BodyControlP1::setHeadYawStepDelay(int value)
{
    headYawStepDelay = value;
}

bool BodyControlP1::getHeadYawMotorActivated() const
{
    return headYawMotorActivated;
}

void BodyControlP1::setHeadYawMotorActivated(bool value)
{
    headYawMotorActivated = value;
}

bool BodyControlP1::getHeadYawDirection() const
{
    return headYawDirection;
}

int BodyControlP1::getLeftArmYawADCPosition() const
{
    return leftArmYawADCPosition;
}

void BodyControlP1::setLeftArmYawADCPosition(int value)
{
    leftArmYawADCPosition = value;
}

int BodyControlP1::getRightArmYawADCPosition() const
{
    return rightArmYawADCPosition;
}

void BodyControlP1::setRightArmYawADCPosition(int value)
{
    rightArmYawADCPosition = value;
}

int BodyControlP1::getBodyPitchADCPosition() const
{
    return bodyPitchADCPosition;
}

void BodyControlP1::setBodyPitchADCPosition(int value)
{
    bodyPitchADCPosition = value;
}

bool BodyControlP1::getHeadYawPositionADC() const
{
    return headYawPositionADC;
}

void BodyControlP1::setHeadYawPositionADC(bool value)
{
    headYawPositionADC = value;
}

bool BodyControlP1::getHeadYawPositionTrack() const
{
    return headYawPositionTrack;
}

void BodyControlP1::setHeadYawPositionTrack(bool value)
{
    headYawPositionTrack = value;
}

bool BodyControlP1::getHeadPitchPositionADC() const
{
    return headPitchPositionADC;
}

void BodyControlP1::setHeadPitchPositionADC(bool value)
{
    headPitchPositionADC = value;
}

bool BodyControlP1::getHeadPitchPositionTrack() const
{
    return headPitchPositionTrack;
}

void BodyControlP1::setHeadPitchPositionTrack(bool value)
{
    headPitchPositionTrack = value;
}

bool BodyControlP1::getLeftArmYawCurrentADC() const
{
    return leftArmYawCurrentADC;
}

void BodyControlP1::setLeftArmYawCurrentADC(bool value)
{
    leftArmYawCurrentADC = value;
}

bool BodyControlP1::getLeftArmYawCurrentTrack() const
{
    return leftArmYawCurrentTrack;
}

void BodyControlP1::setLeftArmYawCurrentTrack(bool value)
{
    leftArmYawCurrentTrack = value;
}

bool BodyControlP1::getLeftArmYawPositionADC() const
{
    return leftArmYawPositionADC;
}

void BodyControlP1::setLeftArmYawPositionADC(bool value)
{
    leftArmYawPositionADC = value;
}

bool BodyControlP1::getLeftArmYawPositionTrack() const
{
    return leftArmYawPositionTrack;
}

void BodyControlP1::setLeftArmYawPositionTrack(bool value)
{
    leftArmYawPositionTrack = value;
}

bool BodyControlP1::getRightArmYawCurrentADC() const
{
    return rightArmYawCurrentADC;
}

void BodyControlP1::setRightArmYawCurrentADC(bool value)
{
    rightArmYawCurrentADC = value;
}

bool BodyControlP1::getRightArmYawCurrentTrack() const
{
    return rightArmYawCurrentTrack;
}

void BodyControlP1::setRightArmYawCurrentTrack(bool value)
{
    rightArmYawCurrentTrack = value;
}

bool BodyControlP1::getRightArmYawPositionADC() const
{
    return rightArmYawPositionADC;
}

void BodyControlP1::setRightArmYawPositionADC(bool value)
{
    rightArmYawPositionADC = value;
}

bool BodyControlP1::getRightArmYawPositionTrack() const
{
    return rightArmYawPositionTrack;
}

void BodyControlP1::setRightArmYawPositionTrack(bool value)
{
    rightArmYawPositionTrack = value;
}

bool BodyControlP1::getBodyPitchCurrentADC() const
{
    return bodyPitchCurrentADC;
}

void BodyControlP1::setBodyPitchCurrentADC(bool value)
{
    bodyPitchCurrentADC = value;
}

bool BodyControlP1::getBodyPitchCurrentTrack() const
{
    return bodyPitchCurrentTrack;
}

void BodyControlP1::setBodyPitchCurrentTrack(bool value)
{
    bodyPitchCurrentTrack = value;
}

bool BodyControlP1::getBodyPitchPositionADC() const
{
    return bodyPitchPositionADC;
}

void BodyControlP1::setBodyPitchPositionADC(bool value)
{
    bodyPitchPositionADC = value;
}

bool BodyControlP1::getBodyPitchPositionTrack() const
{
    return bodyPitchPositionTrack;
}

void BodyControlP1::setBodyPitchPositionTrack(bool value)
{
    bodyPitchPositionTrack = value;
}

int BodyControlP1::getLeftArmYawMotorDutyPresetMax() const
{
    return leftArmYawMotorDutyPresetMax;
}

void BodyControlP1::setLeftArmYawMotorDutyPresetMax(int value)
{
    leftArmYawMotorDutyPresetMax = value;
}

bool BodyControlP1::getLeftArmYawMovementDirection() const
{
    return leftArmYawMovementDirection;
}

int BodyControlP1::getHeadYawADCPosition() const
{
    return headYawADCPosition;
}

void BodyControlP1::setHeadYawADCPosition(int value)
{
    headYawADCPosition = value;
}

int BodyControlP1::getHeadPitchADCPosition() const
{
    return headPitchADCPosition;
}

void BodyControlP1::setHeadPitchADCPosition(int value)
{
//    static int averagerCnt = 0;
//    static int averager = 0;

//    if (averagerCnt < 10)
//    {
//        averager += value;
//        averagerCnt++;
//    }
//    else
//    {
//        headPitchADCPosition = round((double)averager / (double)10);
//        averagerCnt = 0;
//        averager = 0;
//        //qDebug("headPitchADCPosition = %d", headPitchADCPosition);
//    }
    headPitchADCPosition = value;
}
int BodyControlP1::getRightArmYawADCCurrent() const
{
    return rightArmYawADCCurrent;
}

void BodyControlP1::setRightArmYawADCCurrent(int value)
{
    rightArmYawADCCurrent = value;
}

int BodyControlP1::getBodyPitchADCCurrent() const
{
    return bodyPitchADCCurrent;
}

void BodyControlP1::setBodyPitchADCCurrent(int value)
{
    bodyPitchADCCurrent = value;
}

int BodyControlP1::getHeadPitchStepPosition() const
{
    return headPitchStepPosition;
}

void BodyControlP1::setHeadPitchStepPosition(int value)
{
    headPitchStepPosition = value;
}

int BodyControlP1::getHeadYawStepPostion() const
{
    return headYawStepPostion;
}

void BodyControlP1::setHeadYawStepPostion(int value)
{
    headYawStepPostion = value;
}

bool BodyControlP1::getPowerSource5V5State() const
{
    return powerSource5V5State;
}

void BodyControlP1::setPowerSource5V5State(bool value)
{
    powerSource5V5State = value;
}

int BodyControlP1::getLeftArmYawADCCurrent() const
{
    return leftArmYawADCCurrent;
}

void BodyControlP1::setLeftArmYawADCCurrent(int value)
{
    leftArmYawADCCurrent = value;
}

bool BodyControlP1::getWifiPowerState() const
{
    return wifiPowerState;
}

void BodyControlP1::setWifiPowerState(bool value)
{
    wifiPowerState = value;
}

bool BodyControlP1::getLeftArm24VPowerSourceState() const
{
    return leftArm24VPowerSourceState;
}

void BodyControlP1::setLeftArm24VPowerSourceState(bool value)
{
    leftArm24VPowerSourceState = value;
}

bool BodyControlP1::getRightArm24VPowerSourceState() const
{
    return rightArm24VPowerSourceState;
}

void BodyControlP1::setRightArm24VPowerSourceState(bool value)
{
    rightArm24VPowerSourceState = value;
}

bool BodyControlP1::getHeadLedState() const
{
    return headLedState;
}

void BodyControlP1::setHeadLedState(bool value)
{
    headLedState = value;
}

bool BodyControlP1::getRightAccumulatorConnectedState() const
{
    return rightAccumulatorConnectedState;
}

void BodyControlP1::setRightAccumulatorConnectedState(bool value)
{
    rightAccumulatorConnectedState = value;
}

bool BodyControlP1::getLeftAccumulatorConnectedState() const
{
    return leftAccumulatorConnectedState;
}

void BodyControlP1::setLeftAccumulatorConnectedState(bool value)
{
    leftAccumulatorConnectedState = value;
}

bool BodyControlP1::getLeftArm12VPowerSourceState() const
{
    return leftArm12VPowerSourceState;
}

void BodyControlP1::setLeftArm12VPowerSourceState(bool value)
{
    leftArm12VPowerSourceState = value;
}

bool BodyControlP1::getRightArm12VPowerSourceState() const
{
    return rightArm12VPowerSourceState;
}

void BodyControlP1::setRightArm12VPowerSourceState(bool value)
{
    rightArm12VPowerSourceState = value;
}

bool BodyControlP1::getKinect2PowerState() const
{
    return kinect2PowerState;
}

void BodyControlP1::setKinect2PowerState(bool value)
{
    kinect2PowerState = value;
}

bool BodyControlP1::getKinect1PowerState() const
{
    return kinect1PowerState;
}

void BodyControlP1::setKinect1PowerState(bool value)
{
    kinect1PowerState = value;
}
int BodyControlP1::getBodyCameraUpperPosition() const
{
    return bodyCameraUpperPosition;
}

void BodyControlP1::setBodyCameraUpperPosition(int value)
{
    bodyCameraUpperPosition = value;
}

int BodyControlP1::getBodyCameraCenterPosition() const
{
    return bodyCameraCenterPosition;
}

void BodyControlP1::setBodyCameraCenterPosition(int value)
{
    bodyCameraCenterPosition = value;
}

int BodyControlP1::getBodyCameraLowerPosition() const
{
    return bodyCameraLowerPosition;
}

void BodyControlP1::setBodyCameraLowerPosition(int value)
{
    bodyCameraLowerPosition = value;
}
