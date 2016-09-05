#include "valter.h"

void PlatformControlP1::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    power5VOnState                      = false;
    leftAccumulatorConnected            = false;
    rightAccumulatorConnected           = false;
    mainAccumulatorRelayOnState         = false;
    leftAccumulatorRelayOnState         = false;
    rightAccumulatorRelayOnState        = false;
    power220VACAvailable                = false;
    charger35Ah                         = false;
    charger120Ah                        = false;
    chargingInProgress                  = false;
    chargingComplete                    = false;
    scan220ACAvailable                  = false;
    chargerConnected                    = false;
    //platform motors
    leftMotorDirection                  = true;
    rightMotorDirection                 = true;
    leftMotorStop                       = true;
    rightMotorStop                      = true;
    platformEmergencyStop               = false;
    leftMotorActivated                  = false;
    rightMotorActivated                 = false;
    leftMotorAccelerating               = false;
    rightMotorAccelerating              = false;
    leftMotorDecelerating               = false;
    rightMotorDecelerating              = false;
    //turret motor
    turretEmergencyStop                 = false;
    turretMotorDirection                = true;
    turretMotorStop                     = true;
    turretMotorActivated                = false;
    turretMotorAccelerating             = false;
    turretMotorDecelerating             = false;

    leftMotorCurrentRead                = false;
    rightMotorCurrentRead               = false;
    turretMotorCurrentRead              = false;

    leftWheelEncoderRead                = false;
    rightWheelEncoderRead               = false;
    leftWheelEncoderAutoreset           = false;
    rightWheelEncoderAutoreset          = false;
    leftWheelEncoderGetOnce              = false;
    rightWheelEncoderGetOnce             = false;

    turretPositionRead                  = false;
    turretPositionGetOnce               = false;

    mainAccumulatorVoltageRead          = false;
    leftAccumulatorVoltageRead          = false;
    rightAccumulatorVoltageRead         = false;
    mainAccumulatorAmperageTotalRead    = false;
    platformAmperageRead                = false;
    bodyAmperageRead                    = false;
    leftAccumulatorAmperageRead         = false;
    rightAccumulatorAmperageRead        = false;
    chargerVoltageRead                  = false;

    mainAccumulatorVoltageReadADCPreset         = false;
    leftAccumulatorVoltageReadADCPreset         = false;
    rightAccumulatorVoltageReadADCPreset        = false;
    mainAccumulatorAmperageTotalReadADCPreset   = false;
    platformAmperageReadADCPreset               = false;
    bodyAmperageReadADCPreset                   = false;
    leftAccumulatorAmperageReadADCPreset        = false;
    rightAccumulatorAmperageReadADCPreset       = false;
    chargerVoltageReadADCPreset                 = false;

    curChannel1Input                    = 0;
    curChannel2Input                    = 0;
    //platform motors
    leftMotorDutyMax                    = 1;
    rightMotorDutyMax                   = 1;
    leftMotorDuty                       = 1;
    rightMotorDuty                      = 1;
    platformDeceleration                = 1;
    platformAcceleration                = 1;
    //turret motor
    turretDeceleration                  = 1;
    turretAcceleration                  = 1;
    turretMotorDutyMax                  = 1;
    turretMotorDuty                     = 1;

    motorsPWMFrequncy                   = 8000;
    leftMotorCurrentADC                 = 0;
    rightMotorCurrentADC                = 0;
    turretMotorCurrentADC               = 0;
    leftMotorCurrentAmps                = 0.0;
    rightMotorCurrentAmps               = 0.0;
    turretMotorCurrentAmps              = 0.0;

    leftWheelEncoder                    = 0;
    rightWheelEncoder                   = 0;

    turretPositionADC                   = 0;
    turretPositionDeg                   = 0.0;
    turretPositionRad                   = 0.0;

    chargerVoltageADC                   = 0;
    chargerVoltageVolts                 = 0.0;

    mainAccumulatorVoltageADC           = 0;
    leftAccumulatorVoltageADC           = 0;
    rightAccumulatorVoltageADC          = 0;
    mainAccumulatorAmperageTotalADC     = 0;
    platformAmperageADC                 = 0;
    bodyAmperageADC                     = 0;
    leftAccumulatorAmperageADC          = 0;
    rightAccumulatorAmperageADC         = 0;

    mainAccumulatorVoltageVolts         = 0.0;
    leftAccumulatorVoltageVolts         = 0.0;
    rightAccumulatorVoltageVolts        = 0.0;
    mainAccumulatorAmperageTotalAmps    = 0.0;
    platformAmperageAmps                = 0.0;
    bodyAmperageAmps                    = 0.0;
    leftAccumulatorAmperageAmps         = 0.0;
    rightAccumulatorAmperageAmps        = 0.0;

    additionalReadingsDelayPresetMin    = 5;
    additionalReadingsDelayPresetMax    = 1000;
    additionalReadingsDelayCur          = 250;

    if (chargerButtonPressStep != 0)
    {
        setChargerMode(false);
    }

    if (this->controlDeviceIsSet)
    {
        getControlDevice()->addMsgToDataExchangeLog(Valter::format_string("%s Module Reset to default!", PlatformControlP1::controlDeviceId.c_str()));
    }

    values["leftWheelEncoder"] = "0";
    values["rightWheelEncoder"] = "0";
}

string PlatformControlP1::getValue(string key)
{
    if (key[key.length()-1] == '\n')
    {
        key.erase(key.length()-1);
    }

    if (values.find(key) == values.end())
    {
        return "undefined";
    }
    else
    {
        return values[key];
    }
}

int PlatformControlP1::getAdditionalReadingsDelayCur() const
{
    return additionalReadingsDelayCur;
}

void PlatformControlP1::setAdditionalReadingsDelayCur(int value)
{
    additionalReadingsDelayCur = value;
}

int PlatformControlP1::getAdditionalReadingsDelayPresetMax() const
{
    return additionalReadingsDelayPresetMax;
}

void PlatformControlP1::setAdditionalReadingsDelayPresetMax(int value)
{
    additionalReadingsDelayPresetMax = value;
}

int PlatformControlP1::getAdditionalReadingsDelayPresetMin() const
{
    return additionalReadingsDelayPresetMin;
}

void PlatformControlP1::setAdditionalReadingsDelayPresetMin(int value)
{
    additionalReadingsDelayPresetMin = value;
}
bool PlatformControlP1::getChargerVoltageReadADCPreset() const
{
    return chargerVoltageReadADCPreset;
}

void PlatformControlP1::setChargerVoltageReadADCPreset(bool value)
{
    chargerVoltageReadADCPreset = value;
}

bool PlatformControlP1::getRightAccumulatorAmperageReadADCPreset() const
{
    return rightAccumulatorAmperageReadADCPreset;
}

void PlatformControlP1::setRightAccumulatorAmperageReadADCPreset(bool value)
{
    rightAccumulatorAmperageReadADCPreset = value;
}

bool PlatformControlP1::getLeftAccumulatorAmperageReadADCPreset() const
{
    return leftAccumulatorAmperageReadADCPreset;
}

void PlatformControlP1::setLeftAccumulatorAmperageReadADCPreset(bool value)
{
    leftAccumulatorAmperageReadADCPreset = value;
}

bool PlatformControlP1::getBodyAmperageReadADCPreset() const
{
    return bodyAmperageReadADCPreset;
}

void PlatformControlP1::setBodyAmperageReadADCPreset(bool value)
{
    bodyAmperageReadADCPreset = value;
}

bool PlatformControlP1::getPlatformAmperageReadADCPreset() const
{
    return platformAmperageReadADCPreset;
}

void PlatformControlP1::setPlatformAmperageReadADCPreset(bool value)
{
    platformAmperageReadADCPreset = value;
}

bool PlatformControlP1::getMainAccumulatorAmperageTotalReadADCPreset() const
{
    return mainAccumulatorAmperageTotalReadADCPreset;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalReadADCPreset(bool value)
{
    mainAccumulatorAmperageTotalReadADCPreset = value;
}

bool PlatformControlP1::getRightAccumulatorVoltageReadADCPreset() const
{
    return rightAccumulatorVoltageReadADCPreset;
}

void PlatformControlP1::setRightAccumulatorVoltageReadADCPreset(bool value)
{
    rightAccumulatorVoltageReadADCPreset = value;
}

bool PlatformControlP1::getLeftAccumulatorVoltageReadADCPreset() const
{
    return leftAccumulatorVoltageReadADCPreset;
}

void PlatformControlP1::setLeftAccumulatorVoltageReadADCPreset(bool value)
{
    leftAccumulatorVoltageReadADCPreset = value;
}

bool PlatformControlP1::getMainAccumulatorVoltageReadADCPreset() const
{
    return mainAccumulatorVoltageReadADCPreset;
}

void PlatformControlP1::setMainAccumulatorVoltageReadADCPreset(bool value)
{
    mainAccumulatorVoltageReadADCPreset = value;
}

bool PlatformControlP1::getChargerVoltageRead() const
{
    return chargerVoltageRead;
}

void PlatformControlP1::setChargerVoltageRead(bool value)
{
    chargerVoltageRead = value;
}

bool PlatformControlP1::getRightAccumulatorAmperageRead() const
{
    return rightAccumulatorAmperageRead;
}

void PlatformControlP1::setRightAccumulatorAmperageRead(bool value)
{
    rightAccumulatorAmperageRead = value;
}

bool PlatformControlP1::getLeftAccumulatorAmperageRead() const
{
    return leftAccumulatorAmperageRead;
}

void PlatformControlP1::setLeftAccumulatorAmperageRead(bool value)
{
    leftAccumulatorAmperageRead = value;
}

bool PlatformControlP1::getBodyAmperageRead() const
{
    return bodyAmperageRead;
}

void PlatformControlP1::setBodyAmperageRead(bool value)
{
    bodyAmperageRead = value;
}

bool PlatformControlP1::getPlatformAmperageRead() const
{
    return platformAmperageRead;
}

void PlatformControlP1::setPlatformAmperageRead(bool value)
{
    platformAmperageRead = value;
}

bool PlatformControlP1::getMainAccumulatorAmperageTotalRead() const
{
    return mainAccumulatorAmperageTotalRead;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalRead(bool value)
{
    mainAccumulatorAmperageTotalRead = value;
}

bool PlatformControlP1::getRightAccumulatorVoltageRead() const
{
    return rightAccumulatorVoltageRead;
}

void PlatformControlP1::setRightAccumulatorVoltageRead(bool value)
{
    rightAccumulatorVoltageRead = value;
}

bool PlatformControlP1::getLeftAccumulatorVoltageRead() const
{
    return leftAccumulatorVoltageRead;
}

void PlatformControlP1::setLeftAccumulatorVoltageRead(bool value)
{
    leftAccumulatorVoltageRead = value;
}

bool PlatformControlP1::getMainAccumulatorVoltageRead() const
{
    return mainAccumulatorVoltageRead;
}

void PlatformControlP1::setMainAccumulatorVoltageRead(bool value)
{
    mainAccumulatorVoltageRead = value;
}

float PlatformControlP1::getRightAccumulatorAmperageAmps() const
{
    return rightAccumulatorAmperageAmps;
}

void PlatformControlP1::setRightAccumulatorAmperageAmps(float value)
{
    rightAccumulatorAmperageAmps = value;
}

float PlatformControlP1::getLeftAccumulatorAmperageAmps() const
{
    return leftAccumulatorAmperageAmps;
}

void PlatformControlP1::setLeftAccumulatorAmperageAmps(float value)
{
    leftAccumulatorAmperageAmps = value;
}

float PlatformControlP1::getBodyAmperageAmps() const
{
    return bodyAmperageAmps;
}

void PlatformControlP1::setBodyAmperageAmps(float value)
{
    bodyAmperageAmps = value;
}

float PlatformControlP1::getPlatformAmperageAmps() const
{
    return platformAmperageAmps;
}

void PlatformControlP1::setPlatformAmperageAmps(float value)
{
    platformAmperageAmps = value;
}

float PlatformControlP1::getMainAccumulatorAmperageTotalAmps() const
{
    return mainAccumulatorAmperageTotalAmps;
}

void PlatformControlP1::setMainAccumulatorAmperageTotalAmps(float value)
{
    mainAccumulatorAmperageTotalAmps = value;
}

float PlatformControlP1::getRightAccumulatorVoltageVolts() const
{
    return rightAccumulatorVoltageVolts;
}

void PlatformControlP1::setRightAccumulatorVoltageVolts(float value)
{
    rightAccumulatorVoltageVolts = value;
}

float PlatformControlP1::getLeftAccumulatorVoltageVolts() const
{
    return leftAccumulatorVoltageVolts;
}

void PlatformControlP1::setLeftAccumulatorVoltageVolts(float value)
{
    leftAccumulatorVoltageVolts = value;
}

float PlatformControlP1::getMainAccumulatorVoltageVolts() const
{
    return mainAccumulatorVoltageVolts;
}

void PlatformControlP1::setMainAccumulatorVoltageVolts(float value)
{
    mainAccumulatorVoltageVolts = value;
}

int PlatformControlP1::getRightAccumulatorAmperageADC() const
{
    return rightAccumulatorAmperageADC;
}

bool PlatformControlP1::getScan220ACAvailable() const
{
    return scan220ACAvailable;
}

void PlatformControlP1::setScan220ACAvailable(bool value)
{
    scan220ACAvailable = value;
}

int PlatformControlP1::getCurChannel1Input() const
{
    return curChannel1Input;
}

void PlatformControlP1::setCurChannel1Input(int value)
{
    curChannel1Input = value;
}


bool PlatformControlP1::getChargingComplete() const
{
    return chargingComplete;
}

void PlatformControlP1::setChargingComplete(bool value)
{
    chargingComplete = value;
}

bool PlatformControlP1::getChargingInProgress() const
{
    return chargingInProgress;
}

void PlatformControlP1::setChargingInProgress(bool value)
{
    chargingInProgress = value;
}

bool PlatformControlP1::getCharger120Ah() const
{
    return charger120Ah;
}

void PlatformControlP1::setCharger120Ah(bool value)
{
    charger120Ah = value;
}

bool PlatformControlP1::getCharger35Ah() const
{
    return charger35Ah;
}

void PlatformControlP1::setCharger35Ah(bool value)
{
    charger35Ah = value;
}

bool PlatformControlP1::getPower220VACAvailable() const
{
    return power220VACAvailable;
}

void PlatformControlP1::setPower220VACAvailable(bool value)
{
    value = false;
    if (getChargerVoltageADC() > 500)
    {
        value = true;
    }
    if (getChargerConnected())
    {
        value = true;
    }
    power220VACAvailable = value;
}

int PlatformControlP1::getRightMotorDutyMax() const
{
    return rightMotorDutyMax;
}

void PlatformControlP1::setRightMotorDutyMax(int value)
{
    rightMotorDutyMax = value;
}

int PlatformControlP1::getLeftMotorDutyMax() const
{
    return leftMotorDutyMax;
}

void PlatformControlP1::setLeftMotorDutyMax(int value)
{
    leftMotorDutyMax = value;
}

bool PlatformControlP1::getRightAccumulatorConnected() const
{
    return rightAccumulatorConnected;
}

void PlatformControlP1::setRightAccumulatorConnected(bool value)
{
    rightAccumulatorConnected = value;
}

bool PlatformControlP1::getLeftAccumulatorConnected() const
{
    return leftAccumulatorConnected;
}

void PlatformControlP1::setLeftAccumulatorConnected(bool value)
{
    leftAccumulatorConnected = value;
}

int PlatformControlP1::getRightMotorDuty() const
{
    return rightMotorDuty;
}

void PlatformControlP1::setRightMotorDuty(int value)
{
    rightMotorDuty = value;
    sendCommand(Valter::format_string("SETRIGHTMOTORPWMDUTY#%d", rightMotorDuty));
}

bool PlatformControlP1::getMainAccumulatorRelayOnState() const
{
    return mainAccumulatorRelayOnState;
}

void PlatformControlP1::setMainAccumulatorRelayOnState(bool value)
{
    mainAccumulatorRelayOnState = value;
}

bool PlatformControlP1::getPower5VOnState() const
{
    return power5VOnState;
}

void PlatformControlP1::setPower5VOnState(bool value)
{
    power5VOnState = value;
}

bool PlatformControlP1::getRightAccumulatorRelayOnState() const
{
    return rightAccumulatorRelayOnState;
}

void PlatformControlP1::setRightAccumulatorRelayOnState(bool value)
{
    rightAccumulatorRelayOnState = value;
}

bool PlatformControlP1::getLeftAccumulatorRelayOnState() const
{
    return leftAccumulatorRelayOnState;
}

void PlatformControlP1::setLeftAccumulatorRelayOnState(bool value)
{
    leftAccumulatorRelayOnState = value;
}

int PlatformControlP1::getCurChannel2Input() const
{
    return curChannel2Input;
}

void PlatformControlP1::setCurChannel2Input(int value)
{
    curChannel2Input = value;
}

float PlatformControlP1::getChargerVoltageVolts() const
{
    return chargerVoltageVolts;
}

void PlatformControlP1::setChargerVoltageVolts(float value)
{
    chargerVoltageVolts = value;
}

int PlatformControlP1::getChargerVoltageADC() const
{
    return chargerVoltageADC;
}

int PlatformControlP1::getChargerButtonPressStep() const
{
    return chargerButtonPressStep;
}

void PlatformControlP1::setChargerButtonPressStep(int value)
{
    chargerButtonPressStep = value;
}

bool PlatformControlP1::getChargerConnected() const
{
    return chargerConnected;
}

void PlatformControlP1::setChargerConnected(bool value)
{
    chargerConnected = value;
}

bool PlatformControlP1::getPlatformEmergencyStop() const
{
    return platformEmergencyStop;
}

void PlatformControlP1::setPlatformEmergencyStop(bool value)
{
    platformEmergencyStop = value;
}

bool PlatformControlP1::getRightMotorStop() const
{
    return rightMotorStop;
}

int PlatformControlP1::getTurretMotorDuty() const
{
    return turretMotorDuty;
}

void PlatformControlP1::setTurretMotorDuty(int value)
{
    turretMotorDuty = value;
}

int PlatformControlP1::getTurretMotorDutyMax() const
{
    return turretMotorDutyMax;
}

void PlatformControlP1::setTurretMotorDutyMax(int value)
{
    turretMotorDutyMax = value;
}

bool PlatformControlP1::getRightMotorDecelerating() const
{
    return rightMotorDecelerating;
}

void PlatformControlP1::setRightMotorDecelerating(bool value)
{
    rightMotorDecelerating = value;
}

bool PlatformControlP1::getLeftMotorDecelerating() const
{
    return leftMotorDecelerating;
}

void PlatformControlP1::setLeftMotorDecelerating(bool value)
{
    leftMotorDecelerating = value;
}

bool PlatformControlP1::getRightMotorAccelerating() const
{
    return rightMotorAccelerating;
}

void PlatformControlP1::setRightMotorAccelerating(bool value)
{
    rightMotorAccelerating = value;
}

bool PlatformControlP1::getLeftMotorAccelerating() const
{
    return leftMotorAccelerating;
}

void PlatformControlP1::setLeftMotorAccelerating(bool value)
{
    leftMotorAccelerating = value;
}

int PlatformControlP1::getPlatformAcceleration() const
{
    return platformAcceleration;
}

void PlatformControlP1::setPlatformAcceleration(int value)
{
    platformAcceleration = value;
}

int PlatformControlP1::getPlatformDeceleration() const
{
    return platformDeceleration;
}

void PlatformControlP1::setPlatformDeceleration(int value)
{
    platformDeceleration = value;
}

bool PlatformControlP1::getRightMotorActivated() const
{
    return rightMotorActivated;
}

bool PlatformControlP1::getTurretMotorStop() const
{
    return turretMotorStop;
}

bool PlatformControlP1::getTurretEmergencyStop() const
{
    return turretEmergencyStop;
}

void PlatformControlP1::setTurretEmergencyStop(bool value)
{
    turretEmergencyStop = value;
}

bool PlatformControlP1::getTurretMotorDecelerating() const
{
    return turretMotorDecelerating;
}

void PlatformControlP1::setTurretMotorDecelerating(bool value)
{
    turretMotorDecelerating = value;
}

bool PlatformControlP1::getTurretMotorAccelerating() const
{
    return turretMotorAccelerating;
}

void PlatformControlP1::setTurretMotorAccelerating(bool value)
{
    turretMotorAccelerating = value;
}

int PlatformControlP1::getTurretAcceleration() const
{
    return turretAcceleration;
}

void PlatformControlP1::setTurretAcceleration(int value)
{
    turretAcceleration = value;
}

int PlatformControlP1::getTurretDeceleration() const
{
    return turretDeceleration;
}

void PlatformControlP1::setTurretDeceleration(int value)
{
    turretDeceleration = value;
}

bool PlatformControlP1::getTurretMotorActivated() const
{
    return turretMotorActivated;
}

int PlatformControlP1::getTurretAccelerationPresetCur() const
{
    return turretAccelerationPresetCur;
}

void PlatformControlP1::setTurretAccelerationPresetCur(int value)
{
    turretAccelerationPresetCur = value;
}

int PlatformControlP1::getTurretAccelerationPresetMax() const
{
    return turretAccelerationPresetMax;
}

void PlatformControlP1::setTurretAccelerationPresetMax(int value)
{
    turretAccelerationPresetMax = value;
}

int PlatformControlP1::getTurretAccelerationPresetMin() const
{
    return turretAccelerationPresetMin;
}

void PlatformControlP1::setTurretAccelerationPresetMin(int value)
{
    turretAccelerationPresetMin = value;
}

int PlatformControlP1::getTurretDecelerationPresetCur() const
{
    return turretDecelerationPresetCur;
}

void PlatformControlP1::setTurretDecelerationPresetCur(int value)
{
    turretDecelerationPresetCur = value;
}

int PlatformControlP1::getTurretDecelerationPresetMax() const
{
    return turretDecelerationPresetMax;
}

void PlatformControlP1::setTurretDecelerationPresetMax(int value)
{
    turretDecelerationPresetMax = value;
}

int PlatformControlP1::getTurretDecelerationPresetMin() const
{
    return turretDecelerationPresetMin;
}

void PlatformControlP1::setTurretDecelerationPresetMin(int value)
{
    turretDecelerationPresetMin = value;
}

int PlatformControlP1::getTurretMotorDutyPresetCur() const
{
    return turretMotorDutyPresetCur;
}

void PlatformControlP1::setTurretMotorDutyPresetCur(int value)
{
    turretMotorDutyPresetCur = value;
}

int PlatformControlP1::getTurretMotorDutyPresetMax() const
{
    return turretMotorDutyPresetMax;
}

void PlatformControlP1::setTurretMotorDutyPresetMax(int value)
{
    turretMotorDutyPresetMax = value;
}

int PlatformControlP1::getTurretMotorDutyPresetMin() const
{
    return turretMotorDutyPresetMin;
}

void PlatformControlP1::setTurretMotorDutyPresetMin(int value)
{
    turretMotorDutyPresetMin = value;
}

int PlatformControlP1::getPlatformAccelerationPresetCur() const
{
    return platformAccelerationPresetCur;
}

void PlatformControlP1::setPlatformAccelerationPresetCur(int value)
{
    platformAccelerationPresetCur = value;
}

int PlatformControlP1::getPlatformAccelerationPresetMax() const
{
    return platformAccelerationPresetMax;
}

void PlatformControlP1::setPlatformAccelerationPresetMax(int value)
{
    platformAccelerationPresetMax = value;
}

int PlatformControlP1::getPlatformAccelerationPresetMin() const
{
    return platformAccelerationPresetMin;
}

void PlatformControlP1::setPlatformAccelerationPresetMin(int value)
{
    platformAccelerationPresetMin = value;
}

int PlatformControlP1::getPlatformDecelerationPresetCur() const
{
    return platformDecelerationPresetCur;
}

void PlatformControlP1::setPlatformDecelerationPresetCur(int value)
{
    platformDecelerationPresetCur = value;
}

int PlatformControlP1::getPlatformDecelerationPresetMax() const
{
    return platformDecelerationPresetMax;
}

void PlatformControlP1::setPlatformDecelerationPresetMax(int value)
{
    platformDecelerationPresetMax = value;
}

int PlatformControlP1::getPlatformDecelerationPresetMin() const
{
    return platformDecelerationPresetMin;
}

void PlatformControlP1::setPlatformDecelerationPresetMin(int value)
{
    platformDecelerationPresetMin = value;
}

int PlatformControlP1::getRightMotorDutyPresetCur() const
{
    return rightMotorDutyPresetCur;
}

void PlatformControlP1::setRightMotorDutyPresetCur(int value)
{
    rightMotorDutyPresetCur = value;
}

int PlatformControlP1::getRightMotorDutyPresetMax() const
{
    return rightMotorDutyPresetMax;
}

void PlatformControlP1::setRightMotorDutyPresetMax(int value)
{
    rightMotorDutyPresetMax = value;
}

int PlatformControlP1::getRightMotorDutyPresetMin() const
{
    return rightMotorDutyPresetMin;
}

void PlatformControlP1::setRightMotorDutyPresetMin(int value)
{
    rightMotorDutyPresetMin = value;
}

int PlatformControlP1::getLeftMotorDutyPresetCur() const
{
    return leftMotorDutyPresetCur;
}

void PlatformControlP1::setLeftMotorDutyPresetCur(int value)
{
    leftMotorDutyPresetCur = value;
}

int PlatformControlP1::getLeftMotorDutyPresetMax() const
{
    return leftMotorDutyPresetMax;
}

void PlatformControlP1::setLeftMotorDutyPresetMax(int value)
{
    leftMotorDutyPresetMax = value;
}

int PlatformControlP1::getLeftMotorDutyPresetMin() const
{
    return leftMotorDutyPresetMin;
}

void PlatformControlP1::setLeftMotorDutyPresetMin(int value)
{
    leftMotorDutyPresetMin = value;
}

bool PlatformControlP1::getTurretMotorDirection() const
{
    return turretMotorDirection;
}

bool PlatformControlP1::getLeftWheelEncoderAutoreset() const
{
    return leftWheelEncoderAutoreset;
}

void PlatformControlP1::setLeftWheelEncoderAutoreset(bool value)
{
    leftWheelEncoderAutoreset = value;
}

bool PlatformControlP1::getRightWheelEncoderRead() const
{
    return rightWheelEncoderRead;
}

void PlatformControlP1::setRightWheelEncoderRead(bool value)
{
    rightWheelEncoderRead = value;
}

bool PlatformControlP1::getLeftWheelEncoderRead() const
{
    return leftWheelEncoderRead;
}

void PlatformControlP1::setLeftWheelEncoderRead(bool value)
{
    leftWheelEncoderRead = value;
}

int PlatformControlP1::getRightWheelEncoder() const
{
    return rightWheelEncoder;
}

void PlatformControlP1::setRightWheelEncoder(int value)
{
    rightWheelEncoder = value;
    values["rightWheelEncoder"] = Valter::format_string("%d", rightWheelEncoder);
}

int PlatformControlP1::getLeftWheelEncoder() const
{
    return leftWheelEncoder;
}

void PlatformControlP1::setLeftWheelEncoder(int value)
{
    leftWheelEncoder = value;
    values["leftWheelEncoder"] = Valter::format_string("%d", leftWheelEncoder);
}
bool PlatformControlP1::getTurretMotorCurrentRead() const
{
    return turretMotorCurrentRead;
}

void PlatformControlP1::setTurretMotorCurrentRead(bool value)
{
    turretMotorCurrentRead = value;
}

bool PlatformControlP1::getRightMotorCurrentRead() const
{
    return rightMotorCurrentRead;
}

void PlatformControlP1::setRightMotorCurrentRead(bool value)
{
    rightMotorCurrentRead = value;
}

bool PlatformControlP1::getLeftMotorCurrentRead() const
{
    return leftMotorCurrentRead;
}

void PlatformControlP1::setLeftMotorCurrentRead(bool value)
{
    leftMotorCurrentRead = value;
}

float PlatformControlP1::getTurretMotorCurrentAmps() const
{
    return turretMotorCurrentAmps;
}

void PlatformControlP1::setTurretMotorCurrentAmps(float value)
{
    turretMotorCurrentAmps = value;
}

float PlatformControlP1::getRightMotorCurrentAmps() const
{
    return rightMotorCurrentAmps;
}

void PlatformControlP1::setRightMotorCurrentAmps(float value)
{
    rightMotorCurrentAmps = value;
}

float PlatformControlP1::getLeftMotorCurrentAmps() const
{
    return leftMotorCurrentAmps;
}

void PlatformControlP1::setLeftMotorCurrentAmps(float value)
{
    leftMotorCurrentAmps = value;
}

int PlatformControlP1::getRightMotorCurrentADC() const
{
    return rightMotorCurrentADC;
}

bool PlatformControlP1::getTurretPositionRead() const
{
    return turretPositionRead;
}

void PlatformControlP1::setTurretPositionRead(bool value)
{
    turretPositionRead = value;
}


bool PlatformControlP1::getRightWheelEncoderAutoreset() const
{
    return rightWheelEncoderAutoreset;
}

void PlatformControlP1::setRightWheelEncoderAutoreset(bool value)
{
    rightWheelEncoderAutoreset = value;
}

float PlatformControlP1::getTurretPositionRad() const
{
    return turretPositionRad;
}

void PlatformControlP1::setTurretPositionRad(float value)
{
    turretPositionRad = value;
}

float PlatformControlP1::getTurretPositionDeg() const
{
    return turretPositionDeg;
}

void PlatformControlP1::setTurretPositionDeg(float value)
{
    turretPositionDeg = value;
}

int PlatformControlP1::getTurretPositionADC() const
{
    return turretPositionADC;
}

int PlatformControlP1::getLeftAccumulatorAmperageADC() const
{
    return leftAccumulatorAmperageADC;
}

int PlatformControlP1::getBodyAmperageADC() const
{
    return bodyAmperageADC;
}

int PlatformControlP1::getPlatformAmperageADC() const
{
    return platformAmperageADC;
}

int PlatformControlP1::getMainAccumulatorAmperageTotalADC() const
{
    return mainAccumulatorAmperageTotalADC;
}

int PlatformControlP1::getRightAccumulatorVoltageADC() const
{
    return rightAccumulatorVoltageADC;
}

int PlatformControlP1::getLeftAccumulatorVoltageADC() const
{
    return leftAccumulatorVoltageADC;
}

int PlatformControlP1::getMainAccumulatorVoltageADC() const
{
    return mainAccumulatorVoltageADC;
}

bool PlatformControlP1::getTurretPositionGetOnce() const
{
    return turretPositionGetOnce;
}

bool PlatformControlP1::getRightWheelEncoderGetOnce() const
{
    return rightWheelEncoderGetOnce;
}

bool PlatformControlP1::getLeftWheelEncoderGetOnce() const
{
    return leftWheelEncoderGetOnce;
}

int PlatformControlP1::getTurretMotorCurrentADC() const
{
    return turretMotorCurrentADC;
}

int PlatformControlP1::getLeftMotorCurrentADC() const
{
    return leftMotorCurrentADC;
}

int PlatformControlP1::getMotorsPWMFrequncy() const
{
    return motorsPWMFrequncy;
}

bool PlatformControlP1::getLeftMotorActivated() const
{
    return leftMotorActivated;
}

bool PlatformControlP1::getLeftMotorStop() const
{
    return leftMotorStop;
}

bool PlatformControlP1::getLeftMotorDirection() const
{
    return leftMotorDirection;
}

bool PlatformControlP1::getChargerMode() const
{
    return chargerMode;
}

int PlatformControlP1::getLeftMotorDuty() const
{
    return leftMotorDuty;
}

void PlatformControlP1::setLeftMotorDuty(int value)
{
    leftMotorDuty = value;
    sendCommand(Valter::format_string("SETLEFTMOTORPWMDUTY#%d", leftMotorDuty));
}

void PlatformControlP1::loadDefaults()
{
    ifstream defaultsFile(Valter::filePathPrefix + PlatformControlP1::defaultsFilePath);
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

    //leftMotorDuty
    defaultValue = getDefault("leftMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setLeftMotorDutyPresetMin(minValue);
    setLeftMotorDutyPresetMax(maxValue);
    setLeftMotorDutyPresetCur(curValue);
    setLeftMotorDutyMax(getLeftMotorDutyPresetCur());

    //rightMotorDuty
    defaultValue = getDefault("rightMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setRightMotorDutyPresetMin(minValue);
    setRightMotorDutyPresetMax(maxValue);
    setRightMotorDutyPresetCur(curValue);
    setRightMotorDutyMax(getRightMotorDutyPresetCur());

    //motorsDeceleration
    defaultValue = getDefault("motorsDeceleration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setPlatformDecelerationPresetMin(minValue);
    setPlatformDecelerationPresetMax(maxValue);
    setPlatformDecelerationPresetCur(curValue);
    setPlatformDeceleration(getPlatformDecelerationPresetCur());

    //motorsAcceleration
    defaultValue = getDefault("motorsAcceleration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setPlatformAccelerationPresetMin(minValue);
    setPlatformAccelerationPresetMax(maxValue);
    setPlatformAccelerationPresetCur(curValue);
    setPlatformAcceleration(getPlatformAccelerationPresetCur());

    //turretMotorDuty
    defaultValue = getDefault("turretMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setTurretMotorDutyPresetMin(minValue);
    setTurretMotorDutyPresetMax(maxValue);
    setTurretMotorDutyPresetCur(curValue);
    setTurretMotorDutyMax(getTurretMotorDutyPresetCur());

    //turretMotorDeceleration
    defaultValue = getDefault("turretMotorDeceleration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setTurretDecelerationPresetMin(minValue);
    setTurretDecelerationPresetMax(maxValue);
    setTurretDecelerationPresetCur(curValue);
    setTurretDeceleration(getTurretDecelerationPresetCur());

    //turretMotorAcceleration
    defaultValue = getDefault("turretMotorAcceleration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setTurretAccelerationPresetMin(minValue);
    setTurretAccelerationPresetMax(maxValue);
    setTurretAccelerationPresetCur(curValue);
    setTurretAcceleration(getTurretAccelerationPresetCur());

    //trackLeftMotorCurrent
    defaultValue = getDefault("trackLeftMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setLeftMotorCurrentRead((atoi(defaultValuePtr)) ? true : false);

    //trackRightMotorCurrent
    defaultValue = getDefault("trackRightMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setRightMotorCurrentRead((atoi(defaultValuePtr)) ? true : false);

    //trackTurretMotorCurrent
    defaultValue = getDefault("trackTurretMotorCurrent");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setTurretMotorCurrentRead((atoi(defaultValuePtr)) ? true : false);

    //trackLeftWheelEncoder
    defaultValue = getDefault("trackLeftWheelEncoder");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setLeftWheelEncoderRead((atoi(defaultValuePtr)) ? true : false);

    //trackRightWheelEncoder
    defaultValue = getDefault("trackRightWheelEncoder");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setRightWheelEncoderRead((atoi(defaultValuePtr)) ? true : false);

    //leftWheelEncoderAutoreset
    defaultValue = getDefault("leftWheelEncoderAutoreset");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setLeftWheelEncoderAutoreset((atoi(defaultValuePtr)) ? true : false);

    //rightWheelEncoderAutoreset
    defaultValue = getDefault("rightWheelEncoderAutoreset");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setRightWheelEncoderAutoreset((atoi(defaultValuePtr)) ? true : false);

    //trackTurretPosition
    defaultValue = getDefault("trackTurretPosition");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    setTurretPositionRead((atoi(defaultValuePtr)) ? true : false);

    //platformControlP1ReadingsTable
    defaultValue = getDefault("platformControlP1ReadingsTable");
    vector<string>defaultValue_str_values = Valter::split(defaultValue, ',');
    vector<string>::iterator iter = defaultValue_str_values.begin();

    int idx = 1;
    while( iter != defaultValue_str_values.end() )
    {
        string val = *iter++;
        defaultValuePtr = Valter::stringToCharPtr(val);
        switch (idx)
        {
            case 1:
                setMainAccumulatorVoltageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 2:
                setLeftAccumulatorVoltageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 3:
                setRightAccumulatorVoltageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 4:
                setMainAccumulatorAmperageTotalRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 5:
                setPlatformAmperageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 6:
                setBodyAmperageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 7:
                setLeftAccumulatorAmperageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 8:
                setRightAccumulatorAmperageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 9:
                setChargerVoltageRead((atoi(defaultValuePtr)) ? true : false);
            break;
            case 10:
                setMainAccumulatorVoltageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 11:
                setLeftAccumulatorVoltageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 12:
                setRightAccumulatorVoltageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 13:
                setMainAccumulatorAmperageTotalReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 14:
                setPlatformAmperageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 15:
                setBodyAmperageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 16:
                setLeftAccumulatorAmperageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 17:
                setRightAccumulatorAmperageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
            case 18:
                setChargerVoltageReadADCPreset((atoi(defaultValuePtr)) ? true : false);
            break;
        }

        idx++;
    }
    //additionalReadingsScanDelay
    defaultValue = getDefault("additionalReadingsScanDelay");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setAdditionalReadingsDelayPresetMin(minValue);
    setAdditionalReadingsDelayPresetMax(maxValue);
    setAdditionalReadingsDelayCur(curValue);
}
