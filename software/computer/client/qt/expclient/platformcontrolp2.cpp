#include <string>
#include <iostream>
#include <cstdio>

#include "valter.h"
#include "platformcontrolp2.h"

PlatformControlP2 *PlatformControlP2::pPlatformControlP2 = NULL;
bool PlatformControlP2::instanceFlag = false;
const string PlatformControlP2::controlDeviceId = "PLATFORM-CONTROL-P2";
const string PlatformControlP2::defaultsFilePath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/settings/platform-control-p2-defaults";

PlatformControlP2::PlatformControlP2()
{
    Valter::log(PlatformControlP2::controlDeviceId + " singleton started");
    this->controlDeviceIsSet = false;
}

int PlatformControlP2::getChargerMotorDuty() const
{
    return chargerMotorDuty;
}

void PlatformControlP2::setChargerMotorDuty(int value)
{
    chargerMotorDuty = value;
}

void PlatformControlP2::encodersWorker()
{

}

void PlatformControlP2::irScanningWorker()
{

}

bool PlatformControlP2::getBottomRearLeds() const
{
    return bottomRearLeds;
}

void PlatformControlP2::setBottomRearLeds(bool value)
{
    bottomRearLeds = value;
}

bool PlatformControlP2::getBottomFrontLeds() const
{
    return bottomFrontLeds;
}

void PlatformControlP2::setBottomFrontLeds(bool value)
{
    bottomFrontLeds = value;
}

bool PlatformControlP2::getChargerLeds() const
{
    return chargerLeds;
}

void PlatformControlP2::setChargerLeds(bool value)
{
    chargerLeds = value;
}

int PlatformControlP2::getBeepDurationPresetMax() const
{
    return beepDurationPresetMax;
}

void PlatformControlP2::setBeepDurationPresetMax(int value)
{
    beepDurationPresetMax = value;
}

int PlatformControlP2::getBeepDurationPresetMin() const
{
    return beepDurationPresetMin;
}

void PlatformControlP2::setBeepDurationPresetMin(int value)
{
    beepDurationPresetMin = value;
}

int PlatformControlP2::getBeepDuration() const
{
    return beepDuration;
}

void PlatformControlP2::setBeepDuration(int value)
{
    beepDuration = value;
}

bool PlatformControlP2::getIrScanningWorkerActivated() const
{
    return irScanningWorkerActivated;
}

void PlatformControlP2::setIrScanningWorkerActivated(bool value)
{
    irScanningWorkerActivated = value;
}

void PlatformControlP2::spawnEncodersWorker()
{

}

void PlatformControlP2::spawnIRScanningWorker()
{

}

bool PlatformControlP2::getEncodersWorkerActivated() const
{
    return encodersWorkerActivated;
}

void PlatformControlP2::setEncodersWorkerActivated(bool value)
{
    encodersWorkerActivated = value;
}

int PlatformControlP2::getChargerMotorPushDuration() const
{
    return chargerMotorPushDuration;
}

void PlatformControlP2::setChargerMotorPushDuration(int value)
{
    chargerMotorPushDuration = value;
}

int PlatformControlP2::getChargerMotorPushDurationPresetMax() const
{
    return chargerMotorPushDurationPresetMax;
}

void PlatformControlP2::setChargerMotorPushDurationPresetMax(int value)
{
    chargerMotorPushDurationPresetMax = value;
}

int PlatformControlP2::getChargerMotorPushDurationPresetMin() const
{
    return chargerMotorPushDurationPresetMin;
}

void PlatformControlP2::setChargerMotorPushDurationPresetMin(int value)
{
    chargerMotorPushDurationPresetMin = value;
}

int PlatformControlP2::getChargerMotorDutyPresetCur() const
{
    return chargerMotorDutyPresetCur;
}

void PlatformControlP2::setChargerMotorDutyPresetCur(int value)
{
    chargerMotorDutyPresetCur = value;
}

int PlatformControlP2::getChargerMotorDutyPresetMax() const
{
    return chargerMotorDutyPresetMax;
}

void PlatformControlP2::setChargerMotorDutyPresetMax(int value)
{
    chargerMotorDutyPresetMax = value;
}

int PlatformControlP2::getChargerMotorDutyPresetMin() const
{
    return chargerMotorDutyPresetMin;
}

void PlatformControlP2::setChargerMotorDutyPresetMin(int value)
{
    chargerMotorDutyPresetMin = value;
}

bool PlatformControlP2::getRightEncoderAutoreset() const
{
    return rightEncoderAutoreset;
}

void PlatformControlP2::setRightEncoderAutoreset(bool value)
{
    rightEncoderAutoreset = value;
}

bool PlatformControlP2::getLeftEncoderAutoreset() const
{
    return leftEncoderAutoreset;
}

void PlatformControlP2::setLeftEncoderAutoreset(bool value)
{
    leftEncoderAutoreset = value;
}

int PlatformControlP2::getRightEncoder() const
{
    return rightEncoder;
}

void PlatformControlP2::setRightEncoder(int value)
{
    rightEncoder = value;
}

int PlatformControlP2::getLeftEncoder() const
{
    return leftEncoder;
}

void PlatformControlP2::setLeftEncoder(int value)
{
    leftEncoder = value;
}

bool PlatformControlP2::getReadRightEncoder() const
{
    return readRightEncoder;
}

void PlatformControlP2::setReadRightEncoder(bool value)
{
    readRightEncoder = value;
}

bool PlatformControlP2::getReadLeftEncoder() const
{
    return readLeftEncoder;
}

void PlatformControlP2::setReadLeftEncoder(bool value)
{
    readLeftEncoder = value;
}

string PlatformControlP2::getControlDeviceId()
{
    return controlDeviceId;
}


PlatformControlP2 *PlatformControlP2::getInstance()
{
    if(!instanceFlag)
    {
        pPlatformControlP2 = new PlatformControlP2();
        instanceFlag = true;
        return pPlatformControlP2;
    }
    else
    {
        return pPlatformControlP2;
    }
}

void PlatformControlP2::stopAll()
{
    if (this->controlDeviceIsSet)
    {
        Valter::log(Valter::format_string("STOP ALL sent to %s", PlatformControlP2::controlDeviceId.c_str()));
    }
}

void PlatformControlP2::resetToDefault()
{
    if (this->controlDeviceIsSet)
    {
        getControlDevice()->setAutoReActivation(true);
        getControlDevice()->clearMessageQueue();
        getControlDevice()->clearDataExchangeLog();
    }

    readLeftEncoder         = false;
    readRightEncoder        = false;
    leftEncoder             = 0;
    rightEncoder            = 0;
    leftEncoderAutoreset    = false;
    rightEncoderAutoreset   = false;

    chargerMotorDuty            = 1;
    chargerMotorDutyPresetMin   = 1;
    chargerMotorDutyPresetMax   = 100;
    chargerMotorDutyPresetCur   = 50;

    chargerMotorPushDuration            = 250;
    chargerMotorPushDurationPresetMin   = 50;
    chargerMotorPushDurationPresetMax   = 500;

    encodersWorkerActivated     = false;
    irScanningWorkerActivated   = false;


    beepDuration                = 5;
    beepDurationPresetMin       = 1;
    beepDurationPresetMax       = 200;

    chargerLeds         = false;
    bottomFrontLeds     = false;
    bottomRearLeds      = false;
}

void PlatformControlP2::spawnProcessMessagesQueueWorkerThread()
{
    //setProcessMessagesQueueWorkerThread(new std::thread(&PlatformControlP2::processMessagesQueueWorker, this));
}

void PlatformControlP2::loadDefaults()
{
    ifstream defaultsFile(PlatformControlP2::defaultsFilePath);
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

    //trackLeftWheelEncoder
    defaultValue = getDefault("trackLeftWheelEncoder");
    setReadLeftEncoder((defaultValue.compare("1") == 0) ? true : false);

    //trackRightWheelEncoder
    defaultValue = getDefault("trackRightWheelEncoder");
    setReadRightEncoder((defaultValue.compare("1") == 0) ? true : false);

    //autoresetLeftWheelEncoder
    defaultValue = getDefault("autoresetLeftWheelEncoder");
    setLeftEncoderAutoreset((defaultValue.compare("1") == 0) ? true : false);

    //autoresetRightWheelEncoder
    defaultValue = getDefault("autoresetRightWheelEncoder");
    setRightEncoderAutoreset((defaultValue.compare("1") == 0) ? true : false);

    //chargerMotorDuty
    defaultValue = getDefault("chargerMotorDuty");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setChargerMotorDutyPresetMin(minValue);
    setChargerMotorDutyPresetMax(maxValue);
    setChargerMotorDutyPresetCur(curValue);

    //chargerMotorPushDuration
    defaultValue = getDefault("chargerMotorPushDuration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setChargerMotorPushDurationPresetMin(minValue);
    setChargerMotorPushDurationPresetMax(maxValue);
    setChargerMotorPushDuration(curValue);

    //irScanning
    defaultValue = getDefault("autoresetRightWheelEncoder");
    setIrScanningWorkerActivated((defaultValue.compare("1") == 0) ? true : false);

    //beepDuration
    defaultValue = getDefault("beepDuration");
    defaultValuePtr = Valter::stringToCharPtr(defaultValue);
    minValue = atoi(Valter::stringToCharPtr(strtok(defaultValuePtr, "," )));
    maxValue = atoi(strtok(NULL, "," ));
    curValue = atoi(strtok(NULL, "," ));
    setBeepDurationPresetMin(minValue);
    setBeepDurationPresetMax(maxValue);
    setBeepDuration(curValue);

    //chargerLeds
    defaultValue = getDefault("chargerLeds");
    setChargerLeds((defaultValue.compare("1") == 0) ? true : false);

    //bottomFrontLeds
    defaultValue = getDefault("bottomFrontLeds");
    setBottomFrontLeds((defaultValue.compare("1") == 0) ? true : false);

    //bottomRearLeds
    defaultValue = getDefault("bottomRearLeds");
    setBottomRearLeds((defaultValue.compare("1") == 0) ? true : false);
}

void PlatformControlP2::setModuleInitialState()
{

}
