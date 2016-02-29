#include "valter.h"
#include "controldevice.h"

Valter* Valter::pValter = NULL;
bool Valter::instanceFlag = false;

Valter::Valter()
{

}

Valter* Valter::getInstance()
{
    if(! instanceFlag)
    {
        pValter = new Valter();
        instanceFlag = true;
        return pValter;
    }
    else
    {
        return pValter;
    }
}

string Valter::getVersion()
{
    return "0.0.1";
}

void Valter::listControlDevices(bool fullInfo)
{
    ControlDevice::listDevices(fullInfo);
}
