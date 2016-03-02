#ifndef VALTER_H
#define VALTER_H

#include "controldevice.h"

using namespace std;

class Valter
{
public:
   static Valter* getInstance();
   string getVersion();

   void listControlDevices(bool fullInfo = false);
   void scanControlDevices();

   vector<string> controlDeviceIds;

   void addControlDevice(string controlDeviceId, string port);
   void addControlDeviceToControlDevicesMap(ControlDevice* controlDevice);
   ControlDevice* getControlDeviceById(string controlDeviceId);

   map<string, ControlDevice *> getControlDevicesMap() const;
   void setControlDevicesMap(const map<string, ControlDevice *> &value);

private:
   Valter();
   static Valter* pValter;		// Valter's singleton instance
   static bool instanceFlag;
   map<string, ControlDevice*> controlDevicesMap;
};


#endif // VALTER_H
