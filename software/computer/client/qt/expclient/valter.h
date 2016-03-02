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

   vector<ControlDevice> getControlDevices() const;
   void setControlDevices(const vector<ControlDevice> &value);

   std::vector<std::string> controlDeviceIds;

private:
   Valter();
   static Valter* pValter;		// Valter's singleton instance
   static bool instanceFlag;

   vector<ControlDevice> controlDevices;
};


#endif // VALTER_H
