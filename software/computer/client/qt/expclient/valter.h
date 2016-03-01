#ifndef VALTER_H
#define VALTER_H

#include "controldevice.h"

using namespace std;

class Valter
{
public:
   static Valter* getInstance();
   string getVersion();

   const std::vector<std::string> ControlDeviceIds{"BODY-CONTROL-P1"};

   void listControlDevices(bool fullInfo = false);
   void scanControlDevices();

   vector<ControlDevice> getControlDevices() const;
   void setControlDevices(const vector<ControlDevice> &value);



private:
   Valter();
   static Valter* pValter;		// Valter's main singleton instance
   static bool instanceFlag;

   vector<ControlDevice> controlDevices;
};


#endif // VALTER_H
