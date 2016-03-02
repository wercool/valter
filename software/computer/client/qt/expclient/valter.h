#ifndef VALTER_H
#define VALTER_H

#include <fstream>
using std::ifstream;
#include <algorithm>
using std::copy;

#include "controldevice.h"

using namespace std;

class Valter
{
public:
   static Valter* getInstance();
   static string getVersion();
   void readControlDevicesCommandsFromFiles();

   void scanControlDevices();

   vector<string> controlDeviceIds;
   map<string, vector<string>> controlDevicesCommands;

   void addControlDevice(string controlDeviceId, string port);
   void addControlDeviceToControlDevicesMap(ControlDevice* controlDevice);
   ControlDevice* getControlDeviceById(string controlDeviceId);
   void closeAllControlDevicePorts();

   map<string, ControlDevice *> getControlDevicesMap() const;
   void setControlDevicesMap(const map<string, ControlDevice *> &value);

private:
   Valter();
   static Valter* pValter;		// Valter's singleton instance
   static bool instanceFlag;
   map<string, ControlDevice*> controlDevicesMap;

   const string cmdFilesPath = "/home/maska/git/valter/software/computer/client/qt/expclient/resources/commands/";
};


#endif // VALTER_H
