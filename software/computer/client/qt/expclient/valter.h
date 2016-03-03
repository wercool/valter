#ifndef VALTER_H
#define VALTER_H

#include "mainwindow.h"

#include <fstream>
#include <algorithm>
#include <memory>
#include <stdarg.h>
#include <ctime>

#include "controldevice.h"

using namespace std;
using std::ifstream;
using std::copy;

class Valter
{
public:
   static Valter* getInstance();
   static string getVersion();
   static void log(string msg);

   void readControlDevicesCommandsFromFiles(bool printCommands = false);
   void scanControlDevices();
   vector<string> controlDeviceIds;
   map<string, vector<string>> controlDevicesCommands;
   map<string, vector<string>> getControlDevicesCommands();
   void addControlDevice(string controlDeviceId, string port);
   void addControlDeviceToControlDevicesMap(ControlDevice* controlDevice);
   ControlDevice* getControlDeviceById(string controlDeviceId);
   void closeAllControlDevicePorts();
   map<string, ControlDevice *> getControlDevicesMap() const;
   void setControlDevicesMap(const map<string, ControlDevice *> &value);
   bool getLogControlDeviceMessages() const;
   void setLogControlDeviceMessages(bool value);

   //Utils
   static std::string format_string(const std::string fmt_str, ...)
   {
       int final_n, n = ((int)fmt_str.size()) * 2; /* Reserve two times as much as the length of the fmt_str */
       std::string str;
       std::unique_ptr<char[]> formatted;
       va_list ap;
       while(1)
       {
           formatted.reset(new char[n]); /* Wrap the plain char array into the unique_ptr */
           strcpy(&formatted[0], fmt_str.c_str());
           va_start(ap, fmt_str);
           final_n = vsnprintf(&formatted[0], n, fmt_str.c_str(), ap);
           va_end(ap);
           if (final_n < 0 || final_n >= n)
               n += abs(final_n - n + 1);
           else
               break;
       }
       return std::string(formatted.get());
   }


private:
   Valter();
   static Valter* pValter;		// Valter's singleton instance
   static bool instanceFlag;
   static const bool logToGUI = true;
   bool logControlDeviceMessages;
   static const string cmdFilesPath;
   map<string, ControlDevice*> controlDevicesMap;
};


#endif // VALTER_H
