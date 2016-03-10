#ifndef VALTER_H
#define VALTER_H

#include "mainwindow.h"

#include <fstream>
#include <algorithm>
#include <memory>
#include <stdarg.h>
#include <ctime>

#include "controldevice.h"
#include "ivaltermodule.h"
#include "platformcontrolp1.h"
#include "platformcontrolp2.h"
#include "platformlocationp1.h"
#include "platformmanipulatorandirbumper.h"
#include "bodycontrolp1.h"

using namespace std;
using std::ifstream;
using std::copy;

class Valter
{
public:
   static Valter *getInstance();
   static string getVersion();
   static void log(string msg);
   static void delayGUIAction(string msg);

   void readControlDevicesCommandsFromFiles(bool printCommands = false);
   void scanControlDevices();
   vector<string> controlDeviceIds;
   map<string, vector<string>> controlDevicesCommands;
   map<string, vector<string>> getControlDevicesCommands();
   void addControlDevice(string controlDeviceId, string port);
   void addControlDeviceToControlDevicesMap(ControlDevice* controlDevice);
   void updateControlDevice(string controlDeviceId, string port);
   ControlDevice* getControlDeviceById(string controlDeviceId);
   void closeAllControlDevicePorts();
   map<string, ControlDevice *> getControlDevicesMap() const;
   void setControlDevicesMap(const map<string, ControlDevice *> &value);
   void clearControlDevicesMap();
   bool getLogControlDeviceMessages() const;
   void setLogControlDeviceMessages(bool value);

   map<string, IValterModule *> getValterModulesMap() const;
   void setValterModulesMap(const map<string, IValterModule *> &value);
   void addUpdateValterModule(string controlDeviceId, IValterModule *valterModule);
   IValterModule *getValterModule(string controlDeviceId);

   void stopAllModules();

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
   static std::string exec_shell(string cmd)
   {
       char * writableCmd = new char[cmd.size() + 1];
       std::copy(cmd.begin(), cmd.end(), writableCmd);
       writableCmd[cmd.size()] = '\0';

       std::shared_ptr<FILE> pipe(popen(writableCmd, "r"), pclose);
       if (!pipe) return "ERROR";
       char buffer[128];
       std::string result = "";
       while (!feof(pipe.get()))
       {
           if (fgets(buffer, 128, pipe.get()) != NULL)
               result += buffer;
       }
       return result;
   }
   static char* stringToCharPtr(string str)
   {
       char *charPtr = new char[str.size() + 1];
       std::copy(str.begin(), str.end(), charPtr);
       charPtr[str.size()] = '\0';
       return charPtr;
   }

private:
   Valter();
   static Valter *pValter;      // Valter's singleton instance
   static bool instanceFlag;
   static const bool logToGUI = true;
   static const bool logToConsole = true;
   bool logControlDeviceMessages;
   static const string cmdFilesPath;
   map<string, ControlDevice*> controlDevicesMap;

   map<string, IValterModule*> valterModulesMap;

};



#endif // VALTER_H
