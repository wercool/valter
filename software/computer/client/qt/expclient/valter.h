#ifndef VALTER_H
#define VALTER_H

#include <string>

using namespace std;

class Valter
{
public:
   static Valter* getInstance();
   string getVersion();
   void listControlDevices(bool fullInfo = false);

private:
   Valter();
   static Valter* pValter;		// Valter's main singleton instance
   static bool instanceFlag;
};


#endif // VALTER_H
