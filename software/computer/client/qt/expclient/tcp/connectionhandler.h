#ifndef CONNECTIONHANDLER_H
#define CONNECTIONHANDLER_H

#include "tcpinterface.h"

class ConnectionHandler: Thread
{
public:
    virtual void *run() = 0;
};

#endif // CONNECTIONHANDLER_H
