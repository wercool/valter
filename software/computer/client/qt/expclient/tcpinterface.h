#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "tcp/threads/thread.h"
#include "tcp/wqueue/wqueue.h"
#include "tcp/tcpsockets/tcpacceptor.h"
#include "tcp/tcpsockets/workitem.h"

class TCPInterface
{
public:
    TCPInterface();
    Thread *connectionHandler;

    string getIp() const;
    void setIp(const string &value);

private:
    string ip;

};

#endif // TCPINTERFACE_H
