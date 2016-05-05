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
    TCPInterface(int port);

    string getIp() const;
    void setIp(const string &value);
    void readIP();

    int getPort() const;
    void setPort(int value);

    Thread *getConnectionHandler() const;
    void setConnectionHandler(Thread *value);

    void startListening();

    bool getListening() const;
    void setListening(bool value);

    wqueue<WorkItem*> queue;

private:
    string ip;
    int port;
    Thread *connectionHandler;
    TCPAcceptor *connectionAcceptor;

    bool listening;
    void tcpConnectionWorker();
};

#endif // TCPINTERFACE_H
