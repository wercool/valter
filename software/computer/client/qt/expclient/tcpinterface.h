#ifndef TCPINTERFACE_H
#define TCPINTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "tcp/threads/thread.h"
#include "tcp/wqueue/wqueue.h"
#include "tcp/tcpsockets/tcpacceptor.h"
#include "tcp/tcpsockets/tcpconnector.h"
#include "tcp/tcpsockets/tcpstream.h"
#include "tcp/tcpsockets/workitem.h"

class TCPInterface
{

public:
    TCPInterface(int port);

    string getIp() const;
    void setIp(const string &value);
    void readIP();

    static string getLocalHostIP();

    int getPort() const;
    void setPort(int value);

    Thread *getConnectionHandler() const;
    void setConnectionHandler(Thread *value);

    void startListening();

    bool getListening() const;
    void setListening(bool value);

    wqueue<WorkItem*> queue;

    TCPConnector *getCommanfInterfaceConnector() const;
    void setCommanfInterfaceConnector(TCPConnector *value);

    TCPStream *getCommanfInterfaceStream() const;
    void setCommanfInterfaceStream(TCPStream *value);

    bool sendCommandMessage(string command);

    string getCommandHostIP() const;
    void setCommandHostIP(const string &value);

    int getCommandHostPort() const;
    void setCommandHostPort(int value);

    //central command host
    string getCentralCommandHostIP() const;
    void setCentralCommandHostIP(const string &value);

    bool sendCDRToCentralCommandHost(string command);

    int getCentralCommandHostIPPort() const;
    void setCentralCommandHostIPPort(const int &value);

    bool getConnected() const;
    void setConnected(bool value);

private:
    string ip;
    int port;
    Thread *connectionHandler;
    TCPAcceptor *connectionAcceptor;
    TCPConnector *commanfInterfaceConnector;
    TCPStream *commanfInterfaceStream;

    bool listening;
    void tcpConnectionWorker();

    //command host
    string commandHostIP;
    int commandHostPort;

    //central command host
    string centralCommandHostIP;
    int centralCommandHostIPPort;

    bool connected;
};

#endif // TCPINTERFACE_H
