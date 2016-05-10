#include "valter.h"
#include "tcpinterface.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>

TCPInterface::TCPInterface(int port)
{
    setPort(port);
    readIP();
    setListening(false);
    setCommanfInterfaceConnector(new TCPConnector());

    //set uninitialized from the beginning
    centralCommandHostIP = "";
}

string TCPInterface::getIp() const
{
    return ip;
}

void TCPInterface::setIp(const string &value)
{
    ip = value;
}

void TCPInterface::readIP()
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;

    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr->sa_family == AF_INET)
        {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            addr = inet_ntoa(sa->sin_addr);
            qDebug("Interface: %s\tAddress: %s", ifa->ifa_name, addr);
            if (strcmp(ifa->ifa_name, "enp3s0") == 0 || strcmp(ifa->ifa_name, "eth0") == 0)
            {
                std::string str(addr);
                ip = str;
            }
        }
    }

    freeifaddrs(ifap);
}

string TCPInterface::getLocalHostIP()
{
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    char *addr;

    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr->sa_family == AF_INET)
        {
            sa = (struct sockaddr_in *) ifa->ifa_addr;
            addr = inet_ntoa(sa->sin_addr);
            qDebug("Interface: %s\tAddress: %s", ifa->ifa_name, addr);
            if (strcmp(ifa->ifa_name, "enp3s0") == 0 || strcmp(ifa->ifa_name, "eth0") == 0)
            {
                std::string localhostIPAddress(addr);
                freeifaddrs(ifap);
                return localhostIPAddress;
            }
        }
    }
    return "";
}

int TCPInterface::getPort() const
{
    return port;
}

void TCPInterface::setPort(int value)
{
    port = value;
}

Thread *TCPInterface::getConnectionHandler() const
{
    return connectionHandler;
}

void TCPInterface::setConnectionHandler(Thread *value)
{
    connectionHandler = value;
}

void TCPInterface::startListening()
{
    if (!getConnectionHandler())
    {
        Valter::log(Valter::format_string("Could not create ConnectionHandler for connection %s:%d", ip.c_str(), port));
    }
    else
    {
        getConnectionHandler()->start();
        setListening(true);
        new std::thread(&TCPInterface::tcpConnectionWorker, this);
    }
}

bool TCPInterface::getListening() const
{
    return listening;
}

void TCPInterface::setListening(bool value)
{
    listening = value;

    if (listening)
    {
        connectionAcceptor = new TCPAcceptor(port, (char*)ip.c_str());

        if (!connectionAcceptor || connectionAcceptor->start() != 0)
        {
            Valter::log(Valter::format_string("Could not create an connection acceptor on the %s:%d", ip.c_str(), port));
        }
    }
}

TCPConnector *TCPInterface::getCommanfInterfaceConnector() const
{
    return commanfInterfaceConnector;
}

void TCPInterface::setCommanfInterfaceConnector(TCPConnector *value)
{
    commanfInterfaceConnector = value;
}

TCPStream *TCPInterface::getCommanfInterfaceStream() const
{
    return commanfInterfaceStream;
}

void TCPInterface::setCommanfInterfaceStream(TCPStream *value)
{
    commanfInterfaceStream = value;
}

bool TCPInterface::sendCommandMessage(string command)
{
    TCPStream* stream = getCommanfInterfaceConnector()->connect(getCommandHostIP().c_str(), getCommandHostPort());
    int length;
    char response[256];
    if (stream)
    {
        //qDebug("sent - %s", command.c_str());
        stream->send(command.c_str(), command.size());
        length = stream->receive(response, sizeof(response));
        response[length] = '\0';
        qDebug("received - %s", response);
        delete stream;

        return true;
    }
    return false;
}

void TCPInterface::tcpConnectionWorker()
{
    WorkItem* item;
    while (getListening())
    {
        TCPStream* connection = connectionAcceptor->accept();
        item = new WorkItem(connection);
        if (!item)
        {
            Valter::log(Valter::format_string("Could not create work item for connection %s:%d", ip.c_str(), port));
            continue;
        }
        queue.add(item);
        this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int TCPInterface::getCentralCommandHostIPPort() const
{
    return centralCommandHostIPPort;
}

void TCPInterface::setCentralCommandHostIPPort(const int &value)
{
    centralCommandHostIPPort = value;
}

string TCPInterface::getCentralCommandHostIP() const
{
    return centralCommandHostIP;
}

void TCPInterface::setCentralCommandHostIP(const string &value)
{
    centralCommandHostIP = value;
}

bool TCPInterface::sendCDRToCentralCommandHost(string command)
{
    if (getCentralCommandHostIP().compare("") > 0)
    {
        TCPStream* stream = getCommanfInterfaceConnector()->connect(getCentralCommandHostIP().c_str(), getCommandHostPort());
        int length;
        char response[256];
        if (stream)
        {
            //qDebug("sent - %s", command.c_str());
            stream->send(command.c_str(), command.size());
            length = stream->receive(response, sizeof(response));
            response[length] = '\0';
            qDebug("received - %s", response);
            delete stream;

            return true;
        }
    }
    return false;
}

int TCPInterface::getCommandHostPort() const
{
    return commandHostPort;
}

void TCPInterface::setCommandHostPort(int value)
{
    commandHostPort = value;
}

string TCPInterface::getCommandHostIP() const
{
    return commandHostIP;
}

void TCPInterface::setCommandHostIP(const string &value)
{
    commandHostIP = value;
}
