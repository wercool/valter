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

    connectionAcceptor = new TCPAcceptor(port, (char*)ip.c_str());

    if (!connectionAcceptor || connectionAcceptor->start() != 0)
    {
        Valter::log(Valter::format_string("Could not create an connection acceptor on the %s:%d", ip.c_str(), port));
    }
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
            if (strcmp(ifa->ifa_name, "enp3s0") == 0)
            {
                std::string str(addr);
                ip = str;
            }
        }
    }

    freeifaddrs(ifap);
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
        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
