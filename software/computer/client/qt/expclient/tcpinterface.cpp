#include "valter.h"
#include "tcpinterface.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <algorithm>

string TCPInterface::defaultNetworkInterface = "";

TCPInterface::TCPInterface(int port)
{
    setPort(port);
    readIP();
    setListening(false);
    setCommandInterfaceConnector(new TCPConnector());

    //set uninitialized from the beginning
    connected = false;
    centralCommandHostIP = "";

    sentCDRs = {};

    sentCDRsRectifierThreadWorkerStopped = false;

    sentCDRsRectifierThread = new std::thread(&TCPInterface::sentCDRsRectifierThreadWorker, this);
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
            //if (strcmp(ifa->ifa_name, "enp3s0") == 0 || strcmp(ifa->ifa_name, "eth0") == 0 || strcmp(ifa->ifa_name, "wlan0") == 0)
            if (TCPInterface::defaultNetworkInterface.compare("") == 0)
            {
                if (strcmp(ifa->ifa_name, "eth0") == 0)
                {
                    std::string str(addr);
                    ip = str;
                }
            }
            else
            {
                qDebug("Selected Network Interface: %s", TCPInterface::defaultNetworkInterface.c_str());
                if (strcmp(ifa->ifa_name, TCPInterface::defaultNetworkInterface.c_str()) == 0)
                {
                    std::string str(addr);
                    ip = str;
                }
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
            //if (strcmp(ifa->ifa_name, "enp3s0") == 0 || strcmp(ifa->ifa_name, "eth0") == 0 || strcmp(ifa->ifa_name, "enp0s25") == 0 || strcmp(ifa->ifa_name, "wlan0") == 0)
            if (TCPInterface::defaultNetworkInterface.compare("") == 0)
            {
                if (strcmp(ifa->ifa_name, "eth0") == 0)
                {
                    std::string localhostIPAddress(addr);
                    freeifaddrs(ifap);
                    return localhostIPAddress;
                }
                else
                {
                    qDebug("!!!!!!!!%s NETWORK INTERFACE IS NOT PROCESSED!!!!!", ifa->ifa_name);
                }
            }
            else
            {
                if (strcmp(ifa->ifa_name, TCPInterface::defaultNetworkInterface.c_str()) == 0)
                {
                    qDebug("(static call) getLocalHostIP() Selected Network Interface: %s", TCPInterface::defaultNetworkInterface.c_str());
                    std::string localhostIPAddress(addr);
                    freeifaddrs(ifap);
                    return localhostIPAddress;
                }
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

TCPConnector *TCPInterface::getCommandInterfaceConnector() const
{
    return commandInterfaceConnector;
}

void TCPInterface::setCommandInterfaceConnector(TCPConnector *value)
{
    commandInterfaceConnector = value;
}

TCPStream *TCPInterface::getCommandInterfaceStream() const
{
    return commandInterfaceStream;
}

void TCPInterface::setCommandInterfaceStream(TCPStream *value)
{
    commandInterfaceStream = value;
}

bool TCPInterface::sendCommandMessage(string command)
{
    try
    {
        TCPStream* stream = getCommandInterfaceConnector()->connect(getCommandHostIP().c_str(), getCommandHostPort());
        int length;
        char response[256];
        if (stream)
        {
            //qDebug("sent - %s", command.c_str());
            stream->send(command.c_str(), command.size());
            length = stream->receive(response, sizeof(response));
            response[length] = '\0';
            //qDebug("received - %s", response);
            delete stream;

            return true;
        }
    }
    catch (...)
    {
        return false;
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
    qDebug("STOPPED: TCPInterface::tcpConnectionWorker");
}

void TCPInterface::setSentCDRsRectifierThreadWorkerStopped(bool value)
{
    sentCDRsRectifierThreadWorkerStopped = value;
}

bool TCPInterface::getConnected() const
{
    return connected;
}

void TCPInterface::setConnected(bool value)
{
    connected = value;
}

TCPAcceptor *TCPInterface::getConnectionAcceptor() const
{
    return connectionAcceptor;
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
    if (connected)
    {
        if (getCentralCommandHostIP().compare("") > 0)
        {
            bool duplicate = false;
            if (!sentCDRs.empty())
            {
                for(vector<string>::const_iterator i = sentCDRs.begin(); i != sentCDRs.end(); ++i)
                {
                    if (((string)*i).compare(command) == 0)
                    {
                        duplicate = true;
                    }
                }
            }
            if (!duplicate) //ignore too often duplicates
            {
//                qDebug("CDR to be sent: %s", command.c_str());
                std::lock_guard<std::mutex> guard(sentCDRs_mutex);
                sentCDRs.push_back(command);
                try
                {
                    TCPStream* stream = getCommandInterfaceConnector()->connect(getCentralCommandHostIP().c_str(), getCentralCommandHostIPPort());
                    int length;
                    char response[256];
                    if (stream)
                    {
                        //qDebug("sent - %s", command.c_str());
                        stream->send(command.c_str(), command.size());
                        length = stream->receive(response, sizeof(response));
                        response[length] = '\0';
                        //qDebug("received - %s", response);
                        delete stream;

                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                catch (...)
                {
                    return false;
                }
            }
            else
            {
                qDebug("Duplicate CDR detected within 100ms [%s]", command.c_str());
                return true;
            }
        }
    }
    return true;
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
    Valter::getInstance()->addIpAddressToRemoteControlDeviceTCPInterfacesIpAddressesVector(commandHostIP);
}

void TCPInterface::sentCDRsRectifierThreadWorker()
{
    while (!sentCDRsRectifierThreadWorkerStopped)
    {
        if (!sentCDRs.empty())
        {
            std::lock_guard<std::mutex> guard(sentCDRs_mutex);
            sentCDRs.erase(sentCDRs.begin());
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    qDebug("STOPPED: TCPInterface::sentCDRsRectifierThreadWorker");
}
