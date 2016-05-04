#include "valter.h"
#include "tcpinterface.h"

TCPInterface::TCPInterface()
{
    ip = "127.0.0.1";
}

string TCPInterface::getIp() const
{
    return ip;
}

void TCPInterface::setIp(const string &value)
{
    ip = value;
}
