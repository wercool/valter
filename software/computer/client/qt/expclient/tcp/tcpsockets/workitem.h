#ifndef WORKITEM_H
#define WORKITEM_H

#include "tcpstream.h"

class WorkItem
{
private:
    TCPStream* m_stream;

public:
    WorkItem(TCPStream* stream) : m_stream(stream) {}
    ~WorkItem() { delete m_stream; }

    TCPStream* getStream() { return m_stream; }
};

#endif // WORKITEM_H
