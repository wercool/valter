#include "valter.h"
#include "platformcontrolp1.h"

class PlatformControlP1ConnectionHandler: Thread
{
    wqueue<WorkItem*>& m_queue;

public:
    PlatformControlP1ConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

    // Thread interface
    void *run()
    {
        return NULL;
    }
}
