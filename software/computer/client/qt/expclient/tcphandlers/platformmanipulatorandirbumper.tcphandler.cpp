#include "valter.h"

class PlatformManipulatorAndIRBumperTCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    PlatformManipulatorAndIRBumperTCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

    void* run()
    {
        // Remove 1 item at a time and process it. Blocks if no items are
        // available to process.
        for (int i = 0;; i++)
        {
            qDebug("thread %lu, loop %d - waiting for item...", (long unsigned int)self(), i);
            WorkItem* item = m_queue.remove();
            qDebug("thread %lu, loop %d - got one item", (long unsigned int)self(), i);
            TCPStream* stream = item->getStream();

            // Echo messages back the client until the connection is
            // closed
            char input[256];
            int len;
            while ((len = stream->receive(input, sizeof(input)-1)) > 0 )
            {
                input[len] = '\0';
                stream->send(input, len);
                qDebug("thread %lu, echoed '%s' back to the client", (long unsigned int)self(), input);
            }
            delete item;
        }

        // Should never get here
        return NULL;
    }
};
