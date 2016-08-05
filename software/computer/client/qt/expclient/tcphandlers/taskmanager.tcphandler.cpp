#include "valter.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qnamespace.h>

class TaskManagerTCPConnectionHandler : public Thread
{
    wqueue<WorkItem*>& m_queue;

  public:
    TaskManagerTCPConnectionHandler(wqueue<WorkItem*>& queue) : m_queue(queue) {}

    void* run()
    {
        // Remove 1 item at a time and process it. Blocks if no items are
        // available to process.
        for (int i = 0;; i++)
        {
            //qDebug("thread %lu, loop %d - waiting for item...", (long unsigned int)self(), i);
            WorkItem* item = m_queue.remove();
            //qDebug("thread %lu, loop %d - got one item", (long unsigned int)self(), i);
            TCPStream* stream = item->getStream();

            // Echo messages back the client until the connection is
            // closed
            char input[10256];
            for (int i = 0; i < 10255; i++)
            {
                input[i] = '\0';
            }

            string output;
            int len;

            while ((len = stream->receive(input, sizeof(input)-1)) > 0 )
            {
                output = "OK";
                stream->send(output.c_str(), (sizeof(output.c_str())-1));
                //qDebug("thread %lu, echoed '%s' back to the client", (long unsigned int)self(), input);
            }

            delete item;

            //RTMM - Remote Task Manager Message
            char isRTMM[5];
            strncpy(isRTMM, input, 4);
            isRTMM[4] = '\0';

            if (strcmp(isRTMM, "RTMM") == 0)
            {
                std::string rtmm(input);
                qDebug("%s", rtmm.c_str());
                TaskManager::getInstance()->addUpdateRTMM(rtmm);
            }
            else
            {
                std::string script(input);

                if (script.length() > 0)
                {
                    executeScript(script);
                }
            }
        }

        // Should never get here
        return NULL;
    }

    void executeScript(string script)
    {
        if (script.find("setCentralCommandHostInfo") != std::string::npos)
        {
            TaskManager *taskManager = TaskManager::getInstance();
            int substr_pos = script.find("@") + 1;
            string value_str = script.substr(substr_pos);
            vector<string>value_str_values = Valter::split(value_str, '@');

            taskManager->getTcpInterface()->setCentralCommandHostIP(value_str_values[0]);
            taskManager->getTcpInterface()->setCentralCommandHostIPPort(atoi(Valter::stringToCharPtr(value_str_values[1])));
            qDebug("[TaskManager] Central Command Host IP Address:%s Port:%d", taskManager->getTcpInterface()->getCentralCommandHostIP().c_str(), taskManager->getTcpInterface()->getCentralCommandHostIPPort());
            return;
        }

        TaskManager::getInstance()->processScript(script);
    }
};
