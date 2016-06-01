#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <map>
#include <cassert>
#include <mutex>

#include <tasks/itask.h>
#include "tcpinterface.h"

class TaskManager
{
public:
    static TaskManager *getInstance();
    unsigned int addTask(ITask *task);
    ITask* getTaskById(unsigned long id);

    ITask *getProcessingTask();

    bool getQueueStopped() const;
    void setQueueStopped(bool value);

    void  wipeQueuedCompletedTaskFromQueue(unsigned long id);

    void processScript(std::string script);
    void clearQueue();

    bool sendScriptToRemoteTaskManager(string script, string ipAddress);

    unsigned int routeTaskRequest(std::string taskMessage); //returns TaskId

    TCPInterface *getTcpInterface() const;
    void setTcpInterface(TCPInterface *value);

    void initTcpInterface();
    void initTcpCommandAcceptorInterface();

private:
    TaskManager();
    static TaskManager *pTaskManager;      // TaskManager's singleton instance
    static bool instanceFlag;

    std::map<unsigned long, ITask*> queuedTasksMap;
    std::map<std::string, ITask*> executingTasksMap;
    std::mutex tasks_mutex;

    ITask *processingTask;

    bool queueStopped;

    void tasksQueueWorker(void);

    TCPInterface *tcpInterface;
};

#endif // TASKMANAGER_H
