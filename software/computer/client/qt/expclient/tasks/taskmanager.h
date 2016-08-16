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

    unsigned int routeTaskRequest(std::string taskMessage); //returns TaskId

    ITask *getProcessingTask();

    bool getQueueStopped() const;
    void setQueueStopped(bool value);

    void  wipeQueuedCompletedTaskFromQueue(unsigned long id, bool onlyFromQueuedTasks);

    void processScript(std::string script);
    void clearQueue();

    bool sendScriptToRemoteTaskManager(string script, string ipAddress);

    bool sendMessageToCentralHostTaskManager(string message);

    TCPInterface *getTcpInterface() const;
    void setTcpInterface(TCPInterface *value);

    void initTcpInterface();
    void initTcpCommandAcceptorInterface();

    bool getIncomingScriptProcessing() const;
    void setIncomingScriptProcessing(bool value);

    bool getStopTopTask() const;
    void setStopTopTask(bool value);

    void addUpdateRTMM(string rtmm);
    std::string getRTMMDesc(long taskId);
    void removeRTMM(long taskId);
    std::map<long, string> getRtmms() const;
    void clearRTMM();

private:
    TaskManager();
    static TaskManager *pTaskManager;      // TaskManager's singleton instance
    static bool instanceFlag;

    std::map<unsigned long, ITask*> queuedTasksMap;
    std::map<std::string, ITask*> executingTasksMap;
    std::mutex tasks_mutex;

    ITask *processingTask;

    bool queueStopped;
    bool incomingScriptProcessing;
    bool stopTopTask;

    void tasksQueueWorker(void);

    TCPInterface *tcpInterface;

    std::map<long, string> rtmms;
};

#endif // TASKMANAGER_H
