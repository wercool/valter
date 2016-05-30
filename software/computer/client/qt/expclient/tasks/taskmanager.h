#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <map>
#include <cassert>
#include <mutex>

#include <tasks/itask.h>

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

    unsigned int routeTaskRequest(std::string taskMessage); //returns TaskId

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
};

#endif // TASKMANAGER_H
