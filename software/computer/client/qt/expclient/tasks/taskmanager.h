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
    void addTask(ITask *task);
    ITask* getTaskById(unsigned long id);

    ITask *getProcessingTask();

    bool getQueueStopped() const;
    void setQueueStopped(bool value);

    ITask *getNextQueuedTask();
    void  eraseQueuedCompletedTask();

private:
    TaskManager();
    static TaskManager *pTaskManager;      // TaskManager's singleton instance
    static bool instanceFlag;

    std::map<unsigned long, ITask*> queuedTasksMap;
    std::mutex tasks_mutex;

    ITask *processingTask;

    bool queueStopped;

    void tasksQueueWorker(void);
};

#endif // TASKMANAGER_H
