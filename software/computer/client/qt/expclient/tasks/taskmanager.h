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

private:
    TaskManager();
    static TaskManager *pTaskManager;      // TaskManager's singleton instance
    static bool instanceFlag;

    std::map<unsigned long, ITask*> tasksMap;
    std::mutex tasks_add_mutex;
};

#endif // TASKMANAGER_H
