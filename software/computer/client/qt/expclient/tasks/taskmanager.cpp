#include "taskmanager.h"
#include <QtDebug>


TaskManager* TaskManager::pTaskManager = NULL;
bool TaskManager::instanceFlag = false;
std::map<unsigned long, ITask*> tasksMap = {};

TaskManager *TaskManager::getInstance()
{
    if(!instanceFlag)
    {
        pTaskManager = new TaskManager();
        instanceFlag = true;
        return pTaskManager;
    }
    else
    {
        return pTaskManager;
    }
}

void TaskManager::addTask(ITask *task)
{
    std::lock_guard<std::mutex> guard(tasks_add_mutex);
    tasksMap.insert(pair<unsigned long, ITask*>(task->getTaskId(), task));
}

ITask *TaskManager::getTaskById(unsigned long id)
{
    return tasksMap[id];
}

TaskManager::TaskManager()
{
    qDebug("Task Manager initialized");
}
