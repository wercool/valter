#include "taskmanager.h"
#include <QtDebug>


TaskManager* TaskManager::pTaskManager = NULL;
bool TaskManager::instanceFlag = false;
std::map<unsigned long, ITask*> queuedTasksMap = {};
ITask *processingTask = NULL;

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

TaskManager::TaskManager()
{
    qDebug("Task Manager initialized");
    queueStopped = false;
    new std::thread(&TaskManager::tasksQueueWorker, this);
}

void TaskManager::addTask(ITask *task)
{
    std::lock_guard<std::mutex> guard(tasks_mutex);
    queuedTasksMap.insert(pair<unsigned long, ITask*>(task->getTaskId(), task));
}

ITask* TaskManager::getTaskById(unsigned long id)
{
    if ( queuedTasksMap.find(id) == queuedTasksMap.end() )
    {
        return NULL;
    }
    else
    {
        return queuedTasksMap[id];
    }
}

ITask *TaskManager::getProcessingTask()
{
    return processingTask;
}

bool TaskManager::getQueueStopped() const
{
    return queueStopped;
}

void TaskManager::setQueueStopped(bool value)
{
    queueStopped = value;
}

ITask *TaskManager::getNextQueuedTask()
{
    processingTask = queuedTasksMap.begin()->second; //fisrt - key(taskId), second - value (ITask)

    if (!processingTask->getExecuting())
    {
        processingTask->setQDebugOn(false);
        processingTask->execute();
    }

    if (!processingTask->getBlocking())
    {
        eraseQueuedCompletedTask();
    }
    else
    {
        if (processingTask->getCompleted())
        {
            eraseQueuedCompletedTask();
        }
    }

    return processingTask;
}

void TaskManager::eraseQueuedCompletedTask()
{
    std::lock_guard<std::mutex> guard(tasks_mutex);
    queuedTasksMap.erase(processingTask->getTaskId());
}

void TaskManager::tasksQueueWorker()
{
    while (!queueStopped)
    {
        if (!queuedTasksMap.empty())
        {
            processingTask = getNextQueuedTask();

            this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        else
        {
            this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
