#include "itask.h"
#include "taskmanager.h"

ITask::ITask()
{
    taskId = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    taskName = "NO NAME";

    stopped = false;
    executing = false;
    completed = false;
    blocking = false;

    qDebugOn = true;
}

unsigned long ITask::getTaskId() const
{
    return taskId;
}

bool ITask::getExecuting() const
{
    return executing;
}

bool ITask::getBlocking() const
{
    return blocking;
}

bool ITask::getQDebugOn() const
{
    return qDebugOn;
}

void ITask::setQDebugOn(bool value)
{
    qDebugOn = value;
}

std::string ITask::getTaskName() const
{
    return taskName;
}

bool ITask::getStopped() const
{
    return stopped;
}

void ITask::setCompleted(bool value)
{
    completed = value;
    executing = !value;
    TaskManager::getInstance()->wipeQueuedCompletedTaskFromQueue(this->getTaskId());
    reportCompletion();
}

void ITask::setBlocking(bool value)
{
    blocking = value;
}

bool ITask::getCompleted() const
{
    return completed;
}
