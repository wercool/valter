#include "itask.h"

ITask::ITask()
{
    taskId = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    stopped = false;
    executing = false;
    completed = false;
}

unsigned long ITask::getTaskId()
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

void ITask::setCompleted(bool value)
{
    executing = !value;
    completed = value;
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
