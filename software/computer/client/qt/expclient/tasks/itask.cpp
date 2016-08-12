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
    attachable = false;

    qDebugOn = true;

    executionWorkerThread = NULL;
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

std::thread *ITask::getExecutionWorkerThread() const
{
    return executionWorkerThread;
}

void ITask::setExecutionWorkerThread(std::thread *value)
{
    executionWorkerThread = value;
}

std::string ITask::getTaskScriptLine() const
{
    return taskScriptLine;
}

void ITask::setTaskScriptLine(const std::string &value)
{
    taskScriptLine = value;
}

bool ITask::getAttachable() const
{
    return attachable;
}

void ITask::setAttachable(bool value)
{
    attachable = value;
}

void ITask::setCompleted()
{
    completed = true;
    executing = false;
}

void ITask::setBlocking(bool value)
{
    blocking = value;
}

bool ITask::getCompleted() const
{
    return completed;
}
