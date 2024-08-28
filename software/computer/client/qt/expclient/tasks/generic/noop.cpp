#include "valter.h"
#include "noop.h"

NoOpTask::NoOpTask()
{
    completed = false;
    taskName = "NOOP";
    setBlocking(true);
}

bool NoOpTask::checkFeasibility()
{
    return true;
}

bool NoOpTask::initialize()
{
    return true;
}

void NoOpTask::execute()
{
    executing = true;
    new std::thread(&NoOpTask::executionWorker, this);
}

void NoOpTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void NoOpTask::reportCompletion()
{
    qDebug("Task#%lu (%s) completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void NoOpTask::executionWorker()
{
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    qDebug("Task#%lu (%s) being executed...", getTaskId(), getTaskName().c_str());
    setCompleted();
}
