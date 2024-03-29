#include "valter.h"
#include "setmoduleinitialstatetask.h"

SetModuleInitialStateTask::SetModuleInitialStateTask()
{
    completed = false;
    taskName = "SetModuleInitialState";
    setBlocking(true);
}

bool SetModuleInitialStateTask::checkFeasibility()
{
    return true;
}

bool SetModuleInitialStateTask::initialize()
{
    return true;
}

void SetModuleInitialStateTask::execute()
{
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    this_thread::sleep_for(std::chrono::milliseconds(150));
    Valter::getInstance()->setAllModulesInitialState();
    this_thread::sleep_for(std::chrono::milliseconds(150));
    setCompleted();
}

void SetModuleInitialStateTask::stopExecution()
{

}

void SetModuleInitialStateTask::reportCompletion()
{
    qDebug("Task#%lu %s completed", getTaskId(), taskName.c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void SetModuleInitialStateTask::executionWorker()
{

}
