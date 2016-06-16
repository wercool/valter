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
    qDebug("Task#%lu %s started", getTaskId(), taskName.c_str());
    this_thread::sleep_for(std::chrono::milliseconds(150));
    Valter::getInstance()->setAllModulesInitialState();
    this_thread::sleep_for(std::chrono::milliseconds(150));
    setCompleted();
    qDebug("Task#%lu %s completed", getTaskId(), taskName.c_str());
}

void SetModuleInitialStateTask::stopExecution()
{

}

void SetModuleInitialStateTask::reportCompletion()
{

}

void SetModuleInitialStateTask::executionWorker()
{

}
