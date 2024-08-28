#include "valter.h"
#include "stopallwatcherstask.h"

StopAllWatchersTask::StopAllWatchersTask()
{
    completed = false;
    taskName = "STOPALLWATCHERS";
    setBlocking(true);
}

bool StopAllWatchersTask::checkFeasibility()
{
    return true;
}

bool StopAllWatchersTask::initialize()
{
    return true;
}

void StopAllWatchersTask::execute()
{
    executing = true;
    new std::thread(&StopAllWatchersTask::executionWorker, this);
}

void StopAllWatchersTask::stopExecution()
{
    stopped = true;
    qDebug("Task#%lu (%s) stopExecution()", getTaskId(), getTaskName().c_str());
}

void StopAllWatchersTask::reportCompletion()
{
    qDebug("Task#%lu (%s) completed.", getTaskId(), getTaskName().c_str());
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
}

void StopAllWatchersTask::executionWorker()
{
    TaskManager::getInstance()->sendMessageToCentralHostTaskManager(Valter::format_string("%lu~%s~%s~%s~%s", getTaskId(), getTaskName().c_str(), (blocking) ? "blocking" : "non blocking", ((stopped) ? "stopped" : ((completed) ? "completed" : ((executing) ? "executing" : "queued"))), getTaskScriptLine().c_str()));
    qDebug("Task#%lu (%s) being executed...", getTaskId(), getTaskName().c_str());

    ArmControlRight *armControlRight = ArmControlRight::getInstance();
    armControlRight->stopAllWatchers();

    ArmControlLeft *armControlLeft = ArmControlLeft::getInstance();
    armControlLeft->stopAllWatchers();

    setCompleted();
}
